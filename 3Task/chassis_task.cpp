#include "chassis_task.h"
#include "chassis_control.h"
#include "chassis_bench.h"
#include "power_meter.h"
#include "robot_control.h"
#include "arm_task.h"
#include "operator_control.h"
#include "dr16_bsp.h"
#include "CAN_bsp.h"
#include <cmath>
#include <cstring>
extern "C" {
ChassisStatus g_chassis_status{};
volatile ChassisDebugRequest g_chassis_debug_request{};
ChassisDebugStatus g_chassis_debug_status{};
}
namespace {
using namespace engineer;
#if !ARM_FOUR_AXIS_BENCH || CHASSIS_SUSPENDED_BENCH
struct Rx { uint16_t id; uint8_t len,data[8]; uint32_t ms; };
osMessageQueueId_t rx_queue=nullptr;
volatile uint32_t drops=0;
void on_rx(can_device_receive *dev,uint8_t *d) {
    Rx f{}; f.ms=HAL_GetTick(); f.len=dev->rx_dlc;
    f.id=dev->matches(&hcan2,0x31) ? 0x31 :
        (dev->matches(&hcan2,0x200+kChassisConfig.motor_id[0]) ? 0x200+kChassisConfig.motor_id[0] : 0x200+kChassisConfig.motor_id[1]);
    std::memcpy(f.data,d,8);
    if(rx_queue && osMessageQueuePut(rx_queue,&f,0,0)!=osOK) ++drops;
}
#endif
}
extern "C" int Chassis_GetStatus(ChassisStatus *out) {
    if(!out) return 0;
    taskENTER_CRITICAL(); *out=g_chassis_status; taskEXIT_CRITICAL(); return out->ready;
}
extern "C" void chassis_task(void *) {
#if ARM_FOUR_AXIS_BENCH && !CHASSIS_SUSPENDED_BENCH
    for(;;) osDelay(100); // No CAN2 motor registration or output in the normal arm bench.
#else
    using namespace engineer;
    rx_queue=osMessageQueueNew(64,sizeof(Rx),nullptr);
    if(!rx_queue || osKernelGetTickFreq()!=1000) Error_Handler();
    ChassisMotor motors[2]; PowerMeter power; ChassisControl control(kChassisConfig);
    if(!control.status().configuration_valid) {
        taskENTER_CRITICAL();
        g_chassis_status.ready=1; g_chassis_status.faults=ChassisConfigFault;
        g_chassis_status.updated_ms=HAL_GetTick();
        taskEXIT_CRITICAL();
        for(;;) osDelay(100);
    }
    can_device_receive left(&hcan2,0x200+kChassisConfig.motor_id[0],on_rx);
    can_device_receive right(&hcan2,0x200+kChassisConfig.motor_id[1],on_rx);
    can_device_receive meter(&hcan2,0x31,on_rx);
    uint32_t errors=can_tx_error_count[1],old_drops=0,tick=osKernelGetTickCount();
    bool bench_active=false,previous_enabled=false;
#if CHASSIS_SUSPENDED_BENCH
    ChassisBenchGate bench_gate;
#else
    uint32_t seq=0;
    bool local_stop_latched=false;
#endif
    uint32_t previous_stop_epoch=0;
    for(;;) {
        Rx f;
        for(unsigned n=0;n<64 && osMessageQueueGet(rx_queue,&f,nullptr,0)==osOK;++n) {
            if(f.id==0x31) power.receive(f.data,f.len,f.ms);
            else for(unsigned i=0;i<2;++i) if(f.id==0x200+kChassisConfig.motor_id[i]) motors[i].receive(f.data,f.len,f.ms);
        }
        const uint32_t now=HAL_GetTick();
        const bool transport=errors!=can_tx_error_count[1] || drops!=old_drops || (hcan2.Instance->ESR & CAN_ESR_BOFF);
        if(transport) { control.trip(ChassisTransportFault); bench_active=false; can_cancel_pending(2); }
        errors=can_tx_error_count[1]; old_drops=drops;
        RobotControlStatus robot{}; Robot_GetControlStatus(&robot);
        const bool explicit_stop=robot.stop_epoch!=previous_stop_epoch;
        previous_stop_epoch=robot.stop_epoch;
        if(explicit_stop) { bench_active=false; can_cancel_pending(2); }
        ChassisCommand cmd{}; cmd.updated_ms=robot.updated_ms; cmd.epoch=robot.motion_epoch;
        cmd.valid=robot.ready && robot.joint_mask==ARM_ALL_JOINTS;
        cmd.enabled=kVehicleMotionImplemented && robot.active && !robot.emergency_stop;
        cmd.stop=robot.stop_chassis; cmd.brake=robot.longitudinal_brake; cmd.reset=robot.reset_chassis_motion;
        cmd.speed_request=robot.longitudinal_speed_request; cmd.turn_request=robot.turn_rate_request;
        ChassisDebugRequest request{};
        taskENTER_CRITICAL();
        request.mode=g_chassis_debug_request.mode; request.wheel=g_chassis_debug_request.wheel;
        request.duration_ms=g_chassis_debug_request.duration_ms; request.value=g_chassis_debug_request.value;
        request.suspended_confirm=g_chassis_debug_request.suspended_confirm;
        request.request_seq=g_chassis_debug_request.request_seq;
        taskEXIT_CRITICAL();
        DR16Status rc{}; DR16_GetStatus(&rc);
        bool deadman=kOperatorConfig.mapping_confirmed && rc.valid && rc.consecutive_frames>=3 && uint32_t(now-rc.received_ms)<=100;
        for(int ch:rc.frame.channel) deadman &= std::abs(ch)<=30;
        deadman &= rc.frame.switches[kOperatorConfig.left_switch_index]==DR16_SWITCH_UP &&
                   rc.frame.switches[kOperatorConfig.right_switch_index]==DR16_SWITCH_MID;
        ArmStatus arm{}; Arm_GetStatus(&arm);
        deadman &= arm.ready && arm.enabled_mask==0 && robot.ready && uint32_t(now-robot.updated_ms)<=40;
        WheelFeedback wheels[2]={motors[0].status(),motors[1].status()};
#if CHASSIS_SUSPENDED_BENCH
        ChassisBenchRequest pulse{request.mode,request.wheel,request.duration_ms,request.value,
                                   request.suspended_confirm,request.request_seq};
        ChassisBenchHealth health{};
        health.hardware_confirmed=kChassisConfig.hardware_confirmed;
        health.mapping_confirmed=kOperatorConfig.mapping_confirmed;
        health.remote_ok=deadman; health.owner_idle=arm.ready && !arm.enabled_mask;
        health.feedback_fresh=true; health.temperature_ok=true; health.wheels_stationary=true;
        for(unsigned i=0;i<2;++i) {
            health.feedback_fresh &= wheels[i].frames && uint32_t(now-wheels[i].received_ms)<=kChassisConfig.feedback_timeout_ms;
            health.temperature_ok &= wheels[i].temperature_c<kChassisConfig.max_temperature_c;
            health.wheels_stationary &= std::fabs(wheels[i].rotor_rpm)<kChassisConfig.stationary_rpm;
        }
        health.transport_ok=!transport; health.timing_ok=!g_chassis_status.ready || uint32_t(now-g_chassis_status.updated_ms)<=kChassisConfig.period_ms+2;
        health.controller_faulted=control.status().faults!=0;
        health.controller_enabled=control.status().enabled;
        health.global_stop=explicit_stop; health.stop_epoch=robot.stop_epoch;
        const auto &bench=bench_gate.step(now,pulse,health);
        bench_active=bench.active;
        if(bench.clear_requested) { control.trip(0); control.clear_faults(); }
        g_chassis_debug_status={bench.consumed_seq,uint32_t(bench.result),bench.until_ms};
        cmd={}; cmd.valid=true; cmd.updated_ms=now; cmd.enabled=bench_active;
        cmd.stop=true; cmd.brake=true; cmd.reset=false;
#else
        if(request.request_seq!=seq) {
            seq=request.request_seq;
            g_chassis_debug_status={seq,uint32_t(ChassisBenchResult::InvalidRequest),0};
            if(request.mode==CHASSIS_DEBUG_STOP || request.mode==CHASSIS_DEBUG_CLEAR) {
                local_stop_latched=true; cmd.enabled=false; control.trip(0); can_cancel_pending(2);
                Robot_StopControl();
                if(request.mode==CHASSIS_DEBUG_CLEAR) control.clear_faults();
            }
        }
        if(local_stop_latched) {
            cmd.enabled=false;
            if(!robot.active) local_stop_latched=false;
        }
#endif
        control.step(now,wheels,cmd);
        float final_ma[2]={control.status().candidate_ma[0],control.status().candidate_ma[1]};
#if CHASSIS_SUSPENDED_BENCH
        if(control.status().enabled && bench_active) {
            if(bench.mode==CHASSIS_DEBUG_CURRENT) {
                control.direct_output(); final_ma[0]=final_ma[1]=0;
                final_ma[bench.wheel]=std::fmax(-kChassisConfig.max_current_ma,std::fmin(kChassisConfig.max_current_ma,bench.value));
            }
            else {
                // Reuse the same wheel PI; an explicit wheel target is applied below by the controller.
                control.step_debug_speed(bench.wheel,bench.value);
                final_ma[0]=control.status().candidate_ma[0]; final_ma[1]=control.status().candidate_ma[1];
            }
        }
#endif
        if(!control.status().enabled) final_ma[0]=final_ma[1]=0;
        if(previous_enabled && !control.status().enabled) can_cancel_pending(2);
        uint8_t data[8]; ChassisMotor::pack_current(final_ma[0],final_ma[1],kChassisConfig.motor_id,kChassisConfig.direction,data);
        can_device_transmit tx(&hcan2,0x200,8,data);
        const bool sent=tx.can_send_msg()==osOK; control.apply(final_ma,sent);
        if(!sent) { bench_active=false; can_cancel_pending(2); }
        previous_enabled=control.status().enabled;
        ChassisStatus next{}; const auto &s=control.status(); const auto &p=power.status();
        next.ready=1; next.updated_ms=now; next.configuration_valid=s.configuration_valid;
        next.enabled=s.enabled; next.wheels_stationary=s.wheels_stationary; next.bench_active=bench_active && s.enabled;
        next.faults=s.faults; next.rx_dropped=drops; next.tx_errors=can_tx_error_count[1];
        next.v_m_s=s.v_m_s; next.omega_rad_s=s.omega_rad_s;
        next.power_w=p.power_w; next.energy_j=p.energy_j; next.power_received_ms=p.received_ms; next.power_frames=p.frames;
        for(unsigned i=0;i<2;++i) {
            next.measured_rpm[i]=s.measured_rpm[i]; next.target_rpm[i]=s.target_rpm[i];
            next.current_ma[i]=s.applied_ma[i]; next.feedback_ms[i]=wheels[i].received_ms;
            next.feedback_frames[i]=wheels[i].frames; next.temperature_c[i]=wheels[i].temperature_c;
        }
        taskENTER_CRITICAL(); g_chassis_status=next; taskEXIT_CRITICAL();
        const uint32_t current=osKernelGetTickCount();
        tick=int32_t(current-tick)>0 ? current+kChassisConfig.period_ms : tick+kChassisConfig.period_ms;
        osDelayUntil(tick);
    }
#endif
}
