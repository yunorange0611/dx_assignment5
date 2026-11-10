#include "chassis_control.h"
#include <cmath>
namespace engineer {
namespace {
float clamp(float x,float lo,float hi) { return std::fmax(lo,std::fmin(x,hi)); }
float approach(float x,float goal,float step) { return x+clamp(goal-x,-step,step); }
}
ChassisControl::ChassisControl(const ChassisConfig &c):c_(c) {
    const float values[]={c.diameter_m,c.track_m,c.gear_ratio,c.max_speed_m_s,c.max_turn_rad_s,
        c.acceleration_m_s2,c.deceleration_m_s2,c.angular_accel_rad_s2,c.max_rotor_rpm,
        c.max_current_ma,c.velocity_kp,c.stationary_rpm,c.max_temperature_c};
    bool good=true; for(float x:values) good &= std::isfinite(x) && x>0;
    good &= c.motor_id[0]>=1 && c.motor_id[0]<=4 && c.motor_id[1]>=1 && c.motor_id[1]<=4 && c.motor_id[0]!=c.motor_id[1];
    for(int d:c.direction) good &= d==1 || d==-1;
    good &= std::isfinite(c.velocity_ki) && c.velocity_ki>=0;
    good &= c.period_ms>0 && c.feedback_timeout_ms>c.period_ms && c.command_timeout_ms>c.period_ms && c.stationary_ms>0 && c.max_current_ma<=20000;
    s_.configuration_valid=good; if(!good) trip(ChassisConfigFault);
}
void ChassisControl::differential(float v,float w,float d,float b,float ratio,float rpm[2]) {
    const float scale=60.0f*ratio/(3.14159265358979323846f*d);
    rpm[0]=(v-w*b*.5f)*scale; rpm[1]=(v+w*b*.5f)*scale;
}
void ChassisControl::trip(uint32_t reason) {
    if(s_.enabled || reason) rearm_required_=true;
    s_.faults|=reason; s_.enabled=false;
    s_.reference_v_m_s=s_.reference_omega_rad_s=0;
    for(unsigned i=0;i<2;++i) { integral_[i]=0; s_.candidate_ma[i]=s_.applied_ma[i]=s_.target_rpm[i]=0; }
}
bool ChassisControl::clear_faults() {
    if(s_.enabled || !s_.configuration_valid) return false;
    s_.faults=0; have_epoch_=false; return true;
}
const ChassisState &ChassisControl::step(uint32_t now,const WheelFeedback wheel[2],const ChassisCommand &cmd) {
    const bool reset_edge=cmd.reset && !previous_reset_;
    previous_reset_=cmd.reset;
    if(!cmd.enabled) rearm_required_=false; // Must observe explicit disabled input before rearm.
    const uint32_t elapsed=have_step_ ? uint32_t(now-previous_ms_) : c_.period_ms;
    previous_ms_=now; have_step_=true; s_.updated_ms=now;
    const bool timing=elapsed>0 && elapsed<=c_.period_ms+2;
    dt_=timing ? elapsed*.001f : 0;
    bool feedback=true,temp=true;
    for(unsigned i=0;i<2;++i) {
        feedback &= wheel[i].frames && uint32_t(now-wheel[i].received_ms)<=c_.feedback_timeout_ms && std::isfinite(wheel[i].rotor_rpm);
        temp &= std::isfinite(wheel[i].temperature_c) && wheel[i].temperature_c<c_.max_temperature_c;
        s_.measured_rpm[i]=wheel[i].rotor_rpm*c_.direction[i];
    }
    const float scale=3.14159265358979323846f*c_.diameter_m/(60*c_.gear_ratio);
    const float vl=s_.measured_rpm[0]*scale,vr=s_.measured_rpm[1]*scale;
    s_.v_m_s=(vl+vr)*.5f; s_.omega_rad_s=(vr-vl)/c_.track_m;
    const bool still=feedback && std::fabs(s_.measured_rpm[0])<c_.stationary_rpm && std::fabs(s_.measured_rpm[1])<c_.stationary_rpm;
    if(!still) tracking_stationary_=false;
    else if(!tracking_stationary_) { tracking_stationary_=true; stationary_since_=now; }
    s_.wheels_stationary=still && uint32_t(now-stationary_since_)>=c_.stationary_ms;
    if(s_.enabled) {
        if(!timing) trip(ChassisTimingFault);
        if(!feedback) trip(ChassisFeedbackFault);
        if(!temp) trip(ChassisTemperatureFault);
    }
    const bool fresh=cmd.valid && uint32_t(now-cmd.updated_ms)<=c_.command_timeout_ms;
    const bool finite=std::isfinite(cmd.speed_request) && std::isfinite(cmd.turn_request) && std::fabs(cmd.speed_request)<=1 && std::fabs(cmd.turn_request)<=1;
    if(s_.enabled && (!fresh || !finite)) trip(ChassisCommandFault);
    if(!c_.hardware_confirmed || !s_.configuration_valid || !fresh || !finite || !cmd.enabled || !feedback || !temp || !timing || s_.faults || rearm_required_) {
        trip(0);
        if(cmd.enabled) rearm_required_=true; // Rejected enable requests cannot become delayed starts.
        else rearm_required_=false;
        return s_;
    }
    if(!s_.enabled || !have_epoch_ || cmd.epoch!=epoch_ || reset_edge) {
        s_.reference_v_m_s=s_.v_m_s; s_.reference_omega_rad_s=s_.omega_rad_s;
        for(float &i:integral_) i=0;
    }
    epoch_=cmd.epoch; have_epoch_=true; s_.enabled=true;
    const float v=(cmd.stop || cmd.brake) ? 0 : cmd.speed_request*c_.max_speed_m_s;
    const float w=cmd.stop ? 0 : cmd.turn_request*c_.max_turn_rad_s;
    // Reverse through zero first, use the measured state on mode transitions.
    const float goal=s_.reference_v_m_s*v<0 ? 0 : v;
    const float accel=std::fabs(goal)>std::fabs(s_.reference_v_m_s) ? c_.acceleration_m_s2 : c_.deceleration_m_s2;
    s_.reference_v_m_s=approach(s_.reference_v_m_s,goal,accel*dt_);
    s_.reference_omega_rad_s=approach(s_.reference_omega_rad_s,w,c_.angular_accel_rad_s2*dt_);
    differential(s_.reference_v_m_s,s_.reference_omega_rad_s,c_.diameter_m,c_.track_m,c_.gear_ratio,s_.target_rpm);
    const float peak=std::fmax(std::fabs(s_.target_rpm[0]),std::fabs(s_.target_rpm[1]));
    if(peak>c_.max_rotor_rpm) for(float &r:s_.target_rpm) r*=c_.max_rotor_rpm/peak;
    for(unsigned i=0;i<2;++i) {
        error_[i]=s_.target_rpm[i]-s_.measured_rpm[i];
        s_.candidate_ma[i]=clamp(c_.velocity_kp*error_[i]+integral_[i],-c_.max_current_ma,c_.max_current_ma);
    }
    return s_;
}
void ChassisControl::apply(const float ma[2], bool submitted) {
    if(!submitted) { trip(ChassisTransportFault); return; }
    if(!s_.enabled) return;
    if(!std::isfinite(ma[0]) || !std::isfinite(ma[1])) { trip(ChassisCommandFault); return; }
    for(unsigned i=0;i<2;++i) {
        s_.applied_ma[i]=clamp(ma[i],-c_.max_current_ma,c_.max_current_ma);
        const float raw=c_.velocity_kp*error_[i]+integral_[i];
        if((raw-s_.applied_ma[i])*error_[i]<=0)
            integral_[i]=clamp(integral_[i]+c_.velocity_ki*error_[i]*dt_,-c_.max_current_ma,c_.max_current_ma);
    }
}
void ChassisControl::direct_output() {
    for(unsigned i=0;i<2;++i) { integral_[i]=0; error_[i]=0; s_.target_rpm[i]=0; }
}
void ChassisControl::step_debug_speed(unsigned wheel,float rpm) {
    if(!s_.enabled || wheel>1 || !std::isfinite(rpm) || std::fabs(rpm)>300) return;
    for(unsigned i=0;i<2;++i) {
        s_.target_rpm[i]=i==wheel ? rpm : 0;
        error_[i]=s_.target_rpm[i]-s_.measured_rpm[i];
        s_.candidate_ma[i]=i==wheel ? clamp(c_.velocity_kp*error_[i]+integral_[i],-std::fmin(300,c_.max_current_ma),std::fmin(300,c_.max_current_ma)) : 0;
        if(i!=wheel) { error_[i]=0; integral_[i]=0; }
    }
}
}
