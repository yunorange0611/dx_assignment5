#include "arm_control.h"
#include "arm_task.h"
#include "operator_control.h"
#include "yaw_runtime.h"
#include <cmath>
#include <limits>
using namespace engineer;
extern "C" volatile unsigned test_checks;
#define CHECK(x) do { ++test_checks; if(!(x)) return __LINE__; } while(0)
namespace {
Config config() {
    Config c=kArmConfig; c.joint_mask=kAllJoints; c.hardware_confirmed=true;
    c.pitch.mapping_confirmed=true; return c;
}
bool feed(ArmControl &arm,uint32_t ms,unsigned encoder=1000,int rpm=0) {
    const uint8_t d[8]={uint8_t(encoder>>8),uint8_t(encoder),uint8_t(uint16_t(rpm)>>8),uint8_t(rpm),0,0,25,0};
    return arm.receive(1,arm.feedback_id(Yaw),d,8,ms);
}
YawRuntimeLimits limits(uint32_t version=1) { return {-10,10,10,15,2,3,version}; }
bool yaw_zero(const Frame *f,unsigned n) {
    for(unsigned i=0;i<n;++i) if(f[i].id==0x2fe) for(uint8_t b:f[i].data) if(b) return false;
    return true;
}
}
int test_yaw() {
    static_assert(sizeof(ArmStatus::updated_ms)==4);
    Frame frames[kJointCount]{};
    { // No first frame, no calibration, stale/moving/enabled input: atomic refusal.
        ArmControl a(config()); auto l=limits();
        CHECK(a.set_yaw_limits(l,0)==Result::NotOnline);
        CHECK(a.yaw_limits().version==0);
        CHECK(feed(a,0)); CHECK(a.set_yaw_limits(l,0)==Result::NotCalibrated);
        CHECK(a.capture_zero(1,0)==Result::Ok); CHECK(a.set_yaw_limits(l,0)==Result::Ok);
        CHECK(a.yaw_limits().version==1 && a.yaw_limits().max_deg==10);
        l.max_deg=100; CHECK(a.set_yaw_limits(l,0)==Result::InvalidArgument);
        l=limits(); l.braking_margin_deg=10; CHECK(a.set_yaw_limits(l,0)==Result::InvalidArgument);
        l=limits(); l.max_speed_deg_s=std::numeric_limits<float>::quiet_NaN();
        CHECK(a.set_yaw_limits(l,0)==Result::InvalidArgument);
        l=limits(); l.braking_accel_deg_s2=0; CHECK(a.set_yaw_limits(l,0)==Result::InvalidArgument);
        l=limits(); l.version=0; CHECK(a.set_yaw_limits(l,0)==Result::InvalidArgument);
        CHECK(a.yaw_limits().version==1);
        CHECK(a.enable(1,0)==Result::Ok);
        CHECK(a.set_yaw_limits(limits(2),0)==Result::Busy);
        a.disable(); CHECK(feed(a,4,1000,1)); CHECK(a.set_yaw_limits(limits(2),4)==Result::Moving);
        CHECK(feed(a,8)); CHECK(a.set_yaw_limits(limits(2),8)==Result::Ok);
        CHECK(a.set_yaw_limits(limits(3),100)==Result::NotOnline);
        CHECK(a.yaw_limits().version==2);
    }
    { // Shrinking outside actual/target/reference is rejected, old boundary remains.
        ArmControl a(config()); feed(a,0); a.capture_zero(1,0);
        CHECK(a.set_yaw_limits(limits(),0)==Result::Ok);
        CHECK(feed(a,4,1137)); // About +6 degrees, zero-speed feedback.
        auto small=limits(2); small.min_deg=-4; small.max_deg=4; small.braking_margin_deg=1;
        CHECK(a.set_yaw_limits(small,4)==Result::OutOfRange);
        CHECK(a.yaw_limits().max_deg==10 && a.yaw_limits().version==1);
        ArmControl b(config()); feed(b,0); b.capture_zero(1,0); b.set_yaw_limits(limits(),0);
        float q[kJointCount]={3,0,0,0,0}; CHECK(b.set_targets(1,q,0)==Result::Ok);
        small.min_deg=-3; small.max_deg=3; small.braking_margin_deg=1;
        CHECK(b.set_yaw_limits(small,0)==Result::OutOfRange); // Target would be excluded by margin.
    }
    { // Lead bound is enforced in the kernel, including direct target API callers.
        ArmControl a(config()); feed(a,0); a.capture_zero(1,0); a.set_yaw_limits(limits(),0);
        CHECK(a.enable(1,0)==Result::Ok);
        float q[kJointCount]={80,0,0,0,0}; CHECK(a.set_targets(1,q,0)==Result::Ok);
        CHECK(std::fabs(a.status().joint[Yaw].target_deg-3)<.001f);
        CHECK(a.set_yaw_envelope(-4,4,1)==Result::Ok);
        for(uint32_t t=0;t<120;t+=4) {
            CHECK(feed(a,t)); CHECK(a.set_targets(1,q,t)==Result::Ok); a.step(t,frames);
            CHECK(std::fabs(a.status().joint[Yaw].velocity_target_deg_s)<=1.0001f);
            CHECK(std::fabs(a.status().joint[Yaw].reference_deg)<=3.0001f);
        }
        CHECK(a.set_yaw_envelope(-2,2,1)==Result::OutOfRange); // Existing target outside new envelope.
        CHECK(a.set_yaw_envelope(-11,11,1)==Result::InvalidArgument);
    }
    { // Actual outward velocity near the wire boundary commands inward braking.
        ArmControl a(config()); feed(a,0); a.capture_zero(1,0); a.set_yaw_limits(limits(),0); a.enable(1,0);
        CHECK(feed(a,4,1205,1)); // +9deg and +6deg/s; within the old range, inside stopping margin.
        float q[kJointCount]={10,0,0,0,0}; a.set_targets(1,q,4); a.step(4,frames);
        CHECK(a.status().joint[Yaw].velocity_target_deg_s<=0.0001f);
        CHECK(a.status().joint[Yaw].effort<0); // Brake is retained, not blanket-zeroed.
        CHECK(!yaw_zero(frames,4));
        a.disable();
        ArmControl b(config()); feed(b,0); b.capture_zero(1,0); b.set_yaw_limits(limits(),0); b.enable(1,0);
        CHECK(feed(b,4,795,-1));
        q[0]=-10; b.set_targets(1,q,4); b.step(4,frames);
        CHECK(b.status().joint[Yaw].velocity_target_deg_s>=-.0001f);
        CHECK(b.status().joint[Yaw].effort>0);
    }
    { // The owner-only tuning path still uses the same velocity PI and bounds.
        ArmControl a(config()); feed(a,0); a.capture_zero(1,0); a.set_yaw_limits(limits(),0);
        CHECK(a.set_yaw_test_speed(1,0)==Result::InvalidArgument);
        CHECK(a.enable(1,0)==Result::Ok); CHECK(a.set_effort_limit(Yaw,150)==Result::Ok);
        CHECK(a.set_yaw_test_speed(6,0)==Result::InvalidArgument);
        CHECK(a.set_yaw_test_speed(2,0)==Result::Ok); a.step(0,frames);
        CHECK(a.status().joint[Yaw].velocity_target_deg_s==2);
        CHECK(a.status().joint[Yaw].effort>0 && a.status().joint[Yaw].effort<=150);
        CHECK(feed(a,4,1205,1)); CHECK(a.set_yaw_test_speed(2,4)==Result::Ok); a.step(4,frames);
        CHECK(a.status().joint[Yaw].velocity_target_deg_s<=0);
        CHECK(a.status().joint[Yaw].effort<0);
        a.disable(); CHECK(a.set_yaw_test_speed(1,4)==Result::InvalidArgument);
    }
    { // Four-axis bench never registers/enables yaw through the new core API.
        auto c=config(); c.joint_mask=kFourAxisJoints; ArmControl a(c);
        CHECK(a.set_yaw_limits(limits(),0)==Result::InvalidArgument);
        CHECK(a.set_yaw_envelope(-1,1,1)==Result::InvalidArgument);
        const unsigned n=a.step(0,frames);
        for(unsigned i=0;i<n;++i) CHECK(frames[i].id!=0x2fe);
    }
    { // Operator and inner loop consume the same effective limit version.
        auto c=config(); OperatorConfig rc=kOperatorConfig; rc.mapping_confirmed=true;
        ArmOperator op(c,rc); op.set_yaw_limits(limits(7));
        Status s{}; s.configuration_valid=true;
        for(auto &j:s.joint) { j.online=true; j.calibrated=true; }
        RemoteInput r{}; r.valid=true; r.frame.switches[rc.left_switch_index]=DR16_SWITCH_MID;
        r.frame.switches[rc.right_switch_index]=DR16_SWITCH_MID;
        Interlocks in; in.valid=true; in.chassis_stopped=true; in.arm_allows_drive=true;
        OperatorOutput o;
        for(uint32_t t=0;t<=400;t+=4) {
            r.received_ms=t; in.received_ms=t; o=op.step(r,in,s,t,t==400,false);
        }
        CHECK(o.active); s.enabled_mask=kAllJoints;
        r.frame.channel[rc.right_x_channel]=660;
        for(uint32_t t=404;t<=2000;t+=4) {
            r.received_ms=t; in.received_ms=t; o=op.step(r,in,s,t,false,false);
            CHECK(o.target_deg[Yaw]<=3.0001f && o.target_deg[Yaw]>=-3.0001f);
        }
    }
    { // Bounded pulse: no first-frame delayed start, refresh extension, or automatic recovery.
        YawBenchGate gate;
        YawBenchRequest r{1,2,500,0x59415753,1}; YawBenchHealth h;
        h.stationary=true; CHECK(!gate.step(0,r,h).active);
        h.ready=true; CHECK(!gate.step(4,r,h).active); // Rejected sequence already consumed.
        r.request_seq=2; CHECK(gate.step(8,r,h).active);
        h.enabled=true; h.stationary=false; CHECK(gate.step(12,r,h).active);
        r.request_seq=3; CHECK(!gate.step(16,r,h).active); // New request cancels, cannot extend.
        h.enabled=false; h.stationary=true; r.request_seq=4; CHECK(gate.step(20,r,h).active);
        h.enabled=true; h.stop_epoch=1; CHECK(!gate.step(24,r,h).active);
        h.enabled=false; CHECK(!gate.step(28,r,h).active);
        r.request_seq=5; h.stop=true; CHECK(!gate.step(32,r,h).active);
        h.stop=false; CHECK(!gate.step(36,r,h).active);
        r.request_seq=6; r.speed_deg_s=std::numeric_limits<float>::quiet_NaN(); CHECK(!gate.step(40,r,h).active);
        r.request_seq=7; r.speed_deg_s=2; r.duration_ms=501; CHECK(!gate.step(44,r,h).active);
        r.request_seq=8; r.duration_ms=20; CHECK(gate.step(0xfffffff0u,r,h).active);
        h.enabled=true; CHECK(gate.step(0xfffffff4u,r,h).active); CHECK(!gate.step(4,r,h).active);
        h.enabled=false; CHECK(!gate.step(8,r,h).active);
    }
    return 0;
}
