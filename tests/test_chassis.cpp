#include "chassis_control.h"
#include "power_meter.h"
#include "chassis_bench.h"
#include <cmath>
#include <limits>

using namespace engineer;
extern "C" volatile unsigned test_checks;
#define CHECK(x) do { ++test_checks; if(!(x)) return __LINE__; } while(0)
namespace {
bool near(float a,float b,float tolerance=.001f) { return std::fabs(a-b)<=tolerance; }
ChassisConfig confirmed() { ChassisConfig c{}; c.hardware_confirmed=true; return c; }
ChassisCommand request(uint32_t now,float speed=0,float turn=0) {
    ChassisCommand c{}; c.valid=c.enabled=true; c.updated_ms=now;
    c.stop=c.brake=c.reset=false; c.speed_request=speed; c.turn_request=turn; return c;
}
void feedback(WheelFeedback wheel[2],uint32_t now,float left=0,float right=0) {
    for(unsigned i=0;i<2;++i) { wheel[i].received_ms=now; ++wheel[i].frames; wheel[i].temperature_c=25; }
    wheel[0].rotor_rpm=left; wheel[1].rotor_rpm=right;
}
int16_t be16(const uint8_t *d) { return int16_t(uint16_t(d[0])<<8 | d[1]); }
}
int test_chassis() {
    { // Finite suspended pulses: explicit acknowledgement, all health gates, no stale starts.
        ChassisBenchHealth healthy{};
        healthy.hardware_confirmed=healthy.mapping_confirmed=healthy.remote_ok=true;
        healthy.owner_idle=healthy.feedback_fresh=healthy.temperature_ok=true;
        healthy.wheels_stationary=healthy.transport_ok=healthy.timing_ok=true;
        ChassisBenchRequest r{}; r.mode=uint32_t(ChassisBenchMode::Speed); r.wheel=1;
        r.value=100; r.duration_ms=100; r.suspended_confirm=0x53555350; r.request_seq=1;
        for(unsigned gate=0;gate<12;++gate) {
            ChassisBenchGate g; auto h=healthy;
            if(gate==0) h.hardware_confirmed=false;
            if(gate==1) h.mapping_confirmed=false;
            if(gate==2) h.remote_ok=false;
            if(gate==3) h.owner_idle=false;
            if(gate==4) h.feedback_fresh=false;
            if(gate==5) h.temperature_ok=false;
            if(gate==6) h.wheels_stationary=false;
            if(gate==7) h.transport_ok=false;
            if(gate==8) h.timing_ok=false;
            if(gate==9) h.controller_faulted=true;
            if(gate==10) h.controller_enabled=true;
            if(gate==11) h.global_stop=true;
            CHECK(!g.step(0,r,h).active);
            CHECK(!g.step(4,r,healthy).active); // Old rejected sequence is never queued for later.
        }
        for(unsigned bad=0;bad<8;++bad) {
            ChassisBenchGate g; auto q=r;
            if(bad==0) q.mode=99;
            if(bad==1) q.wheel=2;
            if(bad==2) q.duration_ms=0;
            if(bad==3) q.duration_ms=501;
            if(bad==4) q.value=301;
            if(bad==5) q.value=std::numeric_limits<float>::quiet_NaN();
            if(bad==6) q.value=std::numeric_limits<float>::infinity();
            if(bad==7) q.suspended_confirm=0;
            CHECK(!g.step(0,q,healthy).active);
            CHECK(g.status().result==ChassisBenchResult::InvalidRequest);
        }
        ChassisBenchGate g; CHECK(g.step(10,r,healthy).active);
        CHECK(g.status().until_ms==110 && g.status().value==100 && g.status().wheel==1);
        auto active=healthy; active.controller_enabled=true;
        CHECK(g.step(14,r,active).active && g.status().until_ms==110);
        CHECK(g.step(109,r,active).active); CHECK(!g.step(110,r,active).active);
        CHECK(g.status().result==ChassisBenchResult::Expired);
        CHECK(!g.step(114,r,healthy).active);
        r.request_seq=2; CHECK(g.step(118,r,healthy).active);
        r.request_seq=3; CHECK(!g.step(122,r,active).active && g.status().result==ChassisBenchResult::Busy);
        CHECK(!g.step(126,r,healthy).active);
        r.request_seq=4; CHECK(g.step(130,r,healthy).active);
        active.global_stop=true; CHECK(!g.step(134,r,active).active);
        CHECK(!g.step(138,r,healthy).active);
        r.request_seq=5; CHECK(g.step(142,r,healthy).active);
        active=healthy; active.controller_enabled=true; active.stop_epoch=1;
        r.request_seq=6; CHECK(!g.step(146,r,active).active); // Same-cycle new pulse cannot outrank stop.
        CHECK(!g.step(150,r,healthy).active);
        ChassisBenchGate wrap; r.request_seq=1; r.duration_ms=8;
        CHECK(wrap.step(0xfffffffcu,r,healthy).active);
        active=healthy; active.controller_enabled=true;
        CHECK(wrap.step(0,r,active).active); CHECK(!wrap.step(4,r,active).active);
        CHECK(wrap.status().result==ChassisBenchResult::Expired);
        ChassisBenchRequest clear{}; clear.mode=uint32_t(ChassisBenchMode::Clear); clear.request_seq=1;
        ChassisBenchGate fault; auto fault_health=healthy; fault_health.controller_faulted=true;
        CHECK(fault.step(0,clear,fault_health).clear_requested && !fault.status().active);
        CHECK(!fault.step(4,clear,fault_health).clear_requested && !fault.status().active);
        clear.request_seq=2; clear.mode=uint32_t(ChassisBenchMode::Stop);
        CHECK(!fault.step(8,clear,active).active && fault.status().result==ChassisBenchResult::Stopped);
    }
    { // External 1:1 does not erase the M3508 internal reduction; CCW left/right signs.
        float rpm[2]; const float scale=60*19/(3.14159265358979323846f*.145f);
        ChassisControl::differential(.1f,0,.145f,.330f,19,rpm);
        CHECK(near(rpm[0],.1f*scale) && near(rpm[1],.1f*scale));
        ChassisControl::differential(0,1,.145f,.330f,19,rpm);
        CHECK(near(rpm[0],-.165f*scale) && near(rpm[1],.165f*scale));
        ChassisControl::differential(-.1f,-.2f,.145f,.330f,19,rpm);
        CHECK(near(rpm[0],(-.1f+.033f)*scale) && near(rpm[1],(-.1f-.033f)*scale));
    }
    { // C620 feedback is signed big-endian, invalid DLC/encoder cannot refresh health.
        ChassisMotor m; CHECK(m.status().frames==0);
        uint8_t d[8]={0x1f,0xff,0xff,0x9c,0xc0,0x00,41,0};
        CHECK(!m.receive(nullptr,8,4)); CHECK(!m.receive(d,7,4));
        CHECK(m.receive(d,8,7)); CHECK(m.status().frames==1 && m.status().received_ms==7);
        CHECK(m.status().encoder==8191 && m.status().rotor_rpm==-100);
        CHECK(near(m.status().current_ma,-20000) && m.status().temperature_c==41);
        d[0]=0x20; d[1]=0; CHECK(!m.receive(d,8,11));
        CHECK(m.status().frames==1 && m.status().received_ms==7);
        const uint8_t ids[2]={2,4}; const int8_t signs[2]={-1,1}; uint8_t out[8];
        ChassisMotor::pack_current(20000,-20000,ids,signs,out);
        CHECK(be16(out)==0 && be16(out+2)==-16384 && be16(out+4)==0 && be16(out+6)==-16384);
        ChassisMotor::pack_current(40000,0,ids,signs,out); CHECK(be16(out+2)==-16384 && be16(out+6)==0);
        ChassisMotor::pack_current(std::numeric_limits<float>::quiet_NaN(),0,ids,signs,out);
        for(unsigned b=0;b<8;++b) CHECK(out[b]==0);
    }
    { // Power meter: signed little-endian tenths; negative energy must stay negative.
        PowerMeter p; CHECK(p.status().frames==0);
        const uint8_t d[8]={0xf4,0x01,0xf6,0xff,0,0,0,0};
        CHECK(!p.receive(d,8,8)); CHECK(!p.receive(nullptr,4,8));
        CHECK(p.receive(d,4,9)); CHECK(near(p.status().power_w,50) && near(p.status().energy_j,-1));
        CHECK(p.status().received_ms==9 && p.status().frames==1);
        const uint8_t extremes[4]={0x00,0x80,0xff,0x7f};
        CHECK(p.receive(extremes,4,13)); CHECK(near(p.status().power_w,-3276.8f) && near(p.status().energy_j,3276.7f));
        CHECK(!p.receive(d,3,14)); CHECK(p.status().received_ms==13 && p.status().frames==2);
    }
    { // Default unconfirmed hardware and first-frame absence never yield motor effort.
        WheelFeedback w[2]{}; ChassisControl unconfirmed(kChassisConfig);
        feedback(w,0); unconfirmed.step(0,w,request(0,1));
        CHECK(!unconfirmed.status().enabled && unconfirmed.status().candidate_ma[0]==0);
        ChassisControl empty(confirmed()); WheelFeedback none[2]{};
        empty.step(0,none,request(0,1)); CHECK(!empty.status().enabled && !empty.status().wheels_stationary);
        auto bad=confirmed(); bad.motor_id[1]=bad.motor_id[0]; ChassisControl duplicate(bad);
        CHECK(!duplicate.status().configuration_valid && (duplicate.status().faults & ChassisConfigFault));
        bad=confirmed(); bad.gear_ratio=std::numeric_limits<float>::infinity(); ChassisControl ratio(bad);
        CHECK(!ratio.status().configuration_valid);
        auto p_only=confirmed(); p_only.velocity_ki=0; ChassisControl p(p_only); CHECK(p.status().configuration_valid);
        feedback(none,4); empty.step(4,none,request(4,1)); CHECK(!empty.status().enabled); // rejected enable is not pending.
        auto off=request(8); off.enabled=false; feedback(none,8); empty.step(8,none,off);
        feedback(none,12); empty.step(12,none,request(12,1)); CHECK(empty.status().enabled);
    }
    { // Velocity mapping has a fixed target, acceleration ramp, and independent turn/brake.
        auto c=confirmed(); ChassisControl a(c); WheelFeedback w[2]{};
        float last=0;
        for(uint32_t t=0;t<=1000;t+=4) {
            feedback(w,t); auto cmd=request(t,.5f,0); a.step(t,w,cmd);
            CHECK(a.status().enabled && a.status().reference_v_m_s>=last);
            CHECK(a.status().reference_v_m_s-last<=c.acceleration_m_s2*.004f+.00001f);
            last=a.status().reference_v_m_s;
            float final[2]={a.status().candidate_ma[0],a.status().candidate_ma[1]}; a.apply(final,true);
        }
        CHECK(near(last,.5f*c.max_speed_m_s,.00001f));
        feedback(w,1004); auto cmd=request(1004,1,1); cmd.brake=true;
        a.step(1004,w,cmd); CHECK(a.status().reference_v_m_s<last && a.status().reference_omega_rad_s>0);
        CHECK(a.status().target_rpm[0]<a.status().target_rpm[1]);
        last=a.status().reference_v_m_s;
        feedback(w,1008); cmd=request(1008,-1,0); a.step(1008,w,cmd);
        CHECK(a.status().reference_v_m_s>=0 && a.status().reference_v_m_s<last);
    }
    { // Common wheel-speed saturation preserves differential curvature.
        auto c=confirmed(); c.max_rotor_rpm=100; c.acceleration_m_s2=1000; c.angular_accel_rad_s2=1000;
        ChassisControl a(c); WheelFeedback w[2]{}; feedback(w,0);
        a.step(0,w,request(0,1,1));
        float unscaled[2]; ChassisControl::differential(c.max_speed_m_s,c.max_turn_rad_s,c.diameter_m,c.track_m,c.gear_ratio,unscaled);
        CHECK(near(a.status().target_rpm[1],100));
        CHECK(near(a.status().target_rpm[0]/a.status().target_rpm[1],unscaled[0]/unscaled[1],.00001f));
    }
    { // Mode reset starts the stopping ramp at measured motion, not a fictional zero.
        auto c=confirmed(); ChassisControl a(c); WheelFeedback w[2]{};
        feedback(w,0,1000,1000); auto cmd=request(0); cmd.stop=cmd.brake=cmd.reset=true;
        a.step(0,w,cmd); const float measured=a.status().v_m_s;
        CHECK(measured>0 && near(a.status().reference_v_m_s,measured-c.deceleration_m_s2*.004f,.00001f));
        feedback(w,4,1000,1000); cmd.updated_ms=4; cmd.reset=false; cmd.epoch=1;
        a.step(4,w,cmd); CHECK(near(a.status().reference_v_m_s,measured-c.deceleration_m_s2*.004f,.00001f));
        feedback(w,8,1000,1000); cmd.updated_ms=8; cmd.reset=true;
        a.step(8,w,cmd); const float reset_start=a.status().reference_v_m_s;
        for(uint32_t t=12;t<=28;t+=4) {
            feedback(w,t,1000,1000); cmd.updated_ms=t; // Held reset must not reinitialize every cycle.
            const float previous=a.status().reference_v_m_s; a.step(t,w,cmd);
            CHECK(a.status().reference_v_m_s<previous);
        }
        CHECK(a.status().reference_v_m_s<reset_start-.008f);
    }
    { // Active feedback loss latches, and receiving healthy frames alone cannot restart.
        ChassisControl a(confirmed()); WheelFeedback w[2]{}; feedback(w,0); a.step(0,w,request(0,.2f));
        CHECK(a.status().enabled);
        for(uint32_t t=4;t<=52;t+=4) { w[1].received_ms=t; a.step(t,w,request(t,.2f)); }
        CHECK((a.status().faults & ChassisFeedbackFault) && !a.status().enabled);
        feedback(w,56); a.step(56,w,request(56,.2f)); CHECK(!a.status().enabled);
        CHECK(a.status().applied_ma[0]==0 && a.status().candidate_ma[1]==0);
        CHECK(a.clear_faults()); CHECK(!a.status().enabled);
        feedback(w,60); a.step(60,w,request(60,.2f)); CHECK(!a.status().enabled);
        auto off=request(64); off.enabled=false; feedback(w,64); a.step(64,w,off);
        feedback(w,68); a.step(68,w,request(68,.2f)); CHECK(a.status().enabled);
    }
    { // Failed output submission clears both outputs and latches transport failure.
        ChassisControl a(confirmed()); WheelFeedback w[2]{}; feedback(w,0); a.step(0,w,request(0,1));
        float final[2]={a.status().candidate_ma[0],a.status().candidate_ma[1]}; a.apply(final,false);
        CHECK(!a.status().enabled && (a.status().faults & ChassisTransportFault));
        CHECK(a.status().applied_ma[0]==0 && a.status().applied_ma[1]==0);
        feedback(w,4); a.step(4,w,request(4,1)); CHECK(!a.status().enabled);
    }
    { // One observed disabled period is sufficient for a new explicit enable request.
        ChassisControl a(confirmed()); WheelFeedback w[2]{}; feedback(w,0); a.step(0,w,request(0));
        CHECK(a.status().enabled);
        auto off=request(4); off.enabled=false; feedback(w,4); a.step(4,w,off); CHECK(!a.status().enabled);
        feedback(w,8); a.step(8,w,request(8)); CHECK(a.status().enabled);
    }
    { // Final power/safety reduction freezes positive integration, not just the PI's own clamp.
        auto c=confirmed(); c.acceleration_m_s2=1000; c.velocity_ki=100;
        ChassisControl a(c); WheelFeedback w[2]{}; float initial=0;
        for(uint32_t t=0;t<=400;t+=4) {
            feedback(w,t); a.step(t,w,request(t,.25f));
            if(t==0) initial=a.status().candidate_ma[0];
            CHECK(near(a.status().candidate_ma[0],initial,.00001f));
            const float reduced[2]={0,0}; a.apply(reduced,true);
        }
        feedback(w,404); a.step(404,w,request(404,.25f));
        CHECK(near(a.status().candidate_ma[0],initial,.00001f));
        const float applied[2]={initial,initial}; a.apply(applied,true);
        feedback(w,408); a.step(408,w,request(408,.25f)); CHECK(a.status().candidate_ma[0]>initial);
    }
    { // The suspended speed helper uses the same PI, refuses invalid/disabled requests.
        ChassisControl a(confirmed()); a.step_debug_speed(0,300);
        CHECK(a.status().candidate_ma[0]==0);
        WheelFeedback w[2]{}; feedback(w,0); a.step(0,w,request(0)); a.step_debug_speed(1,-300);
        CHECK(a.status().target_rpm[0]==0 && a.status().target_rpm[1]==-300);
        CHECK(a.status().candidate_ma[0]==0 && a.status().candidate_ma[1]>=-300 && a.status().candidate_ma[1]<0);
        const float saved=a.status().candidate_ma[1];
        a.step_debug_speed(2,200); CHECK(a.status().candidate_ma[1]==saved);
        a.step_debug_speed(0,301); CHECK(a.status().candidate_ma[1]==saved);
        a.step_debug_speed(0,std::numeric_limits<float>::quiet_NaN()); CHECK(a.status().candidate_ma[1]==saved);
        feedback(w,4,200,0); a.step(4,w,request(4)); a.step_debug_speed(1,200);
        CHECK(a.status().candidate_ma[0]==0); // Non-selected wheel does not receive braking effort.
        auto low=confirmed(); low.max_current_ma=100; ChassisControl limited(low);
        feedback(w,0); limited.step(0,w,request(0)); limited.step_debug_speed(0,300);
        CHECK(std::fabs(limited.status().candidate_ma[0])<=100);
    }
    { // Wheel-stationary requires fresh measured feedback for a duration, not zero targets.
        ChassisControl a(confirmed()); WheelFeedback w[2]{}; ChassisCommand off{};
        for(uint32_t t=0;t<=200;t+=4) { feedback(w,t); a.step(t,w,off); if(t<200) CHECK(!a.status().wheels_stationary); }
        CHECK(a.status().wheels_stationary);
        feedback(w,204,30,0); a.step(204,w,off); CHECK(!a.status().wheels_stationary);
        feedback(w,208); a.step(208,w,off); CHECK(!a.status().wheels_stationary);
        w[0].frames=0; a.step(212,w,off); CHECK(!a.status().wheels_stationary);
    }
    { // Time arithmetic survives tick wrap; invalid command and missed period fail closed.
        auto c=confirmed(); ChassisControl a(c); WheelFeedback w[2]{};
        feedback(w,0xfffffffcu); a.step(0xfffffffcu,w,request(0xfffffffcu,.1f)); CHECK(a.status().enabled);
        feedback(w,0); a.step(0,w,request(0,.1f)); CHECK(a.status().enabled && a.status().faults==0);
        feedback(w,4); auto cmd=request(4); cmd.speed_request=std::numeric_limits<float>::quiet_NaN();
        a.step(4,w,cmd); CHECK(!a.status().enabled && (a.status().faults & ChassisCommandFault));
        ChassisControl late(c); feedback(w,0); late.step(0,w,request(0));
        feedback(w,8); late.step(8,w,request(8)); CHECK(!late.status().enabled && (late.status().faults & ChassisTimingFault));
        ChassisControl stale(c); feedback(w,0); stale.step(0,w,request(0));
        for(uint32_t t=4;t<=44;t+=4) { feedback(w,t); stale.step(t,w,request(0)); }
        CHECK(!stale.status().enabled && (stale.status().faults & ChassisCommandFault));
    }
    return 0;
}
