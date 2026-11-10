#include "chassis_bench.h"
#include <cmath>

namespace engineer {
void ChassisBenchGate::cancel(ChassisBenchResult result) {
    state_.active=false;
    state_.value=0;
    state_.mode=uint32_t(ChassisBenchMode::Stop);
    state_.result=result;
}
const ChassisBenchState &ChassisBenchGate::step(uint32_t now,
        const ChassisBenchRequest &r,const ChassisBenchHealth &h) {
    state_.clear_requested=false;
    const bool stop_changed=initialized_ && h.stop_epoch!=previous_stop_epoch_;
    previous_stop_epoch_=h.stop_epoch; initialized_=true;
    const bool was_active=state_.active;
    const bool healthy=h.hardware_confirmed && h.mapping_confirmed && h.remote_ok &&
        h.owner_idle && h.feedback_fresh && h.temperature_ok && h.transport_ok &&
        h.timing_ok && !h.global_stop && !stop_changed;
    if(state_.active) {
        if(!healthy || h.controller_faulted || !h.controller_enabled)
            cancel(ChassisBenchResult::Cancelled);
        else if(int32_t(now-state_.until_ms)>=0)
            cancel(ChassisBenchResult::Expired);
    }
    if(r.request_seq==state_.consumed_seq) return state_;
    state_.consumed_seq=r.request_seq;

    // Every new command first withdraws the previous pulse. In particular, a
    // stream of new sequence numbers cannot extend a running pulse indefinitely.
    cancel(ChassisBenchResult::InvalidRequest);
    if(r.mode==uint32_t(ChassisBenchMode::Stop)) {
        state_.result=ChassisBenchResult::Stopped;
        return state_;
    }
    if(r.mode==uint32_t(ChassisBenchMode::Clear)) {
        if(was_active || h.controller_enabled) state_.result=ChassisBenchResult::Busy;
        else if(!healthy || !h.wheels_stationary) state_.result=ChassisBenchResult::NotReady;
        else {
            state_.clear_requested=true;
            state_.result=ChassisBenchResult::ClearRequested;
        }
        return state_;
    }
    const bool valid=(r.mode==uint32_t(ChassisBenchMode::Current) ||
                      r.mode==uint32_t(ChassisBenchMode::Speed)) &&
        r.wheel<2 && r.duration_ms>0 && r.duration_ms<=500 &&
        std::isfinite(r.value) && std::fabs(r.value)<=300 &&
        r.suspended_confirm==0x53555350u;
    if(!valid) return state_;
    if(was_active || h.controller_enabled) {
        state_.result=ChassisBenchResult::Busy;
        return state_;
    }
    if(!healthy || !h.wheels_stationary || h.controller_faulted) {
        state_.result=ChassisBenchResult::NotReady;
        return state_;
    }
    state_.active=true;
    state_.mode=r.mode; state_.wheel=r.wheel; state_.value=r.value;
    state_.until_ms=now+r.duration_ms;
    state_.result=ChassisBenchResult::Accepted;
    return state_;
}
}
