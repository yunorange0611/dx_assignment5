#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Debugger writes requests, then changes request_seq last. Only the existing
// arm/control owner consumes them; status globals are read-only snapshots.
typedef struct {
    float half_range_deg;
    uint32_t request_seq;
} YawLimitRequest;

typedef struct {
    uint32_t consumed_seq, active_seq, result;
    float active_half_range_deg, min_deg, max_deg, max_speed_deg_s;
    uint8_t ready;
} YawLimitStatus;

extern volatile YawLimitRequest g_yaw_limit_request;
extern YawLimitStatus g_yaw_limit_status; // result = engineer::Result.
int Yaw_GetLimitStatus(YawLimitStatus *status);

typedef struct {
    uint32_t mode;
    float speed_deg_s;
    uint32_t duration_ms, suspended_confirm, request_seq;
} YawDebugRequest;

typedef struct {
    uint32_t consumed_seq, result, until_ms;
    float speed_deg_s;
    uint8_t active;
} YawDebugStatus;

extern volatile YawDebugRequest g_yaw_debug_request;
extern YawDebugStatus g_yaw_debug_status;

#ifdef __cplusplus
}

namespace engineer {
class ArmControl;
class ArmOperator;
struct Interlocks;
struct RemoteInput;
struct OperatorOutput;

// Task-layer adaptation, called only by Robot_Execute inside arm_task. These
// functions never create a task, motor instance, CAN publisher or control loop.
bool Yaw_EnvelopeAllowed(float min_deg, float max_deg, float speed_deg_s);
bool Yaw_ConfigurationReady(const ArmControl &arm);

// Updates the core and operator from the same accepted request. Publishing the
// new limits also atomically invalidates an incompatible shared interlock.
void Yaw_ApplyLimitRequest(ArmControl &arm, ArmOperator &controller, Interlocks &locks,
                           Interlocks &shared_locks, uint32_t now_ms);

// Applies mode-dependent range/speed limits. A rejected shrink parks the
// chassis and preserves the existing safe target and envelope.
void Yaw_ApplyOperatingEnvelope(ArmControl &arm, const Interlocks &locks, OperatorOutput &output);

// Dedicated suspended-bench flow. Protection stays in TaskProtect and motor
// output stays in ArmControl; Robot_Execute publishes the common robot status.
// The epoch reference is sampled atomically with the pulse request after the
// protection tick. Return that sample for the common Robot status publication.
uint32_t Yaw_ExecuteBench(ArmControl &arm, const RemoteInput &remote, uint32_t now_ms,
                          uint32_t elapsed_ms, bool stop_requested,
                          const uint32_t &owner_stop_epoch);
} // namespace engineer
#endif
