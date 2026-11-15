#pragma once
#include "Arm/arm_control.h"
void Robot_Execute(engineer::ArmControl &arm, uint32_t now_ms, bool force_stop);
engineer::Result Robot_ApplyArmTuning(engineer::ArmControl &arm, unsigned joint,
                                     const engineer::JointTuning &tuning, uint32_t now_ms);
void Robot_ReportArmState(const engineer::Status &state);
