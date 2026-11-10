#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
typedef struct { uint32_t updated_ms,reasons; uint8_t ready,healthy; } VehicleHealth;
extern VehicleHealth g_vehicle_health;
int Vehicle_GetHealth(VehicleHealth *out);
void vehicle_state_task(void *argument);
#ifdef __cplusplus
}
#endif
