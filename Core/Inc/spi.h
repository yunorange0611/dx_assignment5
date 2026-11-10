#pragma once
#include "stm32f4xx_hal.h"
#if defined(HAL_SPI_MODULE_ENABLED)
#ifdef __cplusplus
extern "C" {
#endif
extern SPI_HandleTypeDef hspi1;
void MX_SPI1_Init(void);
// C-board BMI088: ACC CS PA4, GYRO CS PB0. Set both high before making outputs.
void MX_BMI088_CS_Init(void);
#ifdef __cplusplus
}
#endif
#endif
