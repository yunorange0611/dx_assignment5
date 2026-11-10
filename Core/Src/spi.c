#include "spi.h"
#if defined(HAL_SPI_MODULE_ENABLED)
#include "main.h"
SPI_HandleTypeDef hspi1;

void MX_BMI088_CS_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
    GPIO_InitTypeDef pins={0};
    pins.Pin=GPIO_PIN_4; pins.Mode=GPIO_MODE_OUTPUT_PP;
    pins.Pull=GPIO_NOPULL; pins.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA,&pins);
    pins.Pin=GPIO_PIN_0; HAL_GPIO_Init(GPIOB,&pins);
}
void MX_SPI1_Init(void) {
    MX_BMI088_CS_Init();
    hspi1.Instance=SPI1;
    hspi1.Init.Mode=SPI_MODE_MASTER;
    hspi1.Init.Direction=SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize=SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity=SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase=SPI_PHASE_2EDGE;
    hspi1.Init.NSS=SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit=SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode=SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial=10;
    if(HAL_SPI_Init(&hspi1)!=HAL_OK) Error_Handler();
}
void HAL_SPI_MspInit(SPI_HandleTypeDef *spi) {
    if(spi->Instance!=SPI1) return;
    __HAL_RCC_SPI1_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    // Template pin routing: PB3=SCK, PB4=MISO, PA7=MOSI, AF5. No SPI DMA/IRQ.
    GPIO_InitTypeDef pins={0};
    pins.Pin=GPIO_PIN_3|GPIO_PIN_4; pins.Mode=GPIO_MODE_AF_PP;
    pins.Pull=GPIO_NOPULL; pins.Speed=GPIO_SPEED_FREQ_VERY_HIGH; pins.Alternate=GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOB,&pins);
    pins.Pin=GPIO_PIN_7; HAL_GPIO_Init(GPIOA,&pins);
}
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spi) {
    if(spi->Instance!=SPI1) return;
    __HAL_RCC_SPI1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB,GPIO_PIN_3|GPIO_PIN_4); HAL_GPIO_DeInit(GPIOA,GPIO_PIN_7);
}
#endif
