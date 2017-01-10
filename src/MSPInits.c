
#include "IOConfigs.h"
#include "MSPInits.h"

/**
  * @brief  Initialize SPI MSP.
  * @retval None
  */
void SPIy_MspInit(void)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};

  /*** Configure the GPIOs ***/
  /* Enable GPIO clock */
  TOUCH_SPIx_SCK_GPIO_CLK_ENABLE();
  TOUCH_SPIx_MISO_MOSI_GPIO_CLK_ENABLE();
  TOUCH_CS_GPIO_CLK_ENABLE();
  TOUCH_IRQ_GPIO_CLK_ENABLE();

  /* Configure SPI SCK */
  gpioinitstruct.Pin        = TOUCH_SPIx_SCK_PIN | TOUCH_SPIx_MOSI_PIN;
  gpioinitstruct.Mode       = GPIO_MODE_AF_PP;
  gpioinitstruct.Speed      = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TOUCH_SPIx_SCK_GPIO_PORT, &gpioinitstruct);

  gpioinitstruct.Pin        = TOUCH_SPIx_MISO_PIN;
  gpioinitstruct.Mode       = GPIO_MODE_AF_INPUT;
  HAL_GPIO_Init(TOUCH_SPIx_MISO_MOSI_GPIO_PORT, &gpioinitstruct);

  /*** Configure the SPI peripheral ***/
  /* Enable SPI clock */
  TOUCH_SPIx_CLK_ENABLE();
}


/**
  * @brief  Initialize SPI MSP.
  * @retval None
  */
void SPIx_MspInit(void)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};

  /*** Configure the GPIOs ***/
  /* Enable GPIO clock */
  LCD_SPIx_SCK_GPIO_CLK_ENABLE();
  LCD_SPIx_MISO_MOSI_GPIO_CLK_ENABLE();

  /* Configure SPI SCK */
  gpioinitstruct.Pin        = LCD_SPIx_SCK_PIN;
  gpioinitstruct.Mode       = GPIO_MODE_AF_PP;
  gpioinitstruct.Speed      = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_SPIx_SCK_GPIO_PORT, &gpioinitstruct);

  /* Configure SPI MISO and MOSI */
  gpioinitstruct.Pin        = LCD_SPIx_MOSI_PIN;
  HAL_GPIO_Init(LCD_SPIx_MISO_MOSI_GPIO_PORT, &gpioinitstruct);

  gpioinitstruct.Pin        = LCD_SPIx_MISO_PIN;
  gpioinitstruct.Mode       = GPIO_MODE_INPUT;
  HAL_GPIO_Init(LCD_SPIx_MISO_MOSI_GPIO_PORT, &gpioinitstruct);

  /*** Configure the SPI peripheral ***/
  /* Enable SPI clock */
  LCD_SPIx_CLK_ENABLE();
  /* Enable DMA clock */
  DMAx_CLK_ENABLE();

  /*##-3- Configure the NVIC for SPI #########################################*/
  /* NVIC for SPI */
  HAL_NVIC_SetPriority(SPIx_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(SPIx_IRQn);

}



