/**
  ******************************************************************************
  * @file    stm32f1xx_nucleo.c
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    29-April-2016
  * @brief   This file provides set of firmware functions to manage:
  *          - LEDs and push-button available on STM32F1XX-Nucleo Kit 
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD 
  *            shield (reference ID 802)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "IOConfigs.h"
#include "dsdisplay_config.h"
#include "main.h"
#include "ili9341.h"
#include "MSPInits.h"
/** @addtogroup BSP
  * @{
  */ 

/** @defgroup STM32F103C8T6_MINIMAL STM32F103C8T6_MINIMAL
  * @brief This file provides set of firmware functions to manage Leds and push-button
  *        available on STM32F103C8T6_MINIMAL Kit from STMicroelectronics.
  *        It provides also LCD, joystick and uSD functions to communicate with 
  *        Adafruit 1.8" TFT LCD shield (reference ID 802)
  * @{
  */ 


/** @defgroup STM32F1XX_NUCLEO_Private_Defines Private Defines
  * @{
  */ 
  
/**
* @brief STM32F103RB NUCLEO BSP Driver version
*/
#define __STM32F103C8T6_MINIMAL_BSP_VERSION_MAIN   (0x01) /*!< [31:24] main version */
#define __STM32F103C8T6_MINIMAL_BSP_VERSION_SUB1   (0x00) /*!< [23:16] sub1 version */
#define __STM32F103C8T6_MINIMAL_BSP_VERSION_SUB2   (0x03) /*!< [15:8]  sub2 version */
#define __STM32F103C8T6_MINIMAL_BSP_VERSION_RC     (0x00) /*!< [7:0]  release candidate */
#define __STM32F103C8T6_MINIMAL_BSP_VERSION       ((__STM32F103C8T6_MINIMAL_BSP_VERSION_MAIN << 24)\
                                             |(__STM32F103C8T6_MINIMAL_BSP_VERSION_SUB1 << 16)\
                                             |(__STM32F103C8T6_MINIMAL_BSP_VERSION_SUB2 << 8 )\
                                             |(__STM32F103C8T6_MINIMAL_BSP_VERSION_RC))

/**
  * @brief LINK SD Card
  */
#define SD_DUMMY_BYTE            0xFF    
#define SD_NO_RESPONSE_EXPECTED  0x80
   
/**
  * @}
  */ 


/** @defgroup STM32F1XX_NUCLEO_Private_Variables Private Variables
  * @{
  */ 
GPIO_TypeDef* LED_PORT[LEDn] = {TFT_BACKLED_GPIO_PORT};

const uint16_t LED_PIN[LEDn] = {TFT_BACKLED_PIN};

GPIO_TypeDef* BUTTON_PORT[BUTTONn]  = {USER_BUTTON_GPIO_PORT}; 
const uint16_t BUTTON_PIN[BUTTONn]  = {USER_BUTTON_PIN}; 
const uint8_t  BUTTON_IRQn[BUTTONn] = {USER_BUTTON_EXTI_IRQn };

/**
 * @brief BUS variables
 */

#ifdef HAL_SPI_MODULE_ENABLED
uint32_t SpixTimeout = LCD_SPIx_TIMEOUT_MAX;        /*<! Value of Timeout when SPI communication fails */
SPI_HandleTypeDef TFT_SPI_HANDLE;
SPI_HandleTypeDef TOUCH_SPI_HANDLE;
#endif /* HAL_SPI_MODULE_ENABLED */

/* Private variables ---------------------------------------------------------*/
static DMA_HandleTypeDef hdma_tx;
static DMA_HandleTypeDef hdma_rx;

/* transfer state */
__IO uint32_t wTransferState = TRANSFER_WAIT;


#ifdef HAL_ADC_MODULE_ENABLED
static ADC_HandleTypeDef hnucleo_Adc;
/* ADC channel configuration structure declaration */
static ADC_ChannelConfTypeDef sConfig;
#endif /* HAL_ADC_MODULE_ENABLED */

/**
  * @}
  */ 

/** @defgroup STM32F103C8T6_MINIMAL_Private_Functions Private Functions
  * @{
  */ 
#ifdef HAL_SPI_MODULE_ENABLED
//static void               SPIx_Init(void);
//static void               SPIy_Init(void);

//static void               SPIx_Write(uint8_t Value);
//static void               SPIx_WriteData(uint8_t *DataIn, uint16_t DataLength);
static void               SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth);
static void               SPIx_Error (void);
//static void               SPIx_MspInit(void);

/* SD IO functions */
void                      SD_IO_Init(void);
void                      SD_IO_CSState(uint8_t state);
void                      SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength);
void                      SD_IO_ReadData(uint8_t *DataOut, uint16_t DataLength);
void                      SD_IO_WriteData(const uint8_t *Data, uint16_t DataLength);
uint8_t                   SD_IO_WriteByte(uint8_t Data);
uint8_t                   SD_IO_ReadByte(void);

/* LCD IO functions */
//void                      LCD_IO_Init(void);
void                      LCD_IO_WriteData(uint8_t Data);
void                      LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size);
void                      LCD_IO_WriteReg(uint8_t LCDReg);
void                      LCD_Delay(uint32_t delay);
#endif /* HAL_SPI_MODULE_ENABLED */

#ifdef HAL_ADC_MODULE_ENABLED
static HAL_StatusTypeDef  ADCx_Init(void);
static void               ADCx_DeInit(void);
static void               ADCx_MspInit(ADC_HandleTypeDef *hadc);
static void               ADCx_MspDeInit(ADC_HandleTypeDef *hadc);
#endif /* HAL_ADC_MODULE_ENABLED */

/**
  * @brief  TxRx Transfer completed callback.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report end of Interrupt TxRx transfer, and
  *         you can add your own implementation.
  * @retval None
  */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  /* Turn LED on: Transfer in transmission/reception process is correct */
//  BSP_LED_On(LED2);
  wTransferState = TRANSFER_COMPLETE;
}

/**
  * @brief  SPI error callbacks.
  * @param  hspi: SPI handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = TRANSFER_ERROR;
}


/**
  * @}
  */ 

SPI_HandleTypeDef* GetTFTHandlePtr()
{
  return &TFT_SPI_HANDLE;
}

/** @defgroup STM32F103C8T6_MINIMAL_Exported_Functions Exported Functions
  * @{
  */ 

/**
  * @brief  This method returns the STM32F103C8T6_MINIMAL BSP Driver revision
  * @retval version : 0xXYZR (8bits for each decimal, R for RC)
  */
uint32_t BSP_GetVersion(void)
{
  return __STM32F103C8T6_MINIMAL_BSP_VERSION;
}

/** @defgroup STM32F1XX_NUCLEO_LED_Functions LED Functions
  * @{
  */ 

/**
  * @brief  Configures TFT BACKLITE LED GPIO.
  * @param  Led: Led to be configured. 
  *          This parameter can be one of the following values:
  *     @arg LED2
  * @retval None
  */
void BSP_TFT_BACKLED_Init(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpioinitstruct;
  
  /* Enable the GPIO_LED Clock */
  LEDx_GPIO_CLK_ENABLE(Led);

  /* Configure the GPIO_LED pin */
  gpioinitstruct.Pin    = LED_PIN[Led];
  gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull   = GPIO_NOPULL;
  gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
  
  HAL_GPIO_Init(LED_PORT[Led], &gpioinitstruct);

  /* Reset PIN to switch off the LED */
  HAL_GPIO_WritePin(LED_PORT[Led],LED_PIN[Led], GPIO_PIN_RESET);
}

/**
  * @brief  DeInit LEDs.
  * @param  Led: LED to be de-init. 
  *   This parameter can be one of the following values:
  *     @arg  LED2
  * @note Led DeInit does not disable the GPIO clock nor disable the Mfx 
  * @retval None
  */
void BSP_TFT_BACKLED_DeInit(Led_TypeDef Led)
{
  GPIO_InitTypeDef  gpio_init_structure;

  /* Turn off LED */
  HAL_GPIO_WritePin(LED_PORT[Led],LED_PIN[Led], GPIO_PIN_RESET);
  /* DeInit the GPIO_LED pin */
  gpio_init_structure.Pin = LED_PIN[Led];
  HAL_GPIO_DeInit(LED_PORT[Led], gpio_init_structure.Pin);
}

/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void BSP_TFT_BACKLED_On(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_SET); 
}

/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED2
  * @retval None
  */
void BSP_TFT_BACKLED_Off(Led_TypeDef Led)
{
  HAL_GPIO_WritePin(LED_PORT[Led], LED_PIN[Led], GPIO_PIN_RESET); 
}

/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *            @arg  LED2
  * @retval None
  */
void BSP_TFT_BACKLED_Toggle(Led_TypeDef Led)
{
  HAL_GPIO_TogglePin(LED_PORT[Led], LED_PIN[Led]);
}

/**
  * @}
  */ 

/** @defgroup STM32F103C8T6_MINIMAL_BUTTON_Functions BUTTON Functions
  * @{
  */ 

/**
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter should be: BUTTON_USER
  * @param  ButtonMode: Specifies Button mode.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO 
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                     generation capability  
  * @retval None
  */
void BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode)
{
  GPIO_InitTypeDef gpioinitstruct;

  /* Enable the BUTTON Clock */
  BUTTONx_GPIO_CLK_ENABLE(Button);

  gpioinitstruct.Pin = BUTTON_PIN[Button];
  gpioinitstruct.Pull = GPIO_NOPULL;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_MEDIUM;

  if (ButtonMode == BUTTON_MODE_GPIO)
  {
    /* Configure Button pin as input */
    gpioinitstruct.Mode   = GPIO_MODE_INPUT;
  
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpioinitstruct);
  }
 
  if (ButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configure Button pin as input with External interrupt */
    gpioinitstruct.Mode   = GPIO_MODE_IT_FALLING; 
    HAL_GPIO_Init(BUTTON_PORT[Button], &gpioinitstruct);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    HAL_NVIC_SetPriority((IRQn_Type)(BUTTON_IRQn[Button]), 0x0F, 0);
    HAL_NVIC_EnableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  }
}

/**
  * @brief  Push Button DeInit.
  * @param  Button: Button to be configured
  *   This parameter should be: BUTTON_USER  
  * @note PB DeInit does not disable the GPIO clock
  * @retval None
  */
void BSP_PB_DeInit(Button_TypeDef Button)
{
  GPIO_InitTypeDef gpio_init_structure;

  gpio_init_structure.Pin = BUTTON_PIN[Button];
  HAL_NVIC_DisableIRQ((IRQn_Type)(BUTTON_IRQn[Button]));
  HAL_GPIO_DeInit(BUTTON_PORT[Button], gpio_init_structure.Pin);
}

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter should be: BUTTON_USER
  * @retval Button state.
  */
uint32_t BSP_PB_GetState(Button_TypeDef Button)
  {
  return HAL_GPIO_ReadPin(BUTTON_PORT[Button], BUTTON_PIN[Button]);
  }
/**
  * @}
  */ 

/**
  * @}
  */

/** @addtogroup STM32F103C8T6_MINIMAL_Private_Functions
  * @{
  */ 
  
#ifdef HAL_SPI_MODULE_ENABLED
/******************************************************************************
                            BUS OPERATIONS
*******************************************************************************/

/**
  * @brief  Initialize TFT module SPI(1).
  * @retval None
  */
static void SPIx_Init()
{	SPI_HandleTypeDef* P_TFT_SPI_HANDLE;
  if(HAL_SPI_GetState(&TFT_SPI_HANDLE) == HAL_SPI_STATE_RESET)
  {
    /* SPI Config */
      TFT_SPI_HANDLE.Instance = LCD_SPIx;
      /* SPI baudrate is set to 8 MHz maximum (PCLK2/SPI_BaudRatePrescaler = 64/8 = 8 MHz)
       to verify these constraints:
          - ST7735 LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
            Since the provided driver doesn't use read capability from LCD, only constraint
            on write baudrate is considered.
          - SD card SPI interface max baudrate is 25MHz for write/read
          - PCLK2 max frequency is 32 MHz
       */
    TFT_SPI_HANDLE.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_2;
    TFT_SPI_HANDLE.Init.Direction          = SPI_DIRECTION_2LINES;
    TFT_SPI_HANDLE.Init.CLKPhase           = SPI_PHASE_1EDGE;
    TFT_SPI_HANDLE.Init.CLKPolarity        = SPI_POLARITY_LOW;
    TFT_SPI_HANDLE.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
    TFT_SPI_HANDLE.Init.CRCPolynomial      = 7;
    TFT_SPI_HANDLE.Init.DataSize           = SPI_DATASIZE_8BIT;
    TFT_SPI_HANDLE.Init.FirstBit           = SPI_FIRSTBIT_MSB;
    TFT_SPI_HANDLE.Init.NSS                = SPI_NSS_SOFT;
    TFT_SPI_HANDLE.Init.TIMode             = SPI_TIMODE_DISABLE;
    TFT_SPI_HANDLE.Init.Mode               = SPI_MODE_MASTER;

    SPIx_MspInit();
    HAL_SPI_Init(&TFT_SPI_HANDLE);


    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(&TFT_SPI_HANDLE, hdmatx, hdma_tx);

    /* Configure the DMA handler for Transmission process */
    hdma_rx.Instance                 = SPIx_RX_DMA_CHANNEL;

    hdma_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_rx.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    hdma_rx.Init.Mode                = DMA_NORMAL;
    hdma_rx.Init.Priority            = DMA_PRIORITY_HIGH;

    HAL_DMA_Init(&hdma_rx);

    /* Associate the initialized DMA handle to the the SPI handle */
    __HAL_LINKDMA(&TFT_SPI_HANDLE, hdmarx, hdma_rx);

    /*##-4- Configure the NVIC for DMA #########################################*/
    /* NVIC configuration for DMA transfer complete interrupt (SPI2_TX) */
    HAL_NVIC_SetPriority(SPIx_DMA_TX_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(SPIx_DMA_TX_IRQn);

    /* NVIC configuration for DMA transfer complete interrupt (SPI2_RX) */
    HAL_NVIC_SetPriority(SPIx_DMA_RX_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(SPIx_DMA_RX_IRQn);


  }
}

/**
  * @brief  Initialize SPIy .
  * @retval None
  */
static void SPIy_Init()
{	SPI_HandleTypeDef* P_TFT_SPI_HANDLE;
  if(HAL_SPI_GetState(&TFT_SPI_HANDLE) == HAL_SPI_STATE_RESET)
  {
    /* SPI Config */
      TFT_SPI_HANDLE.Instance = TOUCH_SPIx;
      /* SPI baudrate is set to 8 MHz maximum (PCLK2/SPI_BaudRatePrescaler = 64/8 = 8 MHz)
       to verify these constraints:
          - ST7735 LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
            Since the provided driver doesn't use read capability from LCD, only constraint
            on write baudrate is considered.
          - SD card SPI interface max baudrate is 25MHz for write/read
          - PCLK2 max frequency is 32 MHz
       */
    TFT_SPI_HANDLE.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_2;
    TFT_SPI_HANDLE.Init.Direction          = SPI_DIRECTION_2LINES;
    TFT_SPI_HANDLE.Init.CLKPhase           = SPI_PHASE_1EDGE;
    TFT_SPI_HANDLE.Init.CLKPolarity        = SPI_POLARITY_LOW;
    TFT_SPI_HANDLE.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
    TFT_SPI_HANDLE.Init.CRCPolynomial      = 7;
    TFT_SPI_HANDLE.Init.DataSize           = SPI_DATASIZE_8BIT;
    TFT_SPI_HANDLE.Init.FirstBit           = SPI_FIRSTBIT_MSB;
    TFT_SPI_HANDLE.Init.NSS                = SPI_NSS_SOFT;
    TFT_SPI_HANDLE.Init.TIMode             = SPI_TIMODE_DISABLE;
    TFT_SPI_HANDLE.Init.Mode               = SPI_MODE_MASTER;

    SPIy_MspInit();
    HAL_SPI_Init(&TFT_SPI_HANDLE);


  }
}

HAL_StatusTypeDef SPI_Transmit_DMA_HW(SPI_HandleTypeDef *hspi, uint8_t* pData, uint32_t Size)
{ HAL_StatusTypeDef status; uint32_t i; uint32_t remain = Size;

  uint16_t int_part = ((remain & 0xFFFF0000) >> 16);
  for (i = 0; i < int_part; i++)
    {
      wTransferState = TRANSFER_WAIT;
      status = HAL_SPI_Transmit_DMA(&TFT_SPI_HANDLE, pData, 0xFFFF);
      while (wTransferState != TRANSFER_COMPLETE){ };
    }
  wTransferState = TRANSFER_WAIT;
  status = HAL_SPI_Transmit_DMA(&TFT_SPI_HANDLE, pData, Size);
  while (wTransferState != TRANSFER_COMPLETE){ };
  return status;
}

void Set16BitsSPIwithDMATX(uint32_t DMA_MINC_MODE, SPI_HandleTypeDef* HT)
{
  HAL_SPI_DeInit(HT);
  HT->Init.DataSize = SPI_DATASIZE_16BIT;	// Datasize to 16bit
  HAL_SPI_Init(HT);

  /*##-3- Configure the DMA ##################################################*/
  /* Configure the DMA handler for Transmission process */
  hdma_tx.Instance                 = SPIx_TX_DMA_CHANNEL;
  hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
  hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
  hdma_tx.Init.MemInc              = DMA_MINC_MODE;
  hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
  hdma_tx.Init.Mode                = DMA_NORMAL;
  hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;

  HAL_DMA_Init(&hdma_tx);
  /* Associate the initialized DMA handle to the the SPI handle */
  __HAL_LINKDMA(&TFT_SPI_HANDLE, hdmarx, hdma_rx);
}

void Set8BitsSPIwoutDMATX(SPI_HandleTypeDef* HT)
{
  HAL_DMA_DeInit(&hdma_tx);
  HAL_SPI_DeInit(HT);
  HT->Init.DataSize = SPI_DATASIZE_8BIT;
  HAL_SPI_Init(HT);
}

/**
  * @brief  FillRectangle fill the TFT rectangle area.
  * @retval None
  */

void FillRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
  uint32_t dmapixel = width * height;
  SPI_HandleTypeDef* HT;
  HAL_StatusTypeDef status = HAL_OK;

  SetCursorPosition(x, y, x + width - 1, y + height - 1);
  LCD_IO_WriteReg(ILI9341_GRAM);
  HT = GetTFTHandlePtr();
  Set16BitsSPIwithDMATX(DMA_MINC_DISABLE, HT);

  LCD_DC_HIGH();

  status = SPI_Transmit_DMA_HW(&TFT_SPI_HANDLE, (uint8_t*)&color, dmapixel);

  if (status != HAL_OK)
    {

    }

  LCD_DC_LOW();
  Set8BitsSPIwoutDMATX(HT);
}

void DrawGIMPImage(uint16_t x, uint16_t y, const struct s_image image)
{
  SPI_HandleTypeDef* HT;
  HAL_StatusTypeDef status = HAL_OK;

  SetCursorPosition(x, y, x + image.width - 1, y + image.height - 1);
  LCD_IO_WriteReg(ILI9341_GRAM);
  HT = GetTFTHandlePtr();
  Set16BitsSPIwithDMATX(DMA_MINC_ENABLE, HT);
  LCD_DC_HIGH();

  status = SPI_Transmit_DMA_HW(&TFT_SPI_HANDLE, (uint8_t*)&image.pixel_data, image.height * image.width);

  if (status != HAL_OK)
    {

    }

  LCD_DC_LOW();
  Set8BitsSPIwoutDMATX(HT);
}

/**
  * @brief  Initialize TFT module SPI.
  * @retval None
  */
static void TOUCH_SPIx_Init(void)
{
  if(HAL_SPI_GetState(&TOUCH_SPI_HANDLE) == HAL_SPI_STATE_RESET)
  {
    /* SPI Config */
      TOUCH_SPI_HANDLE.Instance = TOUCH_SPIx;
      /* SPI baudrate is set to 8 MHz maximum (PCLK2/SPI_BaudRatePrescaler = 64/8 = 8 MHz)
       to verify these constraints:
          - ST7735 LCD SPI interface max baudrate is 15MHz for write and 6.66MHz for read
            Since the provided driver doesn't use read capability from LCD, only constraint
            on write baudrate is considered.
          - SD card SPI interface max baudrate is 25MHz for write/read
          - PCLK2 max frequency is 32 MHz
       */
    TOUCH_SPI_HANDLE.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_8;
    TOUCH_SPI_HANDLE.Init.Direction          = SPI_DIRECTION_2LINES;
    TOUCH_SPI_HANDLE.Init.CLKPhase           = SPI_PHASE_1EDGE;
    TOUCH_SPI_HANDLE.Init.CLKPolarity        = SPI_POLARITY_LOW;
    TOUCH_SPI_HANDLE.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
    TOUCH_SPI_HANDLE.Init.CRCPolynomial      = 7;
    TOUCH_SPI_HANDLE.Init.DataSize           = SPI_DATASIZE_8BIT;
    TOUCH_SPI_HANDLE.Init.FirstBit           = SPI_FIRSTBIT_MSB;
    TOUCH_SPI_HANDLE.Init.NSS                = SPI_NSS_HARD_OUTPUT;
    TOUCH_SPI_HANDLE.Init.TIMode             = SPI_TIMODE_DISABLE;
    TOUCH_SPI_HANDLE.Init.Mode               = SPI_MODE_MASTER;

    SPIy_MspInit();
    HAL_SPI_Init(&TOUCH_SPI_HANDLE);
  }
}

/**
  * @brief  SPI Write a byte to device
  * @param  Value: value to be written
  * @retval None
*/
static void SPIx_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_SPI_TransmitReceive(&TFT_SPI_HANDLE, (uint8_t*) DataIn, DataOut, DataLength, SpixTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief  SPI Write an amount of data to device
  * @param  Value: value to be written
  * @param  DataLength: number of bytes to write
  * @retval None
  */
void SPIx_WriteData(uint8_t *DataIn, uint16_t DataLength)
{
  HAL_StatusTypeDef status = HAL_OK;

//  status = HAL_SPI_Transmit_DMA(&TFT_SPI_HANDLE, DataIn, DataLength);

  status = HAL_SPI_Transmit_IT(&TFT_SPI_HANDLE, DataIn, DataLength);

//  status = HAL_SPI_Transmit(&TFT_SPI_HANDLE, DataIn, DataLength, SpixTimeout);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief  SPI Write a byte to device
  * @param  Value: value to be written
  * @retval None
  */
void SPIx_Write(uint8_t Value)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t data;

  status = HAL_SPI_TransmitReceive(&TFT_SPI_HANDLE, (uint8_t*) &Value, &data, 1, SpixTimeout);

  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    SPIx_Error();
  }
}

/**
  * @brief  SPI error treatment function
  * @retval None
  */
static void SPIx_Error (void)
{
  /* De-initialize the SPI communication BUS */
  HAL_SPI_DeInit(&TFT_SPI_HANDLE);

  /* Re-Initiaize the SPI communication BUS */
  SPIx_Init();
}

/******************************************************************************
                            LINK OPERATIONS
*******************************************************************************/

/********************************* LINK SD ************************************/
/**
  * @brief  Initialize the SD Card and put it into StandBy State (Ready for 
  *         data transfer).
  * @retval None
  */
void SD_IO_Init(void)
{
  GPIO_InitTypeDef  gpioinitstruct = {0};
  uint8_t counter = 0;

  /* SD_CS_GPIO Periph clock enable */
  SD_CS_GPIO_CLK_ENABLE();

  /* Configure SD_CS_PIN pin: SD Card CS pin */
  gpioinitstruct.Pin    = SD_CS_PIN;
  gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull   = GPIO_PULLUP;
  gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_PORT, &gpioinitstruct);

  /* Configure LCD_CS_PIN pin: LCD Card CS pin */
  gpioinitstruct.Pin   = LCD_CS_PIN;
  gpioinitstruct.Mode  = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Pull  = GPIO_NOPULL;
  gpioinitstruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_PORT, &gpioinitstruct);
  LCD_CS_HIGH();
  /*------------Put SD in SPI mode--------------*/
  /* SD SPI Config */
  SPIx_Init();

  /* SD chip select high */
  SD_CS_HIGH();
  
  /* Send dummy byte 0xFF, 10 times with CS high */
  /* Rise CS and MOSI for 80 clocks cycles */
  for (counter = 0; counter <= 9; counter++)
  {
    /* Send dummy byte 0xFF */
    SD_IO_WriteByte(SD_DUMMY_BYTE);
  }
}

/**
  * @brief  Set the SD_CS pin.
  * @param  pin value.
  * @retval None
  */
void SD_IO_CSState(uint8_t val)
{
  if(val == 1) 
  {
    SD_CS_HIGH();
}
  else
  {
    SD_CS_LOW();
  }
}
 
/**
  * @brief  Write byte(s) on the SD
  * @param  DataIn: Pointer to data buffer to write
  * @param  DataOut: Pointer to data buffer for read data
  * @param  DataLength: number of bytes to write
  * @retval None
  */
void SD_IO_WriteReadData(const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLength)
  {
  /* Send the byte */
  SPIx_WriteReadData(DataIn, DataOut, DataLength);
}

/**
  * @brief  Write a byte on the SD.
  * @param  Data: byte to send.
  * @retval Data written
  */
uint8_t SD_IO_WriteByte(uint8_t Data)
{
  uint8_t tmp;

  /* Send the byte */
  SPIx_WriteReadData(&Data,&tmp,1);
  return tmp;
}

/**
  * @brief  Write an amount of data on the SD.
  * @param  Data: byte to send.
  * @param  DataLength: number of bytes to write
  * @retval none
  */
void SD_IO_ReadData(uint8_t *DataOut, uint16_t DataLength)
{
  /* Send the byte */
  SD_IO_WriteReadData(DataOut, DataOut, DataLength);
  }   
 
/**
  * @brief  Write an amount of data on the SD.
  * @param  Data: byte to send.
  * @param  DataLength: number of bytes to write
  * @retval none
  */
void SD_IO_WriteData(const uint8_t *Data, uint16_t DataLength)
{
  /* Send the byte */
  SPIx_WriteData((uint8_t *)Data, DataLength);
}

/********************************* LINK LCD ***********************************/
/**
  * @brief  Initialize the LCD
  * @retval None
  */
void LCD_IO_Init(void)
{
  GPIO_InitTypeDef  gpioinitstruct;

  /* LCD_CS_GPIO and LCD_DC_GPIO LCD_RST_GPIO Periph clock enable */
  LCD_CS_GPIO_CLK_ENABLE();
  LCD_DC_GPIO_CLK_ENABLE();
  LCD_RST_GPIO_CLK_ENABLE();
  
/*  Configure LCD DC, and Reset pins. */
  gpioinitstruct.Pin    = LCD_RST_PIN | LCD_DC_PIN | LCD_CS_PIN;
  gpioinitstruct.Mode   = GPIO_MODE_OUTPUT_PP;
  gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_RST_GPIO_PORT, &gpioinitstruct);
      
  /* LCD chip select high */
  LCD_CS_LOW();
  
  /* LCD SPI Config */
  SPIx_Init();
}

void TOUCH_IO_Init(void)
{
  GPIO_InitTypeDef  gpioinitstruct;

  TOUCH_IRQ_GPIO_CLK_ENABLE();

  gpioinitstruct.Pin    = TOUCH_IRQ_PIN;
  gpioinitstruct.Mode   = GPIO_MODE_INPUT;
  gpioinitstruct.Speed  = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_DC_GPIO_PORT, &gpioinitstruct);

  /* TOUCH SPI Config */
  TOUCH_SPIx_Init();

}

/**
  * @brief  Write command to select the LCD register.
  * @param  LCDReg: Address of the selected register.
  * @retval None
  */
void LCD_IO_WriteReg(uint8_t LCDReg)
{
  /* Reset LCD control line CS */
  LCD_CS_LOW();
  
  /* Set LCD data/command line DC to Low */
  LCD_DC_LOW();
    
  /* Send Command */
  SPIx_Write(LCDReg);

  /* Deselect : Chip Select high */
  //LCD_CS_HIGH();
}

/**
* @brief  Write register value.
* @param  pData Pointer on the register value
* @param  Size Size of byte to transmit to the register
* @retval None
*/
void LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size)
{
  uint32_t counter = 0;
  
  /* Reset LCD control line CS */
  LCD_CS_LOW();
  
  /* Set LCD data/command line DC to High */
  LCD_DC_HIGH();

  if (Size == 1)
  {
    /* Only 1 byte to be sent to LCD - general interface can be used */
    /* Send Data */
    SPIx_Write(*pData);
  }
  else
{
      /* Several data should be sent in a raw */
      /* Direct SPI accesses for optimization */
  for (counter = Size; counter != 0; counter--)
  {
      while(((TFT_SPI_HANDLE.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE) 	{}
      /* Need to invert bytes for LCD*/
      *((__IO uint8_t*)&TFT_SPI_HANDLE.Instance->DR) = *(pData);

/*      while(((TFT_SPI_HANDLE.Instance->SR) & SPI_FLAG_TXE) != SPI_FLAG_TXE)	{}
      *((__IO uint8_t*)&TFT_SPI_HANDLE.Instance->DR) = *pData;*/
      pData++;
  }  
  
    /* Wait until the bus is ready before releasing Chip select */ 
    while(((TFT_SPI_HANDLE.Instance->SR) & SPI_FLAG_BSY) != RESET)
  {
  } 
  } 
  
  /* Deselect : Chip Select high */
//  LCD_CS_HIGH();
}

/**
  * @brief  Wait for loop in ms.
  * @param  Delay in ms.
  * @retval None
  */
void LCD_Delay(uint32_t Delay)
{
  HAL_Delay(Delay);
}

#endif /* HAL_SPI_MODULE_ENABLED */

void DisplayOn()
{
  LCD_IO_WriteReg(ILI9341_DISPLAY_ON);
}

void DisplayOff()
{
  LCD_IO_WriteReg(ILI9341_DISPLAY_OFF);
}

const uint8_t datas_pwa[] = {0x39, 0x2C, 0x00, 0x34, 0x02};
const uint8_t datas_pwb[] = {0x00, 0xC1, 0x30};
const uint8_t datas_dtca[] = {0x85, 0x00, 0x78};
const uint8_t datas_dtcb[] = {0x00, 0x00};
const uint8_t datas_pw_seq[] = {0x64, 0x03, 0x12, 0x81};
const uint8_t datas_prc[] = {0x20};
const uint8_t datas_pw1[] = {0x23};
const uint8_t datas_pw2[] = {0x10};
const uint8_t datas_vcom1[] = {0x3E, 0x28};
const uint8_t datas_vcom2[] = {0x86};
const uint8_t datas_mac[] = {0x48};
const uint8_t datas_pform[] = {0x55};
const uint8_t datas_frc[] = {0x00, 0x18};
const uint8_t datas_dfc[] = {0x08, 0x82, 0x27};
const uint8_t datas_gen[] = {0x00};
const uint8_t datas_caddr[] = {0x00, 0x00, 0x00, 0xEF};
const uint8_t datas_paddr[] = {0x00, 0x00, 0x01, 0x3F};
const uint8_t datas_gamma[] = {0x01};
const uint8_t datas_pgamma[] = {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00};
const uint8_t datas_ngamma[] = {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F};

void InitLCD()
{
  /* Force reset */
  LCD_RST_LOW();
  HAL_Delay(200);
  LCD_RST_HIGH();
  /* Delay for RST response */
  HAL_Delay(200);

  /* Software reset */
  LCD_IO_WriteReg(ILI9341_RESET);
  HAL_Delay(200);
  LCD_IO_WriteReg(ILI9341_POWERA);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_pwa, sizeof(datas_pwa));
  LCD_IO_WriteReg(ILI9341_POWERB);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_pwb, sizeof(datas_pwb));
  LCD_IO_WriteReg(ILI9341_DTCA);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_dtca, sizeof(datas_dtca));
  LCD_IO_WriteReg(ILI9341_DTCB);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_dtcb, sizeof(datas_dtcb));
  LCD_IO_WriteReg(ILI9341_POWER_SEQ);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_pw_seq, sizeof(datas_pw_seq));
  LCD_IO_WriteReg(ILI9341_PRC);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_prc, sizeof(datas_prc));
  LCD_IO_WriteReg(ILI9341_POWER1);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_pw1, sizeof(datas_pw1));
  LCD_IO_WriteReg(ILI9341_POWER2);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_pw2, sizeof(datas_pw2));
  LCD_IO_WriteReg(ILI9341_VCOM1);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_vcom1, sizeof(datas_vcom1));
  LCD_IO_WriteReg(ILI9341_VCOM2);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_vcom2, sizeof(datas_vcom2));
  LCD_IO_WriteReg(ILI9341_MAC);

#ifdef RGB_BGR_COLOR
  #ifdef ROW_COL_EXCH
    SendData(0x68);
  #else
    LCD_IO_WriteMultipleData((uint8_t*)&datas_mac, sizeof(datas_mac));
  #endif
#else
  #ifdef ROW_COL_EXCH
  #else
    SendData(0x60);
#endif
  SendData(0x40);
#endif
  LCD_IO_WriteReg(ILI9341_PIXEL_FORMAT);
#ifdef PIXEL_FORMAT_18_BIT
  SendData(0x66);	// 18 bits pixel format
#else
  LCD_IO_WriteMultipleData((uint8_t*)&datas_pform, sizeof(datas_pform));
#endif
  LCD_IO_WriteReg(ILI9341_FRC);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_frc, sizeof(datas_frc));
  LCD_IO_WriteReg(ILI9341_DFC);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_dfc, sizeof(datas_dfc));
  LCD_IO_WriteReg(ILI9341_3GAMMA_EN);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_gen, sizeof(datas_gen));
  LCD_IO_WriteReg(ILI9341_COLUMN_ADDR);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_caddr, sizeof(datas_caddr));
  LCD_IO_WriteReg(ILI9341_PAGE_ADDR);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_paddr, sizeof(datas_paddr));
  LCD_IO_WriteReg(ILI9341_GAMMA);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_gamma, sizeof(datas_gamma));
  LCD_IO_WriteReg(ILI9341_PGAMMA);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_pgamma, sizeof(datas_pgamma));
  LCD_IO_WriteReg(ILI9341_NGAMMA);
  LCD_IO_WriteMultipleData((uint8_t*)&datas_ngamma, sizeof(datas_ngamma));
  LCD_IO_WriteReg(ILI9341_SLEEP_OUT);

  HAL_Delay(200);

  LCD_IO_WriteReg(ILI9341_DISPLAY_ON);
  LCD_IO_WriteReg(ILI9341_GRAM);
}

void SetCursorPosition(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{uint8_t datas[] = {0, 0, 0, 0};
  datas[0] = (x1 >> 8);
  datas[1] = (x1 & 0xFF);
  datas[2] = (x2 >> 8);
  datas[3] = (x2 & 0xFF);
  LCD_IO_WriteReg(ILI9341_COLUMN_ADDR);
  LCD_IO_WriteMultipleData((uint8_t*)&datas, sizeof(datas));
  datas[0] = (y1 >> 8);
  datas[1] = (y1 & 0xFF);
  datas[2] = (y2 >> 8);
  datas[3] = (y2 & 0xFF);
  LCD_IO_WriteReg(ILI9341_PAGE_ADDR);
  LCD_IO_WriteMultipleData((uint8_t*)&datas, sizeof(datas));
}


#ifdef HAL_ADC_MODULE_ENABLED
/******************************* LINK JOYSTICK ********************************/
/**
  * @brief  Initialize ADC MSP.
  * @retval None
  */
static void ADCx_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef  gpioinitstruct;
  
  /*** Configure the GPIOs ***/  
  /* Enable GPIO clock */
//  NUCLEO_ADCx_GPIO_CLK_ENABLE();
  
  /* Configure ADC1 Channel8 as analog input */
/*  gpioinitstruct.Pin    = NUCLEO_ADCx_GPIO_PIN ;
  gpioinitstruct.Mode   = GPIO_MODE_ANALOG;
  gpioinitstruct.Pull   = GPIO_NOPULL;
  gpioinitstruct.Speed  = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(NUCLEO_ADCx_GPIO_PORT, &gpioinitstruct);*/

  /*** Configure the ADC peripheral ***/ 
  /* Enable ADC clock */
  //NUCLEO_ADCx_CLK_ENABLE();
}

/**
  * @brief  DeInitializes ADC MSP.
  * @param  None
  * @note ADC DeInit does not disable the GPIO clock
  * @retval None
  */
static void ADCx_MspDeInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef  gpioinitstruct;

  /*** DeInit the ADC peripheral ***/ 
  /* Disable ADC clock */
  NUCLEO_ADCx_CLK_DISABLE(); 

  /* Configure the selected ADC Channel as analog input */
  //gpioinitstruct.Pin = NUCLEO_ADCx_GPIO_PIN ;
  //HAL_GPIO_DeInit(NUCLEO_ADCx_GPIO_PORT, gpioinitstruct.Pin);

  /* Disable GPIO clock has to be done by the application*/
  /* NUCLEO_ADCx_GPIO_CLK_DISABLE(); */
}

/**
  * @brief  Initializes ADC HAL.
  * @retval None
  */
static HAL_StatusTypeDef ADCx_Init(void)
{
  /* Set ADC instance */
  hnucleo_Adc.Instance = NUCLEO_ADCx;

  if(HAL_ADC_GetState(&hnucleo_Adc) == HAL_ADC_STATE_RESET)
  {
    /* ADC Config */
    hnucleo_Adc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hnucleo_Adc.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hnucleo_Adc.Init.ContinuousConvMode = DISABLE;
    hnucleo_Adc.Init.NbrOfConversion = 1;
    hnucleo_Adc.Init.DiscontinuousConvMode = DISABLE;
    hnucleo_Adc.Init.NbrOfDiscConversion = 1;
    hnucleo_Adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    
    /* Initialize MSP related to ADC */
    ADCx_MspInit(&hnucleo_Adc);
    
    /* Initialize ADC */
    if (HAL_ADC_Init(&hnucleo_Adc) != HAL_OK)
    {
      return HAL_ERROR;
    }
    
    /* Run ADC calibration */
    if (HAL_ADCEx_Calibration_Start(&hnucleo_Adc) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  return HAL_OK;
}  

/**
  * @brief  Initializes ADC HAL.
  * @param  None
  * @retval None
  */
static void ADCx_DeInit(void)
{
    hnucleo_Adc.Instance   = NUCLEO_ADCx;
    
    HAL_ADC_DeInit(&hnucleo_Adc);
    ADCx_MspDeInit(&hnucleo_Adc);
}

/******************************* LINK JOYSTICK ********************************/

/**
  * @brief  Configures joystick available on adafruit 1.8" TFT shield 
  *         managed through ADC to detect motion.
  * @retval Joystickstatus (0=> success, 1=> fail) 
  */
uint8_t BSP_JOY_Init(void)
{
  if (ADCx_Init() != HAL_OK)
  {
    return (uint8_t) HAL_ERROR; 
  }
  
  /* Select Channel 8 to be converted */
  sConfig.Channel       = ADC_CHANNEL_8;
  sConfig.SamplingTime  = ADC_SAMPLETIME_71CYCLES_5;
  sConfig.Rank          = 1;

  /* Return Joystick initialization status */
  return (uint8_t)HAL_ADC_ConfigChannel(&hnucleo_Adc, &sConfig);
}

/**
  * @brief  DeInit joystick GPIOs.
  * @note   JOY DeInit does not disable the Mfx, just set the Mfx pins in Off mode
  * @retval None.
  */
void BSP_JOY_DeInit(void)
{
    ADCx_DeInit();
}

/**
  * @brief  Returns the Joystick key pressed.
  * @note   To know which Joystick key is pressed we need to detect the voltage
  *         level on each key output
  *           - None  : 3.3 V / 4095
  *           - SEL   : 1.055 V / 1308
  *           - DOWN  : 0.71 V / 88
  *           - LEFT  : 3.0 V / 3720 
  *           - RIGHT : 0.595 V / 737
  *           - UP    : 1.65 V / 2046
  * @retval JOYState_TypeDef: Code of the Joystick key pressed.
  */
JOYState_TypeDef BSP_JOY_GetState(void)
{
  JOYState_TypeDef state = JOY_NONE;
  uint16_t  keyconvertedvalue = 0; 

 /* Start the conversion process */
  HAL_ADC_Start(&hnucleo_Adc);
  
  /* Wait for the end of conversion */
  if (HAL_ADC_PollForConversion(&hnucleo_Adc, 10) != HAL_TIMEOUT)
  {
    /* Get the converted value of regular channel */
    keyconvertedvalue = HAL_ADC_GetValue(&hnucleo_Adc);
  }
  
  if((keyconvertedvalue > 1800) && (keyconvertedvalue < 2090))
  {
    state = JOY_UP;
  }
  else if((keyconvertedvalue > 500) && (keyconvertedvalue < 780))
  {
    state = JOY_RIGHT;
  }
  else if((keyconvertedvalue > 1200) && (keyconvertedvalue < 1350))
  {
    state = JOY_SEL;
  }
  else if((keyconvertedvalue > 10) && (keyconvertedvalue < 130))
  {
    state = JOY_DOWN;
  }
  else if((keyconvertedvalue > 3500) && (keyconvertedvalue < 3760))
  {
    state = JOY_LEFT;
  }
  else
  {
    state = JOY_NONE;
  }
  
  /* Return the code of the Joystick key pressed*/
  return state;
}
#endif /* HAL_ADC_MODULE_ENABLED */

/**
  * @}
  */
  
/**
  * @}
  */    

/**
  * @}
  */ 
    
/**
  * @}
  */ 
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
