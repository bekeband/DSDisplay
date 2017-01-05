/**
  ******************************************************************************
  * @file    stm32f1xx_nucleo.h
  * @author  MCD Application Team
  * @version V1.0.3
  * @date    29-April-2016
  * @brief   This file contains definitions for:
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DSDISPLAY_CONFIG_H
#define __DSDISPLAY_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup STM32F103C8T6_MINIMAL
  * @{
  */ 

/* Includes ------------------------------------------------------------------*/
#include "IOConfigs.h"
#include "stm32f1xx_hal.h"
#include "stdint.h"
   
/** @defgroup STM32F1XX_NUCLEO_Exported_Types Exported Types
  * @{
  */
typedef enum 
{
  LED2 = 0,
  LED_GREEN = LED2
} Led_TypeDef;

typedef enum 
{  
  BUTTON_USER = 0,
  /* Alias */
  BUTTON_KEY  = BUTTON_USER
} Button_TypeDef;

typedef enum 
{  
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
} ButtonMode_TypeDef; 

typedef enum 
{ 
  JOY_NONE = 0,
  JOY_SEL = 1,
  JOY_DOWN = 2,
  JOY_LEFT = 3,
  JOY_RIGHT = 4,
  JOY_UP = 5
} JOYState_TypeDef;

enum {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};

struct s_image {
  uint16_t  	 width;
  uint8_t  	 height;
  uint8_t  	 bytes_per_pixel; /* 2:RGB16, 3:RGB, 4:RGBA */
  uint8_t 	pixel_data[52 * 49 * 2 + 1];
};

enum {
  NONE,
  DMA_FILLRECT
};

/* transfer state */
//__IO uint32_t wTransferState = TRANSFER_WAIT;

/**
  * @}
  */ 

/** @defgroup STM32F1XX_NUCLEO_Exported_Constants Exported Constants
  * @{
  */ 

/** 
  * @brief  Define for STM32F1xx_NUCLEO board  
  */ 
#if !defined (USE_STM32F103_MINIMAL_NUCLEO)
 #define USE_STM32F103_MINIMAL_NUCLEO
#endif
  

/**
  * @}
  */ 

void LCD_IO_Init(void);

void SPIx_Write(uint8_t Value);
void SPIx_WriteData(uint8_t *DataIn, uint16_t DataLength);

void DisplayOn();
void DisplayOff();
void SetCursorPosition(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
void LCD_IO_WriteMultipleData(uint8_t *pData, uint32_t Size);
void LCD_IO_WriteReg(uint8_t LCDReg);
void InitLCD();
void FillRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
void DrawGIMPImage(uint16_t x, uint16_t y, const struct s_image image);
void SD_IO_ReadData(uint8_t *DataOut, uint16_t DataLength);
uint8_t SD_IO_WriteByte(uint8_t Data);

SPI_HandleTypeDef* GetTFTHandlePtr();

/**
  * @}
  */

void TOUCH_IO_Init(void);

/**
  * @}
  */
    

/**
  * @brief  SD Control Lines management
  */  
#define SD_CS_LOW()       HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()      HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  TOUCH IC Control Lines management
  */
#define TOUCH_CS_LOW()       HAL_GPIO_WritePin(TOUCH_CS_GPIO_PORT, TOUCH_CS_PIN, GPIO_PIN_RESET)
#define TOUCH_CS_HIGH()      HAL_GPIO_WritePin(TOUCH_CS_GPIO_PORT, TOUCH_CS_PIN, GPIO_PIN_SET)

/**
  * @brief  LCD Control Lines management
  */
#define LCD_CS_LOW()      HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_RESET)
#define LCD_CS_HIGH()     HAL_GPIO_WritePin(LCD_CS_GPIO_PORT, LCD_CS_PIN, GPIO_PIN_SET)
#define LCD_DC_LOW()      HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_RESET)
#define LCD_DC_HIGH()     HAL_GPIO_WritePin(LCD_DC_GPIO_PORT, LCD_DC_PIN, GPIO_PIN_SET)
#define LCD_RST_LOW()     HAL_GPIO_WritePin(LCD_RST_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_RESET)
#define LCD_RST_HIGH()    HAL_GPIO_WritePin(LCD_RST_GPIO_PORT, LCD_RST_PIN, GPIO_PIN_SET)

/**
  * @brief  SD Control Interface pins (shield D4)
  */
#define SD_CS_PIN                                 GPIO_PIN_5
#define SD_CS_GPIO_PORT                           GPIOB
#define SD_CS_GPIO_CLK_ENABLE()                   __HAL_RCC_GPIOB_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE()                  __HAL_RCC_GPIOB_CLK_DISABLE()


/**
  * @brief  LCD Control Interface pins (shield D10)
  */
#define LCD_CS_PIN                                 GPIO_PIN_4
#define LCD_CS_GPIO_PORT                           GPIOA
#define LCD_CS_GPIO_CLK_ENABLE()                   __HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD_CS_GPIO_CLK_DISABLE()                  __HAL_RCC_GPIOA_CLK_DISABLE()

/**
  * @brief  LCD Data/Command Interface pins
  */
#define LCD_DC_PIN                                 GPIO_PIN_1
#define LCD_DC_GPIO_PORT                           GPIOA
#define LCD_DC_GPIO_CLK_ENABLE()                   __HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD_DC_GPIO_CLK_DISABLE()                  __HAL_RCC_GPIOA_CLK_DISABLE()

/**
  * @brief  LCD Reset Pin
  */
#define LCD_RST_PIN                                 GPIO_PIN_3
#define LCD_RST_GPIO_PORT                           GPIOA
#define LCD_RST_GPIO_CLK_ENABLE()                   __HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD_RST_GPIO_CLK_DISABLE()                  __HAL_RCC_GPIOA_CLK_DISABLE()

/*##################### ADC1 ###################################*/
/**
  * @brief  ADC Interface pins
  *         used to detect motion of Joystick available on Adafruit 1.8" TFT shield
  */
#define NUCLEO_ADCx                                 ADC1
#define NUCLEO_ADCx_CLK_ENABLE()                    __HAL_RCC_ADC1_CLK_ENABLE()
#define NUCLEO_ADCx_CLK_DISABLE()                 __HAL_RCC_ADC1_CLK_DISABLE()
    
/*#define NUCLEO_ADCx_GPIO_PORT                       GPIOB
#define NUCLEO_ADCx_GPIO_PIN                        GPIO_PIN_0
#define NUCLEO_ADCx_GPIO_CLK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_ADCx_GPIO_CLK_DISABLE()              __HAL_RCC_GPIOB_CLK_DISABLE()*/
    
/**
  * @}
  */
    

/**
  * @}
  */
    
/** @addtogroup STM32F1XX_NUCLEO_Exported_Functions
  * @{
  */
uint32_t        BSP_GetVersion(void);
/** @addtogroup STM32F1XX_NUCLEO_LED_Functions
  * @{
  */ 

void            BSP_TFT_BACKLED_Init(Led_TypeDef Led);
void            BSP_TFT_BACKLED_DeInit(Led_TypeDef Led);
void            BSP_TFT_BACKLED_On(Led_TypeDef Led);
void            BSP_TFT_BACKLED_Off(Led_TypeDef Led);
void            BSP_TFT_BACKLED_Toggle(Led_TypeDef Led);

/**
  * @}
  */

/** @addtogroup STM32F1XX_NUCLEO_BUTTON_Functions
  * @{
  */

void             BSP_PB_Init(Button_TypeDef Button, ButtonMode_TypeDef ButtonMode);
void             BSP_PB_DeInit(Button_TypeDef Button);
uint32_t         BSP_PB_GetState(Button_TypeDef Button);
#if defined(HAL_ADC_MODULE_ENABLED)
uint8_t          BSP_JOY_Init(void);
JOYState_TypeDef BSP_JOY_GetState(void);
void             BSP_JOY_DeInit(void);
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

#ifdef __cplusplus
}
#endif

#endif /* __STM32F103C8T6_MINIMAL_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
