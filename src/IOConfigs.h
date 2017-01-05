/*
 * IOConfigs.h
 *
 *  Created on: 2017. jan. 4.
 *      Author: bekeband
 */

#ifndef IOCONFIGS_H_
#define IOCONFIGS_H_

/** @defgroup STM32F1XX_NUCLEO_BUTTON BUTTON Constants
  * @{
  */
#define BUTTONn                          1

/**
  * @brief User push-button
 */
#define USER_BUTTON_PIN                  GPIO_PIN_13
#define USER_BUTTON_GPIO_PORT            GPIOC
#define USER_BUTTON_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOC_CLK_ENABLE()
#define USER_BUTTON_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOC_CLK_DISABLE()
#define USER_BUTTON_EXTI_IRQn            EXTI15_10_IRQn
/* Aliases */
#define KEY_BUTTON_PIN                   USER_BUTTON_PIN
#define KEY_BUTTON_GPIO_PORT             USER_BUTTON_GPIO_PORT
#define KEY_BUTTON_GPIO_CLK_ENABLE()     USER_BUTTON_GPIO_CLK_ENABLE()
#define KEY_BUTTON_GPIO_CLK_DISABLE()    USER_BUTTON_GPIO_CLK_DISABLE()
#define KEY_BUTTON_EXTI_IRQn             USER_BUTTON_EXTI_IRQn

#define BUTTONx_GPIO_CLK_ENABLE(__INDEX__)    do { if((__INDEX__) == 0) USER_BUTTON_GPIO_CLK_ENABLE();} while(0)
#define BUTTONx_GPIO_CLK_DISABLE(__INDEX__)   (((__INDEX__) == 0) ? USER_BUTTON_GPIO_CLK_DISABLE() : 0)

/** @addtogroup TOUCH (XPT2046 IC) (SPI2) bus Constants
  * @{
  */
/*###################### SPI1 ###################################*/
#define TOUCH_SPIx                    	SPI2
#define TOUCH_SPIx_CLK_ENABLE()          	__HAL_RCC_SPI2_CLK_ENABLE()

#define TOUCH_SPIx_SCK_GPIO_PORT                   GPIOB
#define TOUCH_SPIx_SCK_PIN                         GPIO_PIN_13
#define TOUCH_SPIx_SCK_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define TOUCH_SPIx_SCK_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()

#define TOUCH_SPIx_MISO_MOSI_GPIO_PORT             GPIOB
#define TOUCH_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define TOUCH_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOA_CLK_DISABLE()
#define TOUCH_SPIx_MISO_PIN                        GPIO_PIN_14
#define TOUCH_SPIx_MOSI_PIN                        GPIO_PIN_15
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define TOUCH_SPIx_TIMEOUT_MAX                   1000

/** @addtogroup LCD TFT (SPI1) bus Constants
  * @{
  */
/*###################### SPI1 ###################################*/
#define LCD_SPIx                    	SPI1
#define LCD_SPIx_CLK_ENABLE()          	__HAL_RCC_SPI1_CLK_ENABLE()
#define DMAx_CLK_ENABLE()               __HAL_RCC_DMA1_CLK_ENABLE()

#define LCD_SPIx_SCK_GPIO_PORT                   GPIOA
#define LCD_SPIx_SCK_PIN                         GPIO_PIN_5
#define LCD_SPIx_SCK_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD_SPIx_SCK_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()

#define LCD_SPIx_MISO_MOSI_GPIO_PORT             GPIOA
#define LCD_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOA_CLK_ENABLE()
#define LCD_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()    __HAL_RCC_GPIOA_CLK_DISABLE()
#define LCD_SPIx_MISO_PIN                        GPIO_PIN_6
#define LCD_SPIx_MOSI_PIN                        GPIO_PIN_7
/* Maximum Timeout values for flags waiting loops. These timeouts are not based
   on accurate values, they just guarantee that the application will not remain
   stuck if the SPI communication is corrupted.
   You may modify these timeout values depending on CPU frequency and application
   conditions (interrupts routines ...). */
#define LCD_SPIx_TIMEOUT_MAX                   1000


/** @defgroup STM32F103C8T6_MINIMAL_LED TFT BACKLITE LED PIN Constants
  * @{
  */
#define LEDn                             1

#define TFT_BACKLED_PIN                		GPIO_PIN_0
#define TFT_BACKLED_GPIO_PORT          		GPIOB
#define TFT_BACKLED_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOB_CLK_ENABLE()
#define TFT_BACKLED_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOB_CLK_DISABLE()

#define LEDx_GPIO_CLK_ENABLE(__INDEX__)   do { if((__INDEX__) == 0) TFT_BACKLED_GPIO_CLK_ENABLE();} while(0)
#define LEDx_GPIO_CLK_DISABLE(__INDEX__)  (((__INDEX__) == 0) ? TFT_BACKLED_GPIO_CLK_DISABLE() : 0)

#define SPIx_IRQn                        SPI1_IRQn
#define SPIx_IRQHandler                  SPI1_IRQHandler

/* Definition for SPIx's DMA */
#define SPIx_TX_DMA_CHANNEL              DMA1_Channel3
#define SPIx_RX_DMA_CHANNEL              DMA1_Channel2


/* Definition for SPIx's NVIC */
#define SPIx_DMA_TX_IRQn                 DMA1_Channel3_IRQn
#define SPIx_DMA_RX_IRQn                 DMA1_Channel2_IRQn

#define SPIx_DMA_TX_IRQHandler           DMA1_Channel3_IRQHandler
#define SPIx_DMA_RX_IRQHandler           DMA1_Channel2_IRQHandler



#endif /* IOCONFIGS_H_ */
