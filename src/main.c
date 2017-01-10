/**
  ******************************************************************************
  * @file    Templates/Src/main.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    29-April-2016
  * @brief   Main program body
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

#include "main.h"
#include "ili9341.h"

extern struct s_image button;

/** @addtogroup STM32F1xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */

/* Private variables ---------------------------------------------------------*/
/* SPI handler declaration */
SPI_HandleTypeDef SpiHandle;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void Error_Handler(void);

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
//
// Recommended memory to run the sample with adequate performance
//
#define RECOMMENDED_MEMORY (1024L * 5)




/* Private functions ---------------------------------------------------------*/




/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

  FRESULT res;                                          /* FatFs function common result code */
  uint32_t byteswritten, bytesread;                     /* File write/read counts */
  uint8_t wtext[] = "This is STM32 working with FatFs"; /* File write buffer */
  uint8_t rtext[100];                                   /* File read buffer */


  /* STM32F103xB HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user 
         can eventually implement his proper time base source (a general purpose 
         timer for example or other time source), keeping in mind that Time base 
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the system clock to 64 MHz */
  SystemClock_Config();
#define Z_THRESHOLD     400
  /* Add your application code here
     */
  BSP_TFT_BACKLED_Init(LED_GREEN);
  LCD_IO_Init();
  TOUCH_IO_Init();
//  SD_IO_Init();

  /*##-1- Link the micro SD disk I/O driver ##################################*/
  if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)
  {
      if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) == FR_OK)
      {
	  /*##-3- Create a FAT file system (format) on the logical drive #########*/
	  /* WARNING: Formatting the uSD card will delete all content on the device */
	  if(f_mkfs((TCHAR const*)SDPath, 0, 0) == FR_OK)
	  {
	      /*##-4- Create and Open a new text file object with write access #####*/
/*	      if(f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
	      {

	      } else*/
		  /* 'STM32.TXT' file Open for write Error */
		Error_Handler();
	  } else
	    /* FatFs Format Error */
	    Error_Handler();
      } else
        /* FatFs Initialization Error */
        Error_Handler();
  };


  uint8_t COMM;
  uint8_t i;
  uint16_t RD;
  uint16_t z;
  uint16_t DT[6];

  TOUCH_CS_LOW();
  SD_IO_WriteByte(0xB1);
  SD_IO_WriteByte(0xC1);
  SD_IO_ReadData((uint8_t*)&RD, 2);
  RD = RD >> 3;
  z = RD + 4095;
  SD_IO_WriteByte(0x91);
  SD_IO_ReadData((uint8_t*)&RD, 2);
  RD = RD >> 3;
  z -= RD;
  if (z >= Z_THRESHOLD)
    {
      SD_IO_WriteByte(0x91);
      SD_IO_ReadData((uint8_t*)&RD, 2);	// dummy X measure, 1st is always noisy
      SD_IO_WriteByte(0xD1);
      SD_IO_ReadData((uint8_t*)&RD, 2);	// dummy X measure, 1st is always noisy
      DT[0] = RD >> 3;
      SD_IO_WriteByte(0x91);
      SD_IO_ReadData((uint8_t*)&RD, 2);	// dummy X measure, 1st is always noisy
      DT[1] = RD >> 3;
      SD_IO_WriteByte(0xD1);
      SD_IO_ReadData((uint8_t*)&RD, 2);	// dummy X measure, 1st is always noisy
      DT[2] = RD >> 3;
      SD_IO_WriteByte(0x91);
      SD_IO_ReadData((uint8_t*)&RD, 2);	// dummy X measure, 1st is always noisy
      DT[3] = RD >> 3;
    }
  else { DT[0] = DT[1] = DT[2] = DT[3] = 0;};	// Compiler warns these values may be used unset on early exit.
    SD_IO_WriteByte(0x91);
    SD_IO_ReadData((uint8_t*)&RD, 2);	// dummy X measure, 1st is always noisy
    DT[4] = RD >> 3;
    SD_IO_WriteByte(0x00);
    SD_IO_ReadData((uint8_t*)&RD, 2);	// dummy X measure, 1st is always noisy
    DT[5] = RD >> 3;
    z = 0;
  TOUCH_CS_HIGH();

  InitLCD();



  DisplayOn();
  DrawGIMPImage(RD, 20, button);
  while (1);
  /* Infinite loop */
  while (1)
  {
      DisplayOn();
      FillRectangle(0, 0, 240, 320, 0b1111100000000000);
      HAL_Delay(50);
//      DisplayOff();
      FillRectangle(0, 0, 240, 320, 0b0000011111110000);
      HAL_Delay(50);
      FillRectangle(0, 0, 240, 320, 0b0000000000011111);
      HAL_Delay(50);
//      BSP_TFT_BACKLED_Toggle(LED2);
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Configure PLL ------------------------------------------------------*/
  /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
  /* Enable HSI and activate PLL with HSi_DIV2 as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
  oscinitstruct.HSEState        = RCC_HSE_OFF;
  oscinitstruct.LSEState        = RCC_LSE_OFF;
  oscinitstruct.HSIState        = RCC_HSI_ON;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI_DIV2;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  while(1)
  {
    /* Toggle LED_RED fast */
//    BSP_LED_Toggle(LED_RED);
    HAL_Delay(40);
  }
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
