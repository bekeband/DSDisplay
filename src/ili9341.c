
#include "dsdisplay_config.h"
#include "ili9341.h"
#include "stm32f1xx_hal.h"

/* Buffer used for transmission */
uint8_t aTxBuffer[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00};

/* Buffer used for reception */
uint8_t aRxBuffer[BUFFERSIZE];


/**
  * @brief  Compares two buffers.
  * @param  pBuffer1, pBuffer2: buffers to be compared.
  * @param  BufferLength: buffer's length
  * @retval 0  : pBuffer1 identical to pBuffer2
  *         >0 : pBuffer1 differs from pBuffer2
  */

uint16_t Buffercmp(uint8_t* pBuffer1, uint8_t* pBuffer2, uint16_t BufferLength)
{
  while (BufferLength--)
  {
    if((*pBuffer1) != *pBuffer2)
    {
      return BufferLength;
    }
    pBuffer1++;
    pBuffer2++;
  }

  return 0;
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
    /* Toogle LED2 for error */
//      BSP_LED_Toggle(LED2);
//	asm("break");
    HAL_Delay(1000);
  }
}

void ReadTFTRegister(uint8_t command, uint8_t readlength)
{
  LCD_CS_LOW();
  HAL_Delay(1);
  SendCommand(command);
  SendData(0x00);
  switch(HAL_SPI_TransmitReceive(GetTFTHandlePtr(), (uint8_t*)aTxBuffer, (uint8_t *)aRxBuffer, readlength, 5000))
  {
    case HAL_OK:
    /* Turn LED2 on: Transfer in transmission/Reception process is correct */
//      BSP_LED_On(LED2);
//      BSP_TFT_BACKLED_Toggle(LED2);
    break;

  case HAL_TIMEOUT:
    /* An Error Occur ______________________________________________________ */
  case HAL_ERROR:
    /* Call Timeout Handler */
    Error_Handler();
    break;
  default:
    break;
  }
  LCD_CS_HIGH();
}


void SendSPIData(uint16_t data)
{
  SPIx_WriteData((uint8_t *)&data, 1);
};

void SendData(uint16_t data)
{
  LCD_DC_HIGH();
  SendSPIData(data);
}

void SendCommand(uint8_t data)
{
  LCD_DC_LOW();
  SendSPIData(data);
}


