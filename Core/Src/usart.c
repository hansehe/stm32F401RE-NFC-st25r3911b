/******************************************************************************
  * \attention
  *
  * <h2><center>&copy; COPYRIGHT 2019 STMicroelectronics</center></h2>
  *
  * Licensed under ST MYLIBERTY SOFTWARE LICENSE AGREEMENT (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        www.st.com/myliberty
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied,
  * AND SPECIFICALLY DISCLAIMING THE IMPLIED WARRANTIES OF MERCHANTABILITY,
  * FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
******************************************************************************/

/*! \file
 *
 *  \author 
 *
 *  \brief UART communication handling implementation.
 *
 */

/* Includes ------------------------------------------------------------------*/

#include "usart.h"
#include "../../Drivers/BSP/Components/ST25R3911/st_errno.h"

#define USART_TIMEOUT          1000

UART_HandleTypeDef *pUsart = 0;

/**
  * @brief  This function initalize the UART handle.
	* @param	husart : already initalized handle to USART HW
  * @retval none :
  */
void UsartInit(UART_HandleTypeDef *husart)
{
    pUsart = husart;
}

/**
  * @brief  This function Transmit one data byte via USART
	* @param	data : data to be transmitted
  * @retval ERR_INVALID_HANDLE : in case the SPI HW is not initalized yet
  * @retval others : HAL status
  */
uint8_t UsartTxByte(uint8_t data)
{
  if(pUsart == 0)
    return ERR_INVALID_HANDLE;

  return HAL_UART_Transmit(pUsart, &data, 1, USART_TIMEOUT);
}

/**
  * @brief  This function Transmit data via USART
	* @param	data : data to be transmitted
	* @param	dataLen : length of data to be transmitted
  * @retval ERR_INVALID_HANDLE : in case the SPI HW is not initalized yet
  * @retval others : HAL status
  */
uint8_t UsartTx(uint8_t *data, uint16_t dataLen)
{
  if(pUsart == 0)
    return ERR_INVALID_HANDLE;

  return HAL_UART_Transmit(pUsart, data, dataLen, USART_TIMEOUT);
}

/**
  * @brief  This function Receive data via USART
	* @param	data : data where received data shall be stored
	* @param	dataLen : length of received data
  * @retval ERR_INVALID_HANDLE : in case the SPI HW is not initalized yet
  * @retval others : HAL status
  */
uint8_t UsartRx(uint8_t *data, uint16_t *dataLen)
{
  uint8_t err = ERR_NONE;
  
  if(pUsart == 0)
    return ERR_INVALID_HANDLE;

  for(uint8_t i = 0; i < *dataLen; i++) {
    err |= HAL_UART_Receive(pUsart, &data[i], 1, USART_TIMEOUT);
    if(data[i] == 0) {
      *dataLen = i;
    }
  }
  return err;
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
