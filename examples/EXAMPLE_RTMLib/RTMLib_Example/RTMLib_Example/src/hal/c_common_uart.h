/**
  ******************************************************************************
  * @file    modules/common/c_common_uart.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Funcões para configuração de UART, para uso em outros módulos.
  *	TODO
  *
  * \todo	 1 - Programar os wrappers para UART3 (dos servos?) .
  * \todo	 2 - ...
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_COMMON_UART_H
#define C_COMMON_UART_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void c_common_usart1_init(int baudrate);
void c_common_usart2_init(int baudrate);
void c_common_usart3_init(int baudrate);
void c_common_usart6_init(int baudrate);
void c_common_usart_puts(USART_TypeDef* USARTx, volatile char *s);
void c_common_usart_putchar(USART_TypeDef* USARTx, volatile char c);
bool c_common_usart_available(USART_TypeDef* USARTx);
int c_common_usart_available2(USART_TypeDef* USARTx);
unsigned char c_common_usart_read(USART_TypeDef* USARTx);
void c_common_usart_flush(USART_TypeDef* USARTx);

/* Header-defined wrapper functions ----------------------------------------- */
/** @addtogroup Common_Components
  * @{
  */

/** @addtogroup Common_Components_UART
  * @{
  */

/** \brief Wrapper para \b USART_ITConfig. */
static inline void c_common_usart_it_set(USART_TypeDef* USARTx, uint16_t USART_IT, FunctionalState NewState) { USART_ITConfig(USARTx, USART_IT, NewState); }

/**
  * @}
  */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif //C_COMMON_UART_H
