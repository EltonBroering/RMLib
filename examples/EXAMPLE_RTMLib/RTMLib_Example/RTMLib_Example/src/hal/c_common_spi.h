/**
  ******************************************************************************
  * @file    modules/common/c_common_spi.h
  * @author  Patrick José Pereira
  * @version V1.0.0
  * @date    4-Junho-2014
  * @brief   Implementação da SPI
  ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_COMMON_SPI_H
#define C_COMMON_SPI_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "c_common_gpio.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void init_SPI(SPI_TypeDef* SPIx);
uint8_t SPI_send(SPI_TypeDef* SPIx ,uint8_t data);

#ifdef __cplusplus
}
#endif

#endif //C_COMMON_DEBUG_H
