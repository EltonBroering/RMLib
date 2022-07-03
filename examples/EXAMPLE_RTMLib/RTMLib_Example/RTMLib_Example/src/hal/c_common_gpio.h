/**
  ******************************************************************************
  * @file    modules/common/c_common_gpio.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Fuções de configuração e uso de GPIO baseadas na CMSIS do STM32F4.
  *
  *	TODO
  *
  * \todo	 1 - Incluir funções de Input em GPIOs.
  * \todo	 2 - ...
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_COMMON_GPIO_H
#define C_COMMON_GPIO_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_conf.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

/** \brief Tipo struct para pinos de GPIO. */
typedef struct GPIO {
	 GPIO_TypeDef* 		Port;
	 uint16_t 			Pin;
	 GPIOMode_TypeDef 	Mode;
} GPIOPin;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
GPIOPin c_common_gpio_init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOMode_TypeDef GPIO_Mode);

/* Header-defined wrapper functions ----------------------------------------- */
/** @addtogroup Common_Components
  * @{
  */
/** @addtogroup Common_Components_GPIO
  * @{
  */

/** \brief Wrapper para \b GPIO_SetBits .
 *  @param GPIOPin Pino a ser usado.
 */
static inline void c_common_gpio_set(GPIOPin gpio) { GPIO_SetBits(gpio.Port, gpio.Pin); }

/** \brief Wrapper para \b GPIO_ToggleBits .
 *  @param GPIOPin Pino a ser usado.
 */
static inline void c_common_gpio_toggle(GPIOPin gpio) { GPIO_ToggleBits(gpio.Port, gpio.Pin); }

/** \brief Wrapper para \b GPIO_ResetBits .
 *  @param GPIOPin Pino a ser usado.
 */
static inline void c_common_gpio_reset(GPIOPin gpio) { GPIO_ResetBits(gpio.Port, gpio.Pin); }

/**
  * @}
  */
/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif //C_COMMON_GPIO_H
