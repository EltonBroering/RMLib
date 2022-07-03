/**
  ******************************************************************************
  * @file    modules/common/c_common_gpio.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-November-2013
  * @brief   Implementação das funções de GPIO.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_common_gpio.h"

/** @addtogroup Common_Components
  * @{
  */

/** @addtogroup Common_Components_GPIO
  * \brief Funções de abstração e uso do GPIO.
  *
  * Define o tipo GPIOPin, seus respectivos contrutores e funções.
  * \code{.c}
  * GPIOPin LED;
  * LED = c_common_gpio_init(GPIOC, GPIO_Pin_13, GPIO_Mode_OUT);
  *
  * void main() {
  * 	while(1) {
  * 		c_common_gpio_set(LED); //acende LED
  * 		delay(1000); //função delay exemplo, não implementada
  * 		c_common_gpio_reset(LED); //apaga LED
  * 		delay(1000);
  * 	}
  * }
  * \endcode
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/


/** \brief Inicializa o pino de GPIO na porta e com a função desejada.
  * Constrói uma struct do tipo GPIOPin.
  *	Exemplo de utilização:
  *	\code{.c} GPIOPin LED = c_common_gpio_init(GPIOC, GPIO_Pin_13, GPIO_Mode_OUT); \endcode
  *
  * @param  GPIOx 	  Porta a ser configurada.
  * @param 	GPIO_Pin  Número do pino desejado.
  * @param 	GPIO_Mode Função do pino.
  * @retval GPIOPin inicializado.
  */
GPIOPin c_common_gpio_init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, GPIOMode_TypeDef GPIO_Mode) {
	GPIOPin createdPin;
	createdPin.Port = GPIOx;
	createdPin.Pin  = GPIO_Pin;
	createdPin.Mode = GPIO_Mode;

	GPIOPuPd_TypeDef PuPdMode = 0;
	GPIO_InitTypeDef  GPIO_InitStructure;

	if(GPIOx == GPIOA)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	if(GPIOx == GPIOB)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	if(GPIOx == GPIOC)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	if(GPIOx == GPIOD)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	if(GPIOx == GPIOE)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	if(GPIOx == GPIOF)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	if(GPIOx == GPIOG)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);

	switch(GPIO_Mode)
	{
	case GPIO_Mode_OUT:
		PuPdMode = GPIO_PuPd_NOPULL; //digital output. Not using open drain mode as I do not know how that operates
		break;
	case GPIO_Mode_IN:
		PuPdMode = GPIO_PuPd_NOPULL; //digital read have Pin as input floating
		break;
	case GPIO_Mode_AN:
		PuPdMode = GPIO_PuPd_NOPULL; //for analog read have Pin as input floating
		break;
	case GPIO_Mode_AF: //need to do a remaping if using alternate functions
		PuPdMode = GPIO_PuPd_UP; //for PWM have not looked at accounting for the various other alternate functions
		break;
	}

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //used for digital output and PWM output
	//this setting does not matter for ADC and digital read
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = PuPdMode;
	GPIO_Init(GPIOx, &GPIO_InitStructure);

	return createdPin;
}


/* IRQ handlers ------------------------------------------------------------- */


/**
  * @}
  */

/**
  * @}
  */

