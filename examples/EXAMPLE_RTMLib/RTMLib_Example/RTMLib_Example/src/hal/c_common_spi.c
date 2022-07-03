/**
  ******************************************************************************
  * @file    modules/common/c_common_spi.c
  * @author  Patrick José Pereira
  * @version V1.0.0
  * @date    4-Junho-2014
  * @brief   Implementação da SPI
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_common_spi.h"

/** @addtogroup Common_Components
  * @{
  */

/** @addtogroup Common_Components_Debug
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions definitions --------------------------------------------*/


/** \brief Função da inicialização de SPI3
 *
 * @param SPI3        Definição de qual SPI3
 *
 */

void init_SPI(SPI_TypeDef* SPIx)
{

	if(SPIx==SPI3)
	{
		#ifdef STM32F4_DISCOVERY
		    GPIO_InitTypeDef GPIO_InitStruct;
		    SPI_InitTypeDef SPI_InitStruct;

		    // enable clock for used IO pins
		    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

		    /* configure pins used by SPI3
		     * PB3 = SCK
		     * PB4 = MISO
		     * PB5 = MOSI
		     */
		    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
		    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
		    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
		    GPIO_Init(GPIOB, &GPIO_InitStruct);

		    // connect SPI3 pins to SPI alternate function
		    GPIO_PinAFConfig(GPIOB, GPIO_PinSource3, GPIO_AF_SPI1);
		    GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_SPI1);
		    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_SPI1);

		    // enable clock for used IO pins
		    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

		    /* Configure the chip select pin */
		    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
		    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
		    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
		    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
		    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
		    GPIO_Init(GPIOB, &GPIO_InitStruct);

		    GPIOB->BSRRL |= GPIO_Pin_5; // set pin 5 high

		    // enable peripheral clock
		    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);

		    /* configure SPI3 in Mode 0 
		     * CPOL = 0 --> clock is low when idle
		     * CPHA = 0 --> data is sampled at the first edge
		     */
		    SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
		    SPI_InitStruct.SPI_Mode = SPI_Mode_Slave;    
		    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
		    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
		    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at first edge
		    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
		    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // SPI frequency is APB1 frequency / 4
		    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
		    SPI_Init(SPI3, &SPI_InitStruct); 

		    SPI_Cmd(SPI3, ENABLE); // enable SPI3
	    #endif
	}
}

/** \brief Função da inicialização de SPI3
 *
 * @param SPI3        SPIx
 * @retval Byte retorna data recebida do registrador SPIx.
 *
 */
uint8_t SPI_send(SPI_TypeDef* SPIx ,uint8_t data)
{

	SPIx->DR = data; // write data to be transmitted to the SPI data register
	while( !(SPIx->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	while( !(SPIx->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	while( SPIx->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	return SPIx->DR; // return received data from SPI data register
}


/* IRQ handlers ------------------------------------------------------------- */


/**
  * @}
  */

/**
  * @}
  */