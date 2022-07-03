/**
  ******************************************************************************
  * @file    modules/common/c_common_i2c.c
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    30-Dezember-2013
  * @brief   Implmentacão das funções de I2C.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_common_i2c.h"

/** @addtogroup Common_Components
  * @{
  */

/** @addtogroup Common_Components_I2C
  *  \brief Funções para uso da I2C (I2C1)
  *
  *  \todo Embutir timeouts nos whiles (travam a thread quando o dipositivo desejado não está no bus).
  *
  *  Posto que apenas a I2C1 será usada, as funções terão esta "hard-coded", via define:
  *
  *  \code{.c}
  *  #define I2Cx I2C1 //tipo I2C_TypeDef*
  *  \endcode
  *
  *  Fonte da biblioteca, <a href="https://github.com/Torrentula/STM32F4-examples/blob/master/I2C%20Master/main.c">aqui</a>. Exemplo de utilização:
  *  \code{.c}
  *  #define SLAVE_ADDRESS 0x3D // the slave address (example)
  *
  *	 int main(void){
  *
  *        init_I2C1(); // initialize I2C peripheral
  *
  *        uint8_t received_data[2];
  *
  *        while(1){
  *
  *                c_common_i2c_start(SLAVE_ADDRESS<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
  *                																	// shifts address for 7 bit mode (makes room for R/W bit)
  *                c_common_i2c_write(0x20); // write one byte to the slave
  *                c_common_i2c_write(0x03); // write another byte to the slave
  *                c_common_i2c_stop(); // stop the transmission
  *
  *                c_common_i2c_start(SLAVE_ADDRESS<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
  *                received_data[0] = I2C_read_ack(I2C1); // read one byte and request another byte
  *                received_data[1] = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission
  *       }
  *	 }
  *	 \endcode
  *  @{
  */

/* Private typedef -----------------------------------------------------------*/
//#define I2Cx 	I2C1 
long whileTimeoutCounter = 0;
long timeoutCounter = 0;
bool lastTimeoutExpired  = 0;

/* Private define ------------------------------------------------------------*/
#define TIMEOUT_MS 1
/* Private macro -------------------------------------------------------------*/
#define while_timout(cond,time)
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** \brief Funcao para timeout da i2c1
 *
 * @param cond        Retorno booleano 
 * @param startime    Tempo de inicio 
 *
 * @retval retorna o booleano da funcao caso esteja dentro do timeoout, caso contrario retorna 0
 * @todo existe alguns problemas onde nunca si dessa funcao , resolver
 */

bool while_timeout(bool cond, long startime) {
  long unsigned int time_diff1 = c_common_utils_millis();
//  long unsigned int time_diff1 = c_common_utils_micros();
  long unsigned int time_diff = time_diff1 - startime ;
  if(time_diff > TIMEOUT_MS)
    { lastTimeoutExpired = 1; return 0; }
  else
    { lastTimeoutExpired = 0; return cond; }
}



/* Exported functions definitions --------------------------------------------*/

/** \brief Inicializa a I2C1 em PB8 e PB9 (SCL e SDA).
 *
 * @param I2Cx  Escolha da I2C a ser inicializada.
 *
 */
void c_common_i2c_init(I2C_TypeDef* I2Cx){

  if(I2Cx==I2C1)
  {
        GPIO_InitTypeDef GPIO_InitStruct;
        I2C_InitTypeDef I2C_InitStruct;

        // enable APB1 peripheral clock for I2C1
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        // enable clock for SCL and SDA pins
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

        /* setup SCL and SDA pins
         * You can connect I2C1 to two different
         * pairs of pins:
         * 1. SCL on PB6 and SDA on PB7
         * 2. SCL on PB8 and SDA on PB9 <-----------
         */
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;   // GPIO Alternate function Mode 
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; // set output to open drain --> the line has to be only pulled low, not driven high
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;   // enable pull up resistors
        GPIO_Init(GPIOB, &GPIO_InitStruct);         // init GPIOB

        // Connect I2C1 pins to AF
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1); // SCL
        GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1); // SDA

        // configure I2C1
        I2C_InitStruct.I2C_ClockSpeed = 400000; // 400kHz    foi modificado para ser mais rapido a leitura da imu na discovery
        I2C_InitStruct.I2C_Mode = I2C_Mode_I2C; // I2C mode
        I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // 50% duty cycle --> standard
        I2C_InitStruct.I2C_OwnAddress1 = 0x00;         // own address, not relevant in master mode
        I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;     // disable acknowledge when reading (can be changed later on)
        I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
        I2C_Init(I2C1, &I2C_InitStruct);      // init I2C1

        // enable I2C1
        I2C_Cmd(I2C1, ENABLE);
  }
  if(I2Cx==I2C2)
  {
    #ifdef STM32F4_H407
        GPIO_InitTypeDef GPIO_InitStruct;
        I2C_InitTypeDef I2C_InitStruct;

        // enable APB1 peripheral clock for I2C2
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
        // enable clock for SCL and SDA pins
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);

        /* setup SCL and SDA pins
         * You can connect I2C2 to two different
         * pairs of pins:
         * 1. SCL on PF1 and SDA on PF0
         * 2. SCL on PF1 and SDA on PF0 <-----------
         */
        
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; // set output to open drain --> the line has to be only pulled low, not driven high
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;   // enable pull up resistors
        GPIO_Init(GPIOF, &GPIO_InitStruct);         // init GPIOF

        // Connect I2C2 pins to AF
        GPIO_PinAFConfig(GPIOF, GPIO_PinSource1, GPIO_AF_I2C2); // SCL
        GPIO_PinAFConfig(GPIOF, GPIO_PinSource0, GPIO_AF_I2C2); // SDA

        // configure I2C2
        I2C_InitStruct.I2C_ClockSpeed = 100000; // 100kHz
        I2C_InitStruct.I2C_Mode = I2C_Mode_I2C; // I2C mode
        I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2; // 50% duty cycle --> standard
        I2C_InitStruct.I2C_OwnAddress1 = 0x00;      // own address, not relevant in master mode
        I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;     // disable acknowledge when reading (can be changed later on)
        I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
        I2C_Init(I2C2, &I2C_InitStruct);      // init I2C2

        // enable I2C2
        I2C_Cmd(I2C2, ENABLE);
    #endif
  }
  if(I2Cx==I2C3)
  {
    #ifdef STM32F4_DISCOVERY
        GPIO_InitTypeDef GPIO_InitStruct;
        I2C_InitTypeDef I2C_InitStruct;

        // enable APB1 peripheral clock for I2C3
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
        // enable clock for SCL and SDA pins
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

        /* setup SCL and SDA pins
         * You can connect I2C3 to two different
         * pairs of pins:
         * 1. SCL on PA8 and SDA on PC9
         */
        
        GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
        GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStruct.GPIO_OType = GPIO_OType_OD; // set output to open drain --> the line has to be only pulled low, not driven high
        GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;   // enable pull up resistors
        GPIO_Init(GPIOA, &GPIO_InitStruct);         // init GPIOA
        GPIO_Init(GPIOC, &GPIO_InitStruct);         // init GPIOC

        // Connect I2C3 pins to AF
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3); // SCL
        GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3); // SDA

        // configure I2C3
        I2C_InitStruct.I2C_ClockSpeed = 100000;           // 100kHz
        I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;           // I2C mode
        I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;   // 50% duty cycle --> standard
        I2C_InitStruct.I2C_OwnAddress1 = 0x00;            // own address, not relevant in master mode
        I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;         // disable acknowledge when reading (can be changed later on)
        I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
        I2C_Init(I2C3, &I2C_InitStruct);                  // init I2C3

        // enable I2C3
        I2C_Cmd(I2C3, ENABLE);
    #endif
  }
}

/** \brief Emite uma condição de início de transmissão e envia o endereço do escravo com o bit de R/W.
 *
 * @param I2Cx  I2C a ser utilizada.
 * @param address 	Endereço de 7 bits do escravo.
 * @param direction	Direção da transmissão. Pode ser:
 * 						\em I2C_Direction_Transmitter \em para <b> Master transmitter mode </b>, ou
 * 						\em I2C_Direction_Receiver \em para <b> Master receiver mode</b>.
 */
void c_common_i2c_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
    taskENTER_CRITICAL();
        // wait until I2C1 is not busy anymore
		timeoutCounter = c_common_utils_millis();
//		timeoutCounter = c_common_utils_micros();
        while(while_timeout(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY), timeoutCounter));

        // Send I2C1 START condition
        I2C_GenerateSTART(I2Cx, ENABLE);

        // wait for I2C1 EV5 --> Slave has acknowledged start condition
        timeoutCounter = c_common_utils_millis();
//        timeoutCounter = c_common_utils_micros();
        while(while_timeout(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT), timeoutCounter));

        if(!lastTimeoutExpired) {
			// Send slave Address for write
			I2C_Send7bitAddress(I2Cx, address, direction);

			/* wait for I2C1 EV6, check if
			 * either Slave has acknowledged Master transmitter or
			 * Master receiver mode, depending on the transmission
			 * direction
			 */
			if(direction == I2C_Direction_Transmitter){
					timeoutCounter = c_common_utils_millis();
//					timeoutCounter = c_common_utils_micros();
					while(while_timeout(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED), timeoutCounter));
			}
			else if(direction == I2C_Direction_Receiver){
					timeoutCounter = c_common_utils_millis();
//					timeoutCounter = c_common_utils_micros();
					while(while_timeout(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED), timeoutCounter));
			}
        }
}

/** \brief Envia um byte ao escravo.
 *
 * @param I2Cx  I2C a ser utilizada.
 * @param data Byte a ser enviado.
 */
void c_common_i2c_write(I2C_TypeDef* I2Cx, uint8_t data) {
        I2C_SendData(I2Cx, data);
        // wait for I2C1 EV8_2 --> byte has been transmitted
        timeoutCounter = c_common_utils_millis();
//        timeoutCounter = c_common_utils_micros();
        while(while_timeout(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED), timeoutCounter));
}

/** \brief Lê um byte do escravo e confima (acknowledges) o byte (requisita um próximo).
 *
 * @param I2Cx  I2C a ser utilizada.
 * @retval Byte lido.
 */
uint8_t c_common_i2c_readAck(I2C_TypeDef* I2Cx) {
		uint8_t data = 0x00;
        // enable acknowledge of recieved data
        I2C_AcknowledgeConfig(I2Cx, ENABLE);
        // wait until one byte has been received
        timeoutCounter = c_common_utils_millis();
//        timeoutCounter = c_common_utils_micros();
        while(while_timeout(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED), timeoutCounter));
        // read data from I2C data register and return data byte
        if(!lastTimeoutExpired)
        	data = I2C_ReceiveData(I2Cx);
        return data;
}

/** \brief Lê um byte do escravo mas não confima (doesn't acknowledges) o byte.
 *
 * @param I2Cx  I2C a ser utilizada.
 * @retval Byte lido.
 */
uint8_t c_common_i2c_readNack(I2C_TypeDef* I2Cx) {
		uint8_t data = 0x00;
        // disabe acknowledge of received data
        // nack also generates stop condition after last byte received
        // see reference manual for more info
        I2C_AcknowledgeConfig(I2Cx, DISABLE);
        I2C_GenerateSTOP(I2Cx, ENABLE);
        // wait until one byte has been received
        timeoutCounter = c_common_utils_millis();
//        timeoutCounter = c_common_utils_micros();
        while(while_timeout(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED), timeoutCounter));
        // read data from I2C data register and return data byte
        if(!lastTimeoutExpired)
        	data = I2C_ReceiveData(I2Cx);
        return data;
}

/** \brief Emite uma condição de parada e libera o barramento.
 * @param I2Cx  I2C a ser utilizada.
 */
void c_common_i2c_stop(I2C_TypeDef* I2Cx) {
        // Send I2C1 STOP Condition
        I2C_GenerateSTOP(I2Cx, ENABLE);
        taskEXIT_CRITICAL();
}

/** \brief Lê uma quantidade de bytes de um dado endereço em um determinado dispositivo.
 *
 *	Exemplo: lê 1 byte a partir do endereço de memória 0x00 do dispositivo com endereço
 *	0x68 no barramento, e salva o resultado em ITG3205_ID.
 *	\code{.c}
 *	c_common_i2c_readBytes(0x68, 0x00, 1, &ITG3205_ID);
 *	\endcode
 *
 *  @param I2Cx  I2C a ser utilizada.
 *	@param device Endereço do dispositivo no barramento.
 *	@param address Endereço de memória a ser lido (comando antes da leitura).
 *	@param bytesToRead Quantos bytes são esperados.
 *	@param recvBuffer Ponteiro para um buffer com tamanho mínimo de \b bytesToRead, no qual a resposta será armazenada.
 *
 */
void c_common_i2c_readBytes(I2C_TypeDef* I2Cx, uint8_t device, uint8_t address, char bytesToRead, uint8_t * recvBuffer) {
	c_common_i2c_start(I2Cx, device<<1, I2C_Direction_Transmitter);
	c_common_i2c_write(I2Cx, address);
	c_common_i2c_stop(I2Cx);

	c_common_i2c_start(I2Cx, device<<1, I2C_Direction_Receiver);
	for(int i=0; i<bytesToRead-1; i++)
		recvBuffer[i] = c_common_i2c_readAck(I2Cx);

	recvBuffer[bytesToRead-1] = c_common_i2c_readNack(I2Cx);
	c_common_i2c_stop(I2Cx);
}

/** \brief Escreve um byte num dispositivo com um dado endereço.
 *
 * @param I2Cx  I2C a ser utilizada.
 * @param device Endereço do dispositivo no barramento.
 * @param address Endereço a ser escrito no dispositivo.
 * @param byteToWrite Byte a ser escrito.
 */
void c_common_i2c_writeByte(I2C_TypeDef* I2Cx, uint8_t device, uint8_t address, uint8_t byteToWrite) {
	c_common_i2c_start(I2Cx, device<<1, I2C_Direction_Transmitter);
	c_common_i2c_write(I2Cx, address);
	c_common_i2c_write(I2Cx, byteToWrite);
	c_common_i2c_stop(I2Cx);
}

/** \brief Escreve apenas um bit em um dado endereço de um dispositivo.
 *
 * @param I2Cx  I2C a ser utilizada.
 * @param device Endereço do dispositivo no barramento.
 * @param address Endereço do byte no qual o bit será escrito.
 * @param bit Posição do bit a ser escrito (0..7).
 * @param value Valor do bit a ser escrito (0 ou 1).
 */
void c_common_i2c_writeBit(I2C_TypeDef* I2Cx ,uint8_t device, uint8_t address, uint8_t bit, bool value) {
	uint8_t byteBuffer;
	c_common_i2c_readBytes(I2Cx, device, address, 1, &byteBuffer);
	byteBuffer = (value == 0)? (byteBuffer & (1<<bit)) : (byteBuffer | (1<<bit));
	c_common_i2c_writeByte(I2Cx, device, address,byteBuffer);
}

/* IRQ handlers ------------------------------------------------------------- */


/**
  * @}
  */

/**
  * @}
  */

