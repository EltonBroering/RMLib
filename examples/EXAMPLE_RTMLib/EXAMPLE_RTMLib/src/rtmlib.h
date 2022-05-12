/*
 * rtmlib.h
 *
 * Created: 4/19/2022 9:56:26 PM
 */ 


#ifndef RTMLIB_H_
#define RTMLIB_H_

#include <asf.h>


#define SIZE_RUN_TIME_BUFFER_QUEUE		1000

#define COMMAND_OK			0			// Comando na fila foi executado com sucesso (Insere / Retira)
#define COMMAND_NOK			1			// Comando na fila não pode ser executado (Insere com fila cheia / Retira com fila vazia)
#define COMMAND_ERROR		-1			// Erro nos ponteiros da fila que necessita ser reinicializada

// TimeStamp of system RunTime Verification
typedef struct PACKED
{
	uint32_t		TimeStamp;
	uint32_t		Identifier_of_Task;
} TimeStamp_t;

// Init RMLib
void rtmlib_init();

int8_t timestamp_runtime(uint32_t TimeStamp,uint32_t Identifier_of_Queue);

#endif /* RTMLIB_H_ */