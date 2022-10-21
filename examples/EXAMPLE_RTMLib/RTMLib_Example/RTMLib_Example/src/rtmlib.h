/*
 * rtmlib.h
 *
 * Created: 4/19/2022 9:56:26 PM
 */ 


#ifndef RTMLIB_H_
#define RTMLIB_H_

#include "Includes.h"


#define SIZE_RUN_TIME_BUFFER_QUEUE		10000

#define NUMBER_TASKS_RUNTIME_VERIFICATION	5


#define COMMAND_OK						0			//Queued command was executed successfully (Insert/Remove)
#define COMMAND_NOK						1			//Queued command cannot be executed (Insert with full queue / Remove with empty queue)
#define COMMAND_ERROR					-1			//Error in the pointers of the queue that needs to be reinitialized

#define TASK_INIT_EXECUTION				0
#define TASK_END_EXECUTION				1

//#define				ONLINE_VERIFICATION

#define				OPTIMIZE_EXPORT_DATA

#ifndef ONLINE_VERIFICATION
#define				OFFLINE_VERIFICATION
#endif

// TimeStamp of system RunTime Verification
typedef struct PACKED
{
	uint32_t		TimeStamp;
	uint32_t		CounterTask : 25;
	uint8_t			Identifier_of_Task : 5;
	uint8_t			State_of_Task : 2;
} TimeStamp_t;

int8_t timestamp_runtime(uint32_t task_identifier,uint16_t task_state);

/**
  * \brief Returns the value of the infinite Miliseconds counter.
  * \return Miliseconds counter value.
  */
extern uint32_t ReadCounterMiliSeconds(void);

#ifdef OFFLINE_VERIFICATION
// Init RMLib
void rtmlib_init();

int8_t rtmlib_export_data(TimeStamp_t * buffer_rtmlib);

const char rtmlib_export_data_string(TimeStamp_t * buffer_rtmlib);
#endif

#ifdef ONLINE_VERIFICATION
#endif


#endif /* RTMLIB_H_ */