/*
 * rmlib.h
 *
 * Created: 4/19/2022 9:56:26 PM
 */ 


#ifndef RMLIB_H_
#define RMLIB_H_

#include <string.h>
#include <stdint.h>

#define true  1
#define false 0


#define SIZE_RUN_TIME_BUFFER_QUEUE		1000

#define NUMBER_TASKS_RUNTIME_VERIFICATION	5


#define COMMAND_OK						0			//Queued command was executed successfully (Insert/Remove)
#define COMMAND_NOK						1			//Queued command cannot be executed (Insert with full queue / Remove with empty queue)
#define COMMAND_ERROR					-1			//Error in the pointers of the queue that needs to be reinitialized

#define TASK_INIT_EXECUTION				0
#define TASK_END_EXECUTION				1

#define				ONLINE_VERIFICATION

// Use this define to improve the optimization of data export
//#define				OPTIMIZE_EXPORT_DATA

#ifndef ONLINE_VERIFICATION
#define				OFFLINE_VERIFICATION
#endif

#ifdef ONLINE_VERIFICATION

// Use this define to export only in cases of violating temporal restrictions
#define EXPORT_ONLY_RTOS_ERRORS

#ifdef EXPORT_ONLY_RTOS_ERRORS
#define EXPORT_DUMP_STATUS_TASKS
#endif

//TimeStamp Veredict of system RunTime Verification
typedef struct __attribute__((packed))
{
	uint16_t		TimeStamp;
	uint16_t		CounterTask;
	uint16_t		ExecutionTime;
	uint8_t			Identifier_of_Task : 5;
	uint8_t			Status_of_WCET_Task : 2;
	uint8_t			Status_of_DeadLine_Task : 2;
	uint8_t			ReservedBits : 7;
} TimeStampVeredict_t;
#endif

//EventTimeStamp of system RunTime Verification
typedef struct __attribute__((packed))
{
	uint32_t		TimeStamp;
	uint32_t		CounterTask : 25;
	uint8_t			Identifier_of_Task : 5;
	uint8_t			State_of_Task : 2;
} EventTimeStamp_t;


int8_t timestamp_runtime(uint32_t task_identifier,uint16_t task_state);

/**
  * \brief Returns the value of the infinite Miliseconds counter.
  * \return Miliseconds counter value.
  */
extern uint32_t ReadCounterMiliSeconds(void);

extern int printf(const char * restrict format, ... );


#ifdef ONLINE_VERIFICATION
void rmlib_init(uint32_t * tasks_identifiers,uint32_t * deadlines_service,uint32_t * wcet_service);

int8_t rmlib_export_data(TimeStampVeredict_t * buffer_rmlib);

const char rmlib_export_data_string(TimeStampVeredict_t * buffer_rmlib);

#ifdef EXPORT_DUMP_STATUS_TASKS
extern void DumpStatusTasks(void);
#endif

#endif

#ifdef OFFLINE_VERIFICATION
void rmlib_init();

int8_t rmlib_export_data(EventTimeStamp_t * buffer_rmlib);

const char rmlib_export_data_string(EventTimeStamp_t * buffer_rmlib);
#endif

#endif /* RMLIB_H_ */