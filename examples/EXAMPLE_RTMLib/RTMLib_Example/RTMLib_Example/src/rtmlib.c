/*
 * rtmlib.c
 *
 * Created: 4/19/2022 9:55:54 PM
 */ 


#include "rtmlib.h"

// Circular queue
typedef struct CircularBuffer
{
	void	*buffer;     // data buffer
	void	*buffer_end; // end of data buffer
	uint32_t	capacity;  // maximum number of items in the buffer
	uint32_t	count;     // number of items in the buffer
	uint32_t	sz;        // size of each item in the buffer
	void	*head;       // pointer to head
	void	*tail;       // pointer to tail
}CircularBuffer_t;

CircularBuffer_t	QueueTimeStamps;		//Circular queue with timestamps


#ifdef ONLINE_VERIFICATION
TimeStampVeredict_t			QueueTimeStampsBuffer[SIZE_RUN_TIME_BUFFER_QUEUE];
TimeStampVeredict_t			TimeStampInsert;
uint16_t counter_tasks_runtime_verification[NUMBER_TASKS_RUNTIME_VERIFICATION];
uint32_t Identifiers_Tasks[NUMBER_TASKS_RUNTIME_VERIFICATION];
uint32_t Vector_WCET_Tasks[NUMBER_TASKS_RUNTIME_VERIFICATION];
uint32_t Vector_Deadline_Tasks[NUMBER_TASKS_RUNTIME_VERIFICATION];
EventTimeStamp_t TimeStampsBufferProcessing[NUMBER_TASKS_RUNTIME_VERIFICATION][2];
#endif

#ifdef OFFLINE_VERIFICATION
EventTimeStamp_t			QueueTimeStampsBuffer[SIZE_RUN_TIME_BUFFER_QUEUE];
EventTimeStamp_t			TimeStampInsert;
uint32_t counter_tasks_runtime_verification[NUMBER_TASKS_RUNTIME_VERIFICATION];
#endif

/**
 * \brief Function that initializes the circular queue
 *
 * \param cb - Pointer to Circular Queue structure
 * \param Buffer - Memory buffer that will be used by the queue being initialized
 * \param capacity - Size of the queue being initialized
 * \param sz - Size of elements that will be stored from the queue
 *
 */
void  cb_init(CircularBuffer_t *cb,void * Buffer, size_t capacity, size_t sz)
{
	cb->buffer =	Buffer;
	//if(cb->buffer == NULL)
	
	cb->buffer_end = cb->buffer + (capacity * sz);
	cb->capacity = capacity;
	cb->count = 0;
	cb->sz = sz;
	cb->head = cb->buffer;
	cb->tail = cb->buffer;
}

/**
 * \brief Function that inserts an element into the queue
 *
 * \param cb - Pointer to Circular Queue structure
 * \param item - Item to be added to the queue
 *
 * \return Return status of requested action
 *     @retval COMMAND_OK		Indicates that the item was successfully inserted
 *     @retval COMMAND_NOK		Indicates that the item could not be entered, the queue was full
 *     @retval COMMAND_ERROR	Event queue error, queue reinitialized
 */
int8_t cb_push_back(CircularBuffer_t *cb, const void *item)
{
	if(cb->count == cb->capacity)
	{
		return COMMAND_NOK;
	}
	
	if(cb->head > (cb->buffer_end) || cb->head < cb->buffer)
	{
		return COMMAND_ERROR;
	}
	
	
	memcpy(cb->head, item, cb->sz);
	cb->head = (char*)cb->head + cb->sz;
	if(cb->head >= cb->buffer_end)
	{
		cb->head = cb->buffer;
	}
	cb->count++;
	return COMMAND_OK;
}

/**
 * \brief Function that removes an element from the queue
 *
 * \param cb - Pointer to Circular Queue structure
 * \param item - Memory where the queue item will be copied
 * \return Return status of requested action 
 *     @retval COMMAND_OK		Indicates that the item was successfully removed
 *     @retval COMMAND_NOK		Indicates that the item could not be removed, the queue was empty
 *     @retval COMMAND_ERROR	Event queue error, queue reinitialized
 */
int8_t cb_pop_front(CircularBuffer_t *cb, void *item)
{
	if(cb->count == 0)
	{
		return COMMAND_NOK;
	}
	
	
	if(cb->tail > (cb->buffer_end) || cb->tail < cb->buffer)
	{
		return COMMAND_ERROR;
	}
	
	
	memcpy(item, cb->tail, cb->sz);
	cb->tail = (char*)cb->tail + cb->sz;
	if(cb->tail >= cb->buffer_end)
	{
		cb->tail = cb->buffer;
	}
	cb->count--;
	return COMMAND_OK;
}

/**
 * \brief Function that returns the size of the queue
 *
 * \param cb - Pointer to Circular Queue structure
 * \return Returns the queue size
 */
uint16_t cb_size(CircularBuffer_t *cb)
{
	return cb->count;
}

#ifdef ONLINE_VERIFICATION
/**
 * \brief Init RMLib
**/
void rtmlib_init(uint32_t * tasks_identifiers,uint32_t * deadlines_service,uint32_t * wcet_service)
{
	cb_init(&QueueTimeStamps,&QueueTimeStampsBuffer[0],(size_t)SIZE_RUN_TIME_BUFFER_QUEUE,(size_t)sizeof(TimeStampVeredict_t));
	
	memset(&counter_tasks_runtime_verification,0x00,sizeof(uint16_t)*NUMBER_TASKS_RUNTIME_VERIFICATION);
	
	memcpy(&Identifiers_Tasks,tasks_identifiers,NUMBER_TASKS_RUNTIME_VERIFICATION * sizeof(uint32_t));
	memcpy(&Vector_WCET_Tasks,wcet_service,NUMBER_TASKS_RUNTIME_VERIFICATION * sizeof(uint32_t));
	memcpy(&Vector_Deadline_Tasks,deadlines_service,NUMBER_TASKS_RUNTIME_VERIFICATION * sizeof(uint32_t));
}

/**
 * \brief Function that saves the task timestamp and make Veredict
 *
 * \param TimeStamp - Pointer to Circular Queue structure
 * \param Identifier_of_Task - Identifier of task
 * \param task_state - Identifier of task
 * \return Return status of requested action 
 *     @retval COMMAND_OK		Indicates that the item was successfully
 *     @retval COMMAND_NOK		Indicates that the item could not be do
 *     @retval COMMAND_ERROR	Event error
 */
int8_t timestamp_runtime(uint32_t task_identifier,uint16_t task_state)
{
	uint8_t task_index = 0;
	
	for(uint8_t number_task = 0;  number_task < NUMBER_TASKS_RUNTIME_VERIFICATION; number_task++)
	{
		if(Identifiers_Tasks[number_task] == task_identifier)
		{
			task_index = number_task;
			break;
		}
	}
	
	TimeStampsBufferProcessing[task_index][task_state].Identifier_of_Task = task_identifier;
	TimeStampsBufferProcessing[task_index][task_state].State_of_Task = task_state;
	TimeStampsBufferProcessing[task_index][task_state].TimeStamp = ReadCounterMiliSeconds();
	if(task_state == TASK_END_EXECUTION)
	{
		counter_tasks_runtime_verification[task_index]++;
	}
	TimeStampsBufferProcessing[task_index][task_state].CounterTask = counter_tasks_runtime_verification[task_index];
	
	if(task_state == TASK_END_EXECUTION)
	{
		TimeStampInsert.TimeStamp		= TimeStampsBufferProcessing[task_index][TASK_INIT_EXECUTION].TimeStamp;
		TimeStampInsert.ExecutionTime		= (TimeStampsBufferProcessing[task_index][TASK_END_EXECUTION].TimeStamp - TimeStampsBufferProcessing[task_index][TASK_INIT_EXECUTION].TimeStamp);
		TimeStampInsert.CounterTask		=  TimeStampsBufferProcessing[task_index][TASK_END_EXECUTION].CounterTask;
		TimeStampInsert.Identifier_of_Task = Identifiers_Tasks[task_index];
		
		if(TimeStampInsert.ExecutionTime <= Vector_WCET_Tasks[task_index])
		{
			TimeStampInsert.Status_of_WCET_Task = true;
		}
		else
		{
			TimeStampInsert.Status_of_WCET_Task = false;
		}
		
		if((TimeStampInsert.ExecutionTime + TimeStampInsert.TimeStamp) <= (Vector_Deadline_Tasks[task_index] * TimeStampInsert.CounterTask))
		{
			TimeStampInsert.Status_of_DeadLine_Task = true;
		}
		else
		{
			TimeStampInsert.Status_of_DeadLine_Task = false;
		}
		#ifdef EXPORT_ONLY_RTOS_ERRORS
		if(!TimeStampInsert.Status_of_DeadLine_Task || !TimeStampInsert.Status_of_WCET_Task)
		{
			return cb_push_back(&QueueTimeStamps,&TimeStampInsert);
		}
		else
		{
			return COMMAND_OK;
		}
		#else
		return cb_push_back(&QueueTimeStamps,&TimeStampInsert);
		#endif
	}
	else
	{
		return COMMAND_OK;
	}
	
}

/**
 * \brief Function to export TimeStamp, copying data to buffer in reference
 * \param buffer_rtmlib - Pointer to TimeStamp structure
 * \return Return status of requested action 
 *     @retval COMMAND_OK		Indicates that the item was successfully
 *     @retval COMMAND_NOK		Indicates that the item could not be do
 *     @retval COMMAND_ERROR	Event error
 */
int8_t rtmlib_export_data(TimeStampVeredict_t * buffer_rtmlib)
{
	return cb_pop_front(&QueueTimeStamps,buffer_rtmlib);
}

/**
 * \brief Function to export TimeStamp, in String format used  to offline verification
 * \param buffer_rtmlib - Pointer to TimeStamp structure
 * \return String of TimeStamp structure in offline verirication format
 */
const char rtmlib_export_data_string(TimeStampVeredict_t * buffer_rtmlib)
{
	#ifdef OPTIMIZE_EXPORT_DATA
	printf("IT%d-TS%d-ET%d-CT%d-SW%d-SD%d\n", buffer_rtmlib->Identifier_of_Task, buffer_rtmlib->TimeStamp, buffer_rtmlib->ExecutionTime, buffer_rtmlib->CounterTask, buffer_rtmlib->Status_of_WCET_Task, buffer_rtmlib->Status_of_DeadLine_Task);
	#else 
	printf("{\"TaskIdentifier\":%d,\"TimeStamp\":%d,\"ExecutionTime\":%d,\"CounterTask\":%d,\"Status WCET\":%d,\"Status Deadline\":%d}\n", buffer_rtmlib->Identifier_of_Task, buffer_rtmlib->TimeStamp, buffer_rtmlib->ExecutionTime, buffer_rtmlib->CounterTask, buffer_rtmlib->Status_of_WCET_Task, buffer_rtmlib->Status_of_DeadLine_Task);
	#endif
	return;
}
#endif


#ifdef OFFLINE_VERIFICATION
/**
 * \brief Init RMLib
**/
void rtmlib_init()
{
	cb_init(&QueueTimeStamps,&QueueTimeStampsBuffer[0],(size_t)SIZE_RUN_TIME_BUFFER_QUEUE,(size_t)sizeof(EventTimeStamp_t));
	
	memset(&counter_tasks_runtime_verification,0x00,sizeof(uint32_t)*NUMBER_TASKS_RUNTIME_VERIFICATION);
}

/**
 * \brief Function that saves the task timestamp
 *
 * \param TimeStamp - Pointer to Circular Queue structure
 * \param Identifier_of_Task - Identifier of task
 * \param task_state - Identifier of task
 * \return Return status of requested action 
 *     @retval COMMAND_OK		Indicates that the item was successfully
 *     @retval COMMAND_NOK		Indicates that the item could not be do
 *     @retval COMMAND_ERROR	Event error
 */
int8_t timestamp_runtime(uint32_t task_identifier,uint16_t task_state)
{
	TimeStampInsert.Identifier_of_Task = task_identifier;
	TimeStampInsert.State_of_Task = task_state;
	TimeStampInsert.TimeStamp = ReadCounterMiliSeconds();
	if(task_state == TASK_END_EXECUTION)
	{
		counter_tasks_runtime_verification[task_identifier-1]++;
	}
	TimeStampInsert.CounterTask = counter_tasks_runtime_verification[task_identifier-1];
	return cb_push_back(&QueueTimeStamps,&TimeStampInsert);
}

/**
 * \brief Function to export TimeStamp, copying data to buffer in reference
 * \param buffer_rtmlib - Pointer to TimeStamp structure
 * \return Return status of requested action 
 *     @retval COMMAND_OK		Indicates that the item was successfully
 *     @retval COMMAND_NOK		Indicates that the item could not be do
 *     @retval COMMAND_ERROR	Event error
 */
int8_t rtmlib_export_data(EventTimeStamp_t * buffer_rtmlib)
{
	return cb_pop_front(&QueueTimeStamps,buffer_rtmlib);
}

/**
 * \brief Function to export TimeStamp, in String format used  to offline verification
 * \param buffer_rtmlib - Pointer to TimeStamp structure
 * \return String of TimeStamp structure in offline verirication format
 */
const char rtmlib_export_data_string(EventTimeStamp_t * buffer_rtmlib)
{
	#ifdef OPTIMIZE_EXPORT_DATA
	printf("I%d-S%d-T%d-C%d\n",buffer_rtmlib->Identifier_of_Task,buffer_rtmlib->State_of_Task,buffer_rtmlib->TimeStamp,buffer_rtmlib->CounterTask);
	#else 
	printf("{\"TaskIdentifier\" : %d,\"TaskState\" : %d,\"TimeStamp\" : %d,\"TaskCounter\" : %d}\n",buffer_rtmlib->Identifier_of_Task,buffer_rtmlib->State_of_Task,buffer_rtmlib->TimeStamp,buffer_rtmlib->CounterTask);
	#endif
	return;
}
#endif