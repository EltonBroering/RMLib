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
TimeStamp_t			QueueTimeStampsBuffer[SIZE_RUN_TIME_BUFFER_QUEUE];
TimeStamp_t			TimeStampInsert;

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

/**
 * \brief Function that dumps the system timestamp memory, to export the data for analysis
 */
void dump_buffer_timestamp()
{
	rtmlib_export_data(&QueueTimeStampsBuffer);
	
	rtmlib_init();
}

/**
 * \brief Function that saves the task timestamp
 *
 * \param TimeStamp - Pointer to Circular Queue structure
 * \param Identifier_of_Task - Identifier of task
 * \return Return status of requested action 
 *     @retval COMMAND_OK		Indicates that the item was successfully
 *     @retval COMMAND_NOK		Indicates that the item could not be do
 *     @retval COMMAND_ERROR	Event error
 */
int8_t timestamp_runtime(uint32_t Identifier_of_Task)
{
	TimeStampInsert.Identifier_of_Task = Identifier_of_Task;
	TimeStampInsert.TimeStamp = ReadCounterHundredsMicroSeconds();
	int8_t return_function = cb_push_back(&QueueTimeStamps,&TimeStampInsert);
	return return_function;
}

int8_t rtmlib_export_data(TimeStamp_t * buffer_rtmlib)
{
	int8_t return_function = cb_pop_front(&QueueTimeStamps,buffer_rtmlib);
	return return_function;
}

/**
 * \brief Init RMLib
 *
**/
void rtmlib_init()
{
	cb_init(&QueueTimeStamps,&QueueTimeStampsBuffer[0],(size_t)SIZE_RUN_TIME_BUFFER_QUEUE,(size_t)sizeof(TimeStamp_t));
}