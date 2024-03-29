/**
 * \file
 *
 * \brief FreeRTOS configuration
 *
 * Copyright (c) 2012-2018 Microchip Technology Inc. and its subsidiaries.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Subject to your compliance with these terms, you may use Microchip
 * software and any derivatives exclusively with Microchip products.
 * It is your responsibility to comply with third party license terms applicable
 * to your use of third party software (including open source software) that
 * may accompany Microchip software.
 *
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE,
 * INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY,
 * AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL MICROCHIP BE
 * LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR CONSEQUENTIAL
 * LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO THE
 * SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE
 * POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE FULLEST EXTENT
 * ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN ANY WAY
 * RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
 * THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * \asf_license_stop
 *
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
 
/**
 * \mainpage FreeRTOS Real Time Kernel example
 *
 * \section Purpose
 *
 * The FreeRTOS example will help users how to use FreeRTOS in SAM boards.
 * This basic application shows how to create task and get information of
 * created task.
 *
 * \section Requirements
 *
 * This package can be used with SAM boards.
 *
 * \section Description
 *
 * The demonstration program create two task, one is make LED on the board
 * blink at a fixed rate, and another is monitor status of task.
 *
 * \section Usage
 *
 * -# Build the program and download it inside the evaluation board. Please
 *    refer to the
 *    <a href="http://www.microchip.com/dyn/resources/prod_documents/doc6224.pdf">
 *    SAM-BA User Guide</a>, the
 *    <a href="http://ww1.microchip.com/downloads/en/appnotes/doc6310.pdf">
 *    GNU-Based Software Development</a>
 *    application note or to the
 *    <a href="ftp://ftp.iar.se/WWWfiles/arm/Guides/EWARM_UserGuide.ENU.pdf">
 *    IAR EWARM User Guide</a>,
 *    depending on your chosen solution.
 * -# On the computer, open and configure a terminal application
 *    (e.g. HyperTerminal on Microsoft Windows) with these settings:
 *   - 115200 bauds
 *   - 8 bits of data
 *   - No parity
 *   - 1 stop bit
 *   - No flow control
 * -# Start the application.
 * -# LED should start blinking on the board. In the terminal window, the
 *    following text should appear (values depend on the board and chip used):
 *    \code
	-- Freertos Example xxx --
	-- xxxxxx-xx
	-- Compiled: xxx xx xxxx xx:xx:xx --
\endcode
 *
 */

#include <asf.h>
#include "conf_board.h"
#include "Includes.h"

#define TASK_COMMUNICATION_STACK_SIZE			(configMINIMAL_STACK_SIZE)
#define TASK_COMMUNICATION_PRIORITY				(tskIDLE_PRIORITY+1)
#define TASK_LED_HLC_STACK_SIZE					(configMINIMAL_STACK_SIZE)
#define TASK_LED_HLC_PRIORITY					(tskIDLE_PRIORITY+3)
#define TASK_CONTROLLER_STACK_SIZE				(configMINIMAL_STACK_SIZE * 4)
#define TASK_CONTROLLER_PRIORITY				(tskIDLE_PRIORITY+2)
#define TASK_DUMMY_SENSING_STACK_SIZE			(configMINIMAL_STACK_SIZE)
#define TASK_DUMMY_SENSING_PRIORITY				(tskIDLE_PRIORITY+4)
#define TASK_DUMMY_ACTUATION_STACK_SIZE			(configMINIMAL_STACK_SIZE)
#define TASK_DUMMY_ACTUATION_PRIORITY			(tskIDLE_PRIORITY+5)

xTaskHandle TaskHandle_Communication;
xTaskHandle TaskHandle_LedHLC;
xTaskHandle TaskHandle_Controller;
xTaskHandle TaskHandle_DummySensing;
xTaskHandle TaskHandle_DummyActuation;

#define TASK_IDENTIFIER_DUMMY_ACTUATION			1
#define TASK_IDENTIFIER_DUMMY_SENSING			2
#define TASK_IDENTIFIER_CONTROLLER				3
#define TASK_IDENTIFIER_BLINK_LED_HLC			4
#define TASK_IDENTIFIER_COMMUNICATION			5

#define NUMBER_TASKS							5
#define NUMBER_TASKS_RUNTIME_VERIFICATION		NUMBER_TASKS

uint32_t counter_tasks_runtime[NUMBER_TASKS];

#define TASK_CONTROLLER_WORST_CASE				4
#define TASK_BLINK_LED_HLC_WORST_CASE			4
#define TASK_COMMUNICATION_WORST_CASE			135
#define TASK_DUMMY_ACTUATION_WORST_CASE			2
#define TASK_DUMMY_SENSING_WORST_CASE			3

#define TASK_CONTROLLER_PERIOD					12
#define TASK_BLINK_LED_HLC_PERIOD				120
#define TASK_COMMUNICATION_PERIOD				600
#define TASK_DUMMY_ACTUATION_PERIOD				12
#define TASK_DUMMY_SENSING_PERIOD				12



extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

#ifdef OFFLINE_VERIFICATION
EventTimeStamp_t		QueueTimeStampsBufferDumped;
#endif

#ifdef ONLINE_VERIFICATION
TimeStampVeredict_t		QueueTimeStampsBufferDumped;
uint32_t Identifiers_Tasks[NUMBER_TASKS];
uint32_t Vector_WCET_Tasks[NUMBER_TASKS];
uint32_t Vector_Deadline_Tasks[NUMBER_TASKS];
uint32_t Vector_Period_Tasks[NUMBER_TASKS];

#ifdef EXPORT_DUMP_STATUS_TASKS
#define size_buffer_export 256
char szList[size_buffer_export];
char str_export_aux[size_buffer_export];
#endif
#endif

#define TASK_COMMUNCATION_PERIODIC

//#define ASYNCHRONOUS_TASK

#ifdef ASYNCHRONOUS_TASK
#define TASK_ASYNCHRONOUS_STACK_SIZE			(configMINIMAL_STACK_SIZE)
#define TASK_ASYNCHRONOUS_PRIORITY				(tskIDLE_PRIORITY+6)
#define TASK_IDENTIFIER_ASYNCHRONOUS			6
#define TASK_ASYNCHRONOUS_WORST_CASE			8
#define TASK_ASYNCHRONOUS_DEADLINE				15
#define TASK_ASYNCHRONOUS_PERIOD				0
#define NUMBER_TASKS							6
#define NUMBER_TASKS_RUNTIME_VERIFICATION		NUMBER_TASKS
xTaskHandle TaskHandle_Asynchronous;
#endif

pv_type_actuation	controller_ouput;
pv_msg_input		controller_input;

/** LED blink time 300ms */
#define BLINK_PERIOD						10

#define MS_COUNTS_DUMMY						6000

#define MS_INIT_USB_MILISECOND				500

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;)
	{
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT( ( volatile void * ) NULL );
}

#ifdef ONLINE_VERIFICATION
void init_buffer_tasks_runtime_verification_online()
{
	for(uint8_t count_task = 0; count_task < NUMBER_TASKS; count_task++)
	{
		switch(count_task+1)
		{
			case TASK_IDENTIFIER_DUMMY_ACTUATION:
				Identifiers_Tasks[count_task]		= TASK_IDENTIFIER_DUMMY_ACTUATION;
				Vector_WCET_Tasks[count_task]		= TASK_DUMMY_ACTUATION_WORST_CASE;
				Vector_Deadline_Tasks[count_task]	= TASK_DUMMY_ACTUATION_PERIOD;
				Vector_Period_Tasks[count_task]		= TASK_DUMMY_ACTUATION_PERIOD;
				break;
			
			case TASK_IDENTIFIER_DUMMY_SENSING:
				Identifiers_Tasks[count_task]		= TASK_IDENTIFIER_DUMMY_SENSING;
				Vector_WCET_Tasks[count_task]		= TASK_DUMMY_SENSING_WORST_CASE;
				Vector_Deadline_Tasks[count_task]	= TASK_DUMMY_SENSING_PERIOD;
				Vector_Period_Tasks[count_task]		= TASK_DUMMY_SENSING_PERIOD;
				break;
			
			case TASK_IDENTIFIER_CONTROLLER:
				Identifiers_Tasks[count_task]		= TASK_IDENTIFIER_CONTROLLER;
				Vector_WCET_Tasks[count_task]		= TASK_CONTROLLER_WORST_CASE;
				Vector_Deadline_Tasks[count_task]	= TASK_CONTROLLER_PERIOD;
				Vector_Period_Tasks[count_task]		= TASK_CONTROLLER_PERIOD;
				break;
				
			case TASK_IDENTIFIER_BLINK_LED_HLC:
				Identifiers_Tasks[count_task]		= TASK_IDENTIFIER_BLINK_LED_HLC;
				Vector_WCET_Tasks[count_task]		= TASK_BLINK_LED_HLC_WORST_CASE;
				Vector_Deadline_Tasks[count_task]	= TASK_BLINK_LED_HLC_PERIOD;
				Vector_Period_Tasks[count_task]		= TASK_BLINK_LED_HLC_PERIOD;
				break;
				
			case TASK_IDENTIFIER_COMMUNICATION:
				Identifiers_Tasks[count_task]		= TASK_IDENTIFIER_COMMUNICATION;
				Vector_WCET_Tasks[count_task]		= TASK_COMMUNICATION_WORST_CASE;
				Vector_Deadline_Tasks[count_task]	= TASK_COMMUNICATION_PERIOD;
				Vector_Period_Tasks[count_task]		= TASK_COMMUNICATION_PERIOD;
				break;
				
			#ifdef ASYNCHRONOUS_TASK
			case TASK_IDENTIFIER_ASYNCHRONOUS:
				Identifiers_Tasks[count_task]		= TASK_IDENTIFIER_ASYNCHRONOUS;
				Vector_WCET_Tasks[count_task]		= TASK_ASYNCHRONOUS_WORST_CASE;
				Vector_Deadline_Tasks[count_task]	= TASK_ASYNCHRONOUS_DEADLINE;
				Vector_Period_Tasks[count_task]		= TASK_ASYNCHRONOUS_PERIOD;
				break;
			#endif
				
			default:
				break;
		}
	}
}
#endif

/**
 *  \brief Configure LED.
 */
static void configure_led(void)
{
	/* Configure PIOs for LED. */
	pio_configure(LED_PIO, LED_TYPE, LED_MASK, LED_ATTR);
}

#ifdef EXPORT_DUMP_STATUS_TASKS
/*
	Task used to dump status tasks - For Reference for format export see https://www.freertos.org/a00021.html#vTaskList
*/
void DumpStatusTasks()
{
	vTaskList(szList);
}
#endif

/**
 * \brief Task Controller
 */
static void task_controller(void *pvParameters)
{
	UNUSED(pvParameters);
	
	c_control_lqr_init();
	
	while(true)
	{
		vTaskSuspendAll();
		timestamp_runtime(TASK_IDENTIFIER_CONTROLLER,TASK_INIT_EXECUTION);
		c_control_lqr_controller(&controller_input,&controller_ouput);
		uint32_t count_tmp = 0;
		while(count_tmp <  (TASK_CONTROLLER_WORST_CASE*MS_COUNTS_DUMMY))
		{
			count_tmp++;
		}
		timestamp_runtime(TASK_IDENTIFIER_CONTROLLER,TASK_END_EXECUTION);
		counter_tasks_runtime[TASK_IDENTIFIER_CONTROLLER-1]++;
		uint32_t MileSecondsTask = ReadCounterMiliSeconds();
		uint32_t ValueTaskDelay;
		
		if((TASK_CONTROLLER_PERIOD*counter_tasks_runtime[TASK_IDENTIFIER_CONTROLLER-1]) > MileSecondsTask)
		{
			ValueTaskDelay = (TASK_CONTROLLER_PERIOD*counter_tasks_runtime[TASK_IDENTIFIER_CONTROLLER-1] - MileSecondsTask);
			vTaskDelay(ValueTaskDelay);
		}
		else
		{
			xTaskResumeAll();
		}
	}
}

/**
 * \brief Task Dummy Actuation
 */
static void task_dummy_actuation(void *pvParameters)
{
	UNUSED(pvParameters);
	
	while(true)
	{
		vTaskSuspendAll();
		timestamp_runtime(TASK_IDENTIFIER_DUMMY_ACTUATION,TASK_INIT_EXECUTION);
		uint32_t count_tmp = 0;
		while(count_tmp <  (TASK_DUMMY_ACTUATION_WORST_CASE*MS_COUNTS_DUMMY))
		{
			count_tmp++;
		}
		timestamp_runtime(TASK_IDENTIFIER_DUMMY_ACTUATION,TASK_END_EXECUTION);
		counter_tasks_runtime[TASK_IDENTIFIER_DUMMY_ACTUATION-1]++;
		uint32_t MileSecondsTask = ReadCounterMiliSeconds();
		uint32_t ValueTaskDelay;
		
		if(TASK_DUMMY_ACTUATION_PERIOD*counter_tasks_runtime[TASK_IDENTIFIER_DUMMY_ACTUATION-1] > MileSecondsTask)
		{
			ValueTaskDelay = (TASK_DUMMY_ACTUATION_PERIOD*counter_tasks_runtime[TASK_IDENTIFIER_DUMMY_ACTUATION-1] - MileSecondsTask);
		}
		else
		{
			ValueTaskDelay = 0;
		}
		vTaskDelay(ValueTaskDelay);
		xTaskResumeAll();
		
	}
}

/**
 * \brief Task Dummy Sensing
 */
static void task_dummy_sensing(void *pvParameters)
{
	UNUSED(pvParameters);
	
	while(true)
	{
		vTaskSuspendAll();
		timestamp_runtime(TASK_IDENTIFIER_DUMMY_SENSING,TASK_INIT_EXECUTION);
		uint32_t count_tmp = 0;
		while(count_tmp <  (TASK_DUMMY_SENSING_WORST_CASE*MS_COUNTS_DUMMY))
		{
			count_tmp++;
		}
		timestamp_runtime(TASK_IDENTIFIER_DUMMY_SENSING,TASK_END_EXECUTION);		
		counter_tasks_runtime[TASK_IDENTIFIER_DUMMY_SENSING-1]++;
		uint32_t MileSecondsTask = ReadCounterMiliSeconds();
		uint32_t ValueTaskDelay;
		
		if(TASK_DUMMY_SENSING_PERIOD*counter_tasks_runtime[TASK_IDENTIFIER_DUMMY_SENSING-1] > MileSecondsTask)
		{
			ValueTaskDelay = (TASK_DUMMY_SENSING_PERIOD*counter_tasks_runtime[TASK_IDENTIFIER_DUMMY_SENSING-1] - MileSecondsTask);
		}
		else
		{
			ValueTaskDelay = 0;
		}
		vTaskDelay(ValueTaskDelay);
		xTaskResumeAll();
	}
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */
static void task_led_hlc(void *pvParameters)
{
	UNUSED(pvParameters);
	
	uint32_t ticks_toggle_led;
	
	configure_led();
	
	while(true)
	{
		vTaskSuspendAll();
		timestamp_runtime(TASK_IDENTIFIER_BLINK_LED_HLC,TASK_INIT_EXECUTION);
		/* Toggle LED at the given period. */
		if((ReadCounterMiliSeconds() - ticks_toggle_led) > BLINK_PERIOD)
		{
			ticks_toggle_led = ReadCounterMiliSeconds();
			LED_Toggle(LED0);
		}
		uint32_t count_tmp = 0;
		while(count_tmp <  (TASK_BLINK_LED_HLC_WORST_CASE*MS_COUNTS_DUMMY))
		{
			count_tmp++;
		}
		timestamp_runtime(TASK_IDENTIFIER_BLINK_LED_HLC,TASK_END_EXECUTION);
		counter_tasks_runtime[TASK_IDENTIFIER_BLINK_LED_HLC-1]++;
		uint32_t MileSecondsTask = ReadCounterMiliSeconds();
		uint32_t ValueTaskDelay;
		
		if(TASK_BLINK_LED_HLC_PERIOD*counter_tasks_runtime[TASK_IDENTIFIER_BLINK_LED_HLC-1] > MileSecondsTask)
		{
			ValueTaskDelay = ((TASK_BLINK_LED_HLC_PERIOD*counter_tasks_runtime[TASK_IDENTIFIER_BLINK_LED_HLC-1]) - MileSecondsTask);
		}
		else
		{
			ValueTaskDelay = 0;
		}
		vTaskDelay(ValueTaskDelay);
		xTaskResumeAll();
	}
}

/**
 * \brief Task Communication
 */
static void task_communication(void *pvParameters)
{
	UNUSED(pvParameters);
	
	while(true)
	{
		if(ReadCounterMiliSeconds() < MS_INIT_USB_MILISECOND)
		{
			continue;
		}
		
		timestamp_runtime(TASK_IDENTIFIER_COMMUNICATION,TASK_INIT_EXECUTION);
		
		while(rmlib_export_data(&QueueTimeStampsBufferDumped) == COMMAND_OK)
		{
			rmlib_export_data_string(&QueueTimeStampsBufferDumped);
		}
		
		#ifdef EXPORT_DUMP_STATUS_TASKS
		char * token = strtok(szList, "\n");
		while(token != NULL)
		{
			printf("%s\n", token); //printing each token
			token = strtok(NULL, "\n");
		}
		memset(&szList[0],0,size_buffer_export);
		#endif
		
		timestamp_runtime(TASK_IDENTIFIER_COMMUNICATION,TASK_END_EXECUTION);
		counter_tasks_runtime[TASK_IDENTIFIER_COMMUNICATION-1]++;
		
		#ifdef TASK_COMMUNCATION_PERIODIC
		uint32_t MileSecondsTask = ReadCounterMiliSeconds();
		uint32_t ValueTaskDelay;
		
		if(TASK_COMMUNICATION_PERIOD*counter_tasks_runtime[TASK_IDENTIFIER_COMMUNICATION-1] > MileSecondsTask)
		{
			ValueTaskDelay = ((TASK_COMMUNICATION_PERIOD*counter_tasks_runtime[TASK_IDENTIFIER_COMMUNICATION-1]) - MileSecondsTask);
		}
		else
		{
			ValueTaskDelay = 0;
		}
		vTaskDelay(ValueTaskDelay);
		#else
		vTaskDelay(10);
		#endif
	}
}

#ifdef ASYNCHRONOUS_TASK
/**
 * \brief Task Asyncronous
 */
static void task_asynchronous(void *pvParameters)
{
	UNUSED(pvParameters);
	
	vTaskSuspend(NULL);
	
	while(true)
	{
		vTaskSuspendAll();
		
		uint32_t count_tmp = 0;
		while(count_tmp <  (TASK_ASYNCHRONOUS_WORST_CASE*MS_COUNTS_DUMMY))
		{
			count_tmp++;
		}
		timestamp_runtime(TASK_IDENTIFIER_ASYNCHRONOUS,TASK_END_EXECUTION);
		xTaskResumeAll();	
		vTaskSuspend(NULL);	
	}
}

uint8_t initial_transition = false;

/**
 *  \brief Handler for Button 1 rising edge interrupt.
 *
 *  Set button1 event flag (g_button_event).
 */
static void button_handler(uint32_t id, uint32_t mask)
{
	if ((PIN_SW0_ID == id) && (PIN_SW0_MASK == mask))
	{
		if(initial_transition)
		{
			vTaskResume(TaskHandle_Asynchronous);
			timestamp_runtime(TASK_IDENTIFIER_ASYNCHRONOUS,TASK_INIT_EXECUTION);
		}
		initial_transition = true;
	}
}

/**
 *  \brief Configure the push button.
 *
 *  Configure the PIOs as inputs and generate corresponding interrupt when
 *  pressed or released.
 */
static void configure_button(void)
{
	/* Configure PIO clock. */
	pmc_enable_periph_clk(PIN_SW0_ID);

	/* Adjust pio debounce filter parameters, uses 10 Hz filter. */
	pio_set_debounce_filter(PIN_SW0_PIO, PIN_SW0_MASK, 10);

	/* Initialize pios interrupt handlers, see PIO definition in board.h. */
	pio_handler_set(PIN_SW0_PIO, PIN_SW0_ID, PIN_SW0_MASK,
		PIN_SW0_ATTR, button_handler);

	/* Enable PIO controller IRQs. */
	NVIC_EnableIRQ((IRQn_Type)PIN_SW0_ID);

	/* Enable PIO line interrupts. */
	pio_enable_interrupt(PIN_SW0_PIO, PIN_SW0_MASK);
}
#endif

/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	
	/* Call a local utility routine to initialize C-Library Standard I/O over
	* a USB CDC protocol. Tunable parameters in a conf_usb.h file must be
	* supplied to configure the USB device correctly.
	*/
	stdio_usb_init();
	
	Timer_init();

	#ifdef ASYNCHRONOUS_TASK
	configure_button();
	#endif
	
	//Start RunTime Verification Lib
	#ifdef ONLINE_VERIFICATION
	init_buffer_tasks_runtime_verification_online();
	rmlib_init(&Identifiers_Tasks,&Vector_Deadline_Tasks,&Vector_Period_Tasks,&Vector_WCET_Tasks);
	#endif
	#ifdef OFFLINE_VERIFICATION
	rmlib_init();
	#endif

	/* Output demo information. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);
	
	/* Create task to controller */
	if(xTaskCreate(task_controller, "Controller", TASK_CONTROLLER_STACK_SIZE, NULL, TASK_CONTROLLER_PRIORITY,&TaskHandle_Controller) != pdPASS)
	{
		printf("Failed to create Controller task\r\n");
	}

	/* Create task communication */
	if(xTaskCreate(task_communication, "Communication", TASK_COMMUNICATION_STACK_SIZE, NULL, TASK_COMMUNICATION_PRIORITY,&TaskHandle_Communication) != pdPASS)
	{
		printf("Failed to create coomunication task\r\n");
	}

	/* Create task to make led blink */
	if(xTaskCreate(task_led_hlc, "Led", TASK_LED_HLC_STACK_SIZE, NULL, TASK_LED_HLC_PRIORITY, &TaskHandle_LedHLC) != pdPASS)
	{
		printf("Failed to create led task\r\n");
	}
	
	/* Create task to make Dummy Sensing */
	if(xTaskCreate(task_dummy_sensing, "Dummy Sensing", TASK_DUMMY_SENSING_STACK_SIZE, NULL, TASK_DUMMY_SENSING_PRIORITY, &TaskHandle_DummySensing) != pdPASS)
	{
		printf("Failed to create dummy task\r\n");
	}
	
	/* Create task to make Dummy Actuation */
	if(xTaskCreate(task_dummy_actuation, "Dummy Actuation", TASK_DUMMY_ACTUATION_STACK_SIZE, NULL,TASK_DUMMY_ACTUATION_PRIORITY, &TaskHandle_DummyActuation) != pdPASS)
	{
		printf("Failed to create dummy task\r\n");
	}
	
	#ifdef ASYNCHRONOUS_TASK
	/* Create task to make Asynchronous Task */
	if(xTaskCreate(task_asynchronous, "Asynchronous", TASK_ASYNCHRONOUS_STACK_SIZE, NULL,TASK_ASYNCHRONOUS_PRIORITY, &TaskHandle_Asynchronous) != pdPASS)
	{
		printf("Failed to create asynchronous task\r\n");
	}
	#endif
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
