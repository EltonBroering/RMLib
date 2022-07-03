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

#define TASK_CONTROLLER_STACK_SIZE				(1024/sizeof(portSTACK_TYPE))
#define TASK_CONTROLLER_STACK_PRIORITY			(tskIDLE_PRIORITY+1)
#define TASK_LED_STACK_SIZE						(1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY					(tskIDLE_PRIORITY+1)
#define TASK_COMMUNICATION_STACK_SIZE			(1024/sizeof(portSTACK_TYPE))
#define TASK_COMMUNICATION_STACK_PRIORITY		(tskIDLE_PRIORITY+1)
#define TASK_DUMMY_STACK_SIZE					(1024/sizeof(portSTACK_TYPE))
#define TASK_DUMMY_STACK_PRIORITY				(tskIDLE_PRIORITY+1)

#define TASK_IDENTIFIER_CONTROLLER			1
#define TASK_IDENTIFIER_BLINK_LED			2
#define TASK_IDENTIFIER_COMMUNICATION		3
#define TASK_IDENTIFIER_DUMMY_1				4
#define TASK_IDENTIFIER_DUMMY_2				5


extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

TimeStamp_t		QueueTimeStampsBufferDumped[SIZE_RUN_TIME_BUFFER_QUEUE];
uint32_t		task_communication_count_messages = 0;

pv_type_actuation	controller_ouput;
pv_msg_input		controller_input;

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

void rtmlib_export_data(void * buffer_rtmlib)
{
	memcpy(&QueueTimeStampsBufferDumped,buffer_rtmlib,(size_t)(SIZE_RUN_TIME_BUFFER_QUEUE*sizeof(TimeStamp_t)));
	
	task_communication_count_messages = SIZE_RUN_TIME_BUFFER_QUEUE;
}

/**
 * \brief Task Controller
 */
static void task_controller(void *pvParameters)
{
	UNUSED(pvParameters);
	
	for (;;)
	{
		timestamp_runtime(TASK_IDENTIFIER_CONTROLLER);
		controller_ouput = c_control_lqrArthur_controller(controller_input);
		vTaskDelay(10);
	}
}

/**
 * \brief Task Dummy 1
 */
static void task_dummy1(void *pvParameters)
{
	UNUSED(pvParameters);
	
	for (;;)
	{
		timestamp_runtime(TASK_IDENTIFIER_DUMMY_1);
		vTaskDelay(10);
	}
}

/**
 * \brief Task Dummy 2
 */
static void task_dummy2(void *pvParameters)
{
	UNUSED(pvParameters);
	
	for (;;)
	{
		timestamp_runtime(TASK_IDENTIFIER_DUMMY_2);
		vTaskDelay(10);
	}
}

/**
 * \brief This task, when activated, make LED blink at a fixed rate
 */
static void task_led(void *pvParameters)
{
	UNUSED(pvParameters);
	for (;;)
	{
		timestamp_runtime(TASK_IDENTIFIER_BLINK_LED);
		#if SAM4CM
		LED_Toggle(LED4);
		#else
		LED_Toggle(LED0);
		#endif
		vTaskDelay(10);
	}
}

/**
 * \brief Task Communication
 */
static void task_communication(void *pvParameters)
{
	UNUSED(pvParameters);
	
	for (;;)
	{
		timestamp_runtime(TASK_IDENTIFIER_COMMUNICATION);
		while(task_communication_count_messages)
		{
			printf("{\"TaskIdentifier\" : %d,\"TimeStamp\" : %d}\n",QueueTimeStampsBufferDumped[(task_communication_count_messages-1)].Identifier_of_Task,QueueTimeStampsBufferDumped[(task_communication_count_messages-1)].TimeStamp);
			task_communication_count_messages--;
		}
		vTaskDelay(10);
	}
}


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
	
	//Start RunTime Verification Lib
	rtmlib_init();

	/* Output demo information. */
	printf("-- Freertos Example --\n\r");
	printf("-- %s\n\r", BOARD_NAME);
	printf("-- Compiled: %s %s --\n\r", __DATE__, __TIME__);


	/* Create task to controller */
	if (xTaskCreate(task_controller, "Controller", TASK_CONTROLLER_STACK_SIZE, NULL,
			TASK_CONTROLLER_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Controller task\r\n");
	}
	
	/* Create task communication */
	if (xTaskCreate(task_communication, "Communication", TASK_COMMUNICATION_STACK_SIZE, NULL,
	TASK_COMMUNICATION_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create coomunication task\r\n");
	}

	/* Create task to make led blink */
	if (xTaskCreate(task_led, "Led", TASK_LED_STACK_SIZE, NULL,
			TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create led task\r\n");
	}
	
	/* Create task to make Dummy 1 */
	if (xTaskCreate(task_dummy1, "Dummy 1", TASK_DUMMY_STACK_SIZE, NULL,
	TASK_DUMMY_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create dummy task\r\n");
	}
	
	/* Create task to make Dummy 2 */
	if (xTaskCreate(task_dummy2, "Dummy 2", TASK_DUMMY_STACK_SIZE, NULL,
	TASK_DUMMY_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create dummy task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
