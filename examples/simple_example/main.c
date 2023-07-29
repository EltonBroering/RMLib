//
// Created by eltonbroering on 7/29/23.
//

#include "rmlib.h"
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>

#define TASK_IDLE_WORST_CASE    5
#define MS_COUNTS_DUMMY         1000

#define TASK_1_IDENTIFIER                   1
#define TASK_2_IDENTIFIER                   2
#define TASK_COMMUNICATION_IDENTIFIER       3

#define NUMBER_TASKS_RUNTIME_VERIFICATION   3

//#define				OFFLINE_VERIFICATION
//#define				ONLINE_VERIFICATION

#ifdef OFFLINE_VERIFICATION
EventTimeStamp_t		QueueTimeStampsBufferDumped;
#endif

#ifdef ONLINE_VERIFICATION
uint32_t Identifiers_Tasks_Simple[NUMBER_TASKS_RUNTIME_VERIFICATION] = {TASK_1_IDENTIFIER,TASK_2_IDENTIFIER,TASK_COMMUNICATION_IDENTIFIER};
uint32_t Vector_WCET_Tasks_Simple[NUMBER_TASKS_RUNTIME_VERIFICATION] = {TASK_IDLE_WORST_CASE,TASK_IDLE_WORST_CASE,TASK_IDLE_WORST_CASE};
uint32_t Vector_Deadline_Tasks_Simple[NUMBER_TASKS_RUNTIME_VERIFICATION] = {20,20,20};
uint32_t Vector_Period_Tasks_Simple[NUMBER_TASKS_RUNTIME_VERIFICATION] = {20,20,20};
TimeStampVeredict_t     QueueTimeStampsBufferDumped;
#endif

uint32_t ReadCounterMiliSeconds(void)
{
    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC_RAW, &time);
    return time.tv_nsec / 1000;
}

void function_idle_1(void)
{
    timestamp_runtime(TASK_1_IDENTIFIER,TASK_INIT_EXECUTION);
    uint32_t count_tmp = 0;
    while(count_tmp <  (TASK_IDLE_WORST_CASE*MS_COUNTS_DUMMY))
    {
		count_tmp++;
    }
    timestamp_runtime(TASK_1_IDENTIFIER,TASK_END_EXECUTION);
}

void function_idle_2(void)
{
    timestamp_runtime(TASK_2_IDENTIFIER,TASK_INIT_EXECUTION);
    uint32_t count_tmp = 0;
    while(count_tmp <  (TASK_IDLE_WORST_CASE*MS_COUNTS_DUMMY))
    {
		count_tmp++;
    }
    timestamp_runtime(TASK_2_IDENTIFIER,TASK_END_EXECUTION);
}

void task_communication()
{

    timestamp_runtime(TASK_COMMUNICATION_IDENTIFIER,TASK_INIT_EXECUTION);
    while(rmlib_export_data(&QueueTimeStampsBufferDumped) == COMMAND_OK)
    {
        rmlib_export_data_string(&QueueTimeStampsBufferDumped);
    }
    timestamp_runtime(TASK_COMMUNICATION_IDENTIFIER,TASK_END_EXECUTION);
}

void main(void)
{
#ifdef OFFLINE_VERIFICATION
    rmlib_init();
#endif
#ifdef ONLINE_VERIFICATION
    rmlib_init(&Identifiers_Tasks_Simple,&Vector_Deadline_Tasks_Simple,&Vector_Period_Tasks_Simple);
#endif


    while(true)
    {
        function_idle_1();
        function_idle_2();
        task_communication();
    }
}