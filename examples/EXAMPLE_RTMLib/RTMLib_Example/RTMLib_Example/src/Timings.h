/*
 * Timings.h
 */ 

#ifndef TIMINGS_H_
#define TIMINGS_H_

#include "Includes.h"


#define SYSTEM_TIMER_COUNT_SCALE		1000

/*                          ***********************
 *************************** Module: TIMINGS (HEADER) ******************
 *                          ***********************
 */
/** 
 * \file Timings.c
 * \headerfile Timings.h
 *
 *****************************************************************************/

void Timer_init();

uint32_t ReadCounterMiliSeconds(void);



/*****************************************************************************
 *																			 *
 *																			 *
 *****************************************************************************/

#endif /* TEMPORIZACOES_H_ */