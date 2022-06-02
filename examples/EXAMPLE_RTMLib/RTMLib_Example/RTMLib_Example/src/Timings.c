/*                          ***********************
 *************************** MÓDULO: TEMPORIZAÇÕES ***************************
 *                          ***********************
 */
/** 
 * \file Timings.c
 * \headerfile Timings.h
 * 
 *****************************************************************************/

#include "Timings.h"

/**
  * \brief Returns the value of the infinite hundredths of microsecond counter.
  *
  * \return Hundredths of microsecond counter value.
  */
uint32_t ReadCounterHundredsMicroSeconds(void)
{
	return xTaskGetTickCount();
}




/*****************************************************************************
 *                                                                           *
 *                                                                           *
 *****************************************************************************/
