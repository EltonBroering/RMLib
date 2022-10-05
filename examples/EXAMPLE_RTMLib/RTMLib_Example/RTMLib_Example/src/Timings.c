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

#include "tc.h"

void Timer_init()
{
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	/* Configure PMC */
	pmc_enable_periph_clk(ID_TC0);
	/** Configure TC for a System Timer Count Scale frequency and trigger on RC compare. */
	tc_find_mck_divisor(SYSTEM_TIMER_COUNT_SCALE, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	tc_init(TC0, 0, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC0, 0, (ul_sysclk / ul_div) / SYSTEM_TIMER_COUNT_SCALE);
	/* Configure and enable interrupt on RC compare */
	NVIC_EnableIRQ((IRQn_Type) ID_TC0);
	tc_enable_interrupt(TC0, 0, TC_IER_CPCS);
	/** Start the counter if LED1 is enabled. */
	tc_start(TC0, 0);
}

volatile uint32_t g_ul_ms_ticks = 0;

void TC0_Handler(void)
{
	volatile uint32_t ul_dummy;
	/* Clear status bit to acknowledge interrupt */
	ul_dummy = tc_get_status(TC0, 0);
	g_ul_ms_ticks++;
}

/**
  * \brief Returns the value of the infinite Miliseconds counter.
  * \return Miliseconds counter value.
  */
uint32_t ReadCounterMiliSeconds(void)
{
	return g_ul_ms_ticks;
}




/*****************************************************************************
 *                                                                           *
 *                                                                           *
 *****************************************************************************/
