/*
 * Includes.h
 *
 * Created: 5/14/2022 3:23:04 PM
 *  Author: Perimeter
 */ 


#ifndef INCLUDES_H_
#define INCLUDES_H_

#include <asf.h>
#include <string.h>

#define PACKED SHORTENUM		// Utilizado nas struct para formatar os dados de 8 em 8 bits (usado em tempo de compilação - compiler.h)


#include "arm_math.h"

#include "rtmlib.h"

#include "Timings.h"

#include "VANT/pv_typedefs.h"

#include "VANT/c_control_lqrArthur.h"

#endif /* INCLUDES_H_ */