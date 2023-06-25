/**
  ******************************************************************************
  * @file    modules/control/c_control_LQR.h
  * @author  Iuro Nascimento
  * @version V1.0.0
  * @date    28-July-2016
  * @brief   Controlador LQR baseado em c_rc_LQR_control.
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef C_CONTROL_LQR_H
#define C_CONTROL_LQR_H

#define ARM_MATH_CM4

#include "Includes.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
void c_control_lqr_init();

//LQR attitude and height(AH) controller. Height control depends on global variable manual_height_control.
void c_control_lqr_controller(pv_msg_input * inputData, pv_type_actuation * output_data);

#ifdef __cplusplus
}
#endif

#endif 
