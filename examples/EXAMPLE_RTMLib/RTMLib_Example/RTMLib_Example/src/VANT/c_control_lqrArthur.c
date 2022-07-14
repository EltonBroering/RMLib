/**
  ******************************************************************************
  * @file    modules/control/c_control_lqrArthur.c
  * @author  Iuro Nascimento
  * @version V1.0.0
  * @date    28-July-2016
  * @brief   Implementa controlador lqrArthur baseado em c_rc_lqrArthur_control de Rodrigo Donadel.
  ******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "c_control_lqrArthur.h"


//---------------------------------------------------------------------------------------------

/* Exported functions definitions --------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static pv_type_stability_error last_error;

/* DlqrArthur */
static float32_t K_f32[4][20] = {{-0.000509474023994 ,  1.381002006541810 ,  2.044930990723325 , -4.098388643419657 ,  0.002544968427177 ,  0.065243786421189, -0.011997162152724 ,  0.012231188237446 , -0.000191109567030 ,  0.977046060078436  , 2.067519443836474 , -1.069820095142832,
0.003981280957280 ,  0.048837474399233 , -0.000157117358773  , 0.000160465562387 , -0.000324184653490 ,  0.932396154093819,
0.987708499654750 ,  0.029590812630684},

{0.000485844262540 , -1.379777079391250 ,  2.046778453514912 ,  4.095526259295323 ,  0.006483505671600 , -0.065184427882726,
0.011988462007403 , -0.012218371367099 , 0.000647666101473 , -0.976219155907380 ,  2.069372953111319  , 1.069647999496566,
0.005073107886213 , -0.048792319825525 ,  0.000156410506717 , -0.000160891309965 ,  0.000193352263628 , -0.931559941080183,
0.988594878555003 , -0.029564069576714},

{0.147511637570957 , -0.044700625197703  , 0.000027359219549  , 0.100038920576059  , 0.186220549413359 ,  0.044874312445416,
0.218427852703492 ,  0.036876304589717 ,  0.070402232740872 , -0.028416542202667 ,  0.000007473706141 ,  0.017792742949581,
0.025919243351085 ,  0.030976534992245 ,  0.007155474366137 ,  0.000723596619702 ,  0.140517760855087 , -0.032497295109449,
0.000009086030883 ,  0.021343249034188},

{0.147480373299421 ,  0.044731365984124 ,  0.000026937597506 , -0.099883806663005 ,  0.186155931716280 , -0.044878136021767,
0.036878957916608 ,  0.218443336829840 ,  0.070383903601564 ,  0.028402976825487  , 0.000007312089313 , -0.017740572220967,
0.025908596148404,  -0.030981560936545  , 0.000723658016145  , 0.007155720063258  , 0.140489293081162 ,  0.032562443693621,
0.000009316184096 , -0.021344301330953}
};

static float32_t equilibrium_point_f32[20] = {2.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static float32_t equilibrium_control_f32[4] = {10.2751, 10.2799, 0.0, 0.0};
static float32_t state_vector_f32[20] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static float32_t error_state_vector_f32[20] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static float32_t control_output_f32[20] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
static float32_t delta_control_f32[20] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

static arm_matrix_instance_f32 equilibrium_control;
static arm_matrix_instance_f32 K;

arm_matrix_instance_f32 error_state_vector, delta_control, control_output;

pv_type_datapr_attitude attitude, attitude_reference;
pv_type_datapr_position position, position_reference;
pv_type_datapr_servos servos;


double x_atual, y_atual, z_atual, yaw_atual;

float32_t xint, x_ant, yint, y_ant, zint, z_ant, yawint, yaw_ant, T;

arm_matrix_instance_f32 state_vector, equilibrium_point;
/* Private function prototypes -----------------------------------------------*/
/* static arm_matrix_instance_f32 c_control_lqrArthur_calcErrorStateVector(pv_type_datapr_attitude attitude, pv_type_datapr_attitude attitude_reference, pv_type_datapr_position position, pv_type_datapr_position position_reference); */

/* Private functions ---------------------------------------------------------*/
void c_control_lqrArthur_calcErrorStateVector(pv_msg_input * inputData)
{	
	
	attitude = inputData->attitude;
	attitude_reference = inputData->attitude_reference;
	position = inputData->position;
	position_reference = inputData->position_reference;
	servos = inputData->servosOutput.servo;
	

	xint = 0.0; x_ant = 0.0;
	yint = 0.0; y_ant = 0.0;
	zint = 0.0; z_ant = 0.0;
	yawint = 0.0; yaw_ant = 0.0;
	
	T = 0.012;

	// Integrador Trapezoidal
	x_atual = (double)position.x - position_reference.x;
	
	xint = (double)xint + (T/2.0)*(x_atual + x_ant);
	x_ant = x_atual;
	
	y_atual = (double)position.y - position_reference.y;
	yint = (double)yint + (T/2.0)*(y_atual + y_ant);
	y_ant = y_atual;
	
	
	z_atual = (double)position.z - position_reference.z;
	zint = (double)zint + (T/2.0)*(z_atual + z_ant);
	z_ant = z_atual;
	
	yaw_atual = (double)attitude.yaw;
	yawint = (double)yawint + (T/2.0)*(yaw_atual + yaw_ant);
	yaw_ant = yaw_atual;
	
	//State Vector
	state_vector_f32[0] = (float32_t) position.x;
	state_vector_f32[1] = (float32_t) position.y;
	state_vector_f32[2] = (float32_t) position.z;
	state_vector_f32[3] = (float32_t) attitude.roll;
	state_vector_f32[4] = (float32_t) attitude.pitch;
	state_vector_f32[5] = (float32_t) attitude.yaw;
	state_vector_f32[6] = (float32_t) servos.alphar;
	state_vector_f32[7] = (float32_t) servos.alphal;
	state_vector_f32[8] = (float32_t) position.dotX;
	state_vector_f32[9] = (float32_t) position.dotY;
	state_vector_f32[10] = (float32_t) position.dotZ;
	state_vector_f32[11] = (float32_t) attitude.dotRoll;
	state_vector_f32[12] = (float32_t) attitude.dotPitch;
	state_vector_f32[13] = (float32_t) attitude.dotYaw;
	state_vector_f32[14] = (float32_t) servos.dotAlphar;
	state_vector_f32[15] = (float32_t) servos.dotAlphal;
	state_vector_f32[16] = (float32_t) xint;
	state_vector_f32[17] = (float32_t) yint;
	state_vector_f32[18] = (float32_t) zint;
	state_vector_f32[19] = (float32_t) yawint;
	//state_vector_f32[16]=0;
	//state_vector_f32[17]=0;
	//state_vector_f32[18]=0;
	//state_vector_f32[19]=0;

	//Updates the height equilibrium point according to the reference
	equilibrium_point_f32[0] = (float32_t) position_reference.x;
	equilibrium_point_f32[1] = (float32_t) position_reference.y;
	equilibrium_point_f32[2] = (float32_t) position_reference.z;
	/* equilibrium_point_f32[5]= attitude_reference.yaw; */
	
	//Initializes the matrices
	arm_mat_init_f32(&equilibrium_point, 20, 1, (float32_t *)equilibrium_point_f32);
	arm_mat_init_f32(&state_vector, 20, 1, (float32_t *)state_vector_f32);
	arm_mat_init_f32(&error_state_vector, 20, 1, (float32_t *)error_state_vector_f32);
	
	//e(t)=x(t)- equilibrium_point
	//arm_mat_sub_f32(&state_vector, &equilibrium_point, &error_state_vector); // @todo

	return;
}


/* Exported functions definitions --------------------------------------------*/

/** \brief Inicilização do controle de estabilidade.
 *
 * O controlador utiliza a API de DSP da CMSIS, e portanto se baseia fortemente no uso do
 * tipo arm_matrix_instance_f32. Esta \b struct contêm os valores de número de linhas e
 * colunas de matriz, além de um ponteiro para seus elementos (na forma de array).
 * Estes arrays são prealocados globalmente (ver código fonte), para evitar overhead
 * de alocação dinâmica em cada chamada e para evitar que, a cada alocação em uma função, a memória para
 * a qual o ponteiro aponta saia de escopo e seja deletada. Uma vez que as funções são privadas e chamadas
 * em ordem determinística, mutexes não são implementadas (por simplicidade apenas)
 */
void c_control_lqrArthur_init()
{

	// Inicializa as matrizes estaticas
	arm_mat_init_f32(&equilibrium_control, 4, 1, (float32_t *)equilibrium_control_f32);
	arm_mat_init_f32(&K, 4, 20, (float32_t *)K_f32);
}



/** \brief lqrArthur Controller.  */
void c_control_lqrArthur_controller(pv_msg_input * inputData, pv_type_actuation * output_data)
{
	//Initialize result matrices
	arm_mat_init_f32(&control_output, 4, 1, (float32_t *)control_output_f32);
	arm_mat_init_f32(&delta_control,4,1,(float32_t *)delta_control_f32);
	
	pv_type_stability_error error;
	float temp_height_takeoff;

	c_control_lqrArthur_calcErrorStateVector(inputData);
		
	/* -delta_u = K*delta_x */
	//arm_mat_mult_f32(&K, &error_state_vector, &delta_control);  // @todo
	/* u = ur - delta_u */
	//arm_mat_sub_f32(&equilibrium_control, &delta_control, &control_output); // @todo
	
	//The result must be in a struct pv_msg_io_actuation
	output_data->escRightNewtons= (float)control_output.pData[0];
	output_data->escLeftNewtons=	 (float)control_output.pData[1];
	output_data->servoRight=	 (float)control_output.pData[2];
	output_data->servoLeft=	 (float)control_output.pData[3];
    //Declares that the servos will use angle control, rather than torque control
	output_data->servoTorqueControlEnable = 0;

	return;
}

// colocar prototipos das funçoes
// definir os tipos
// 1) buffer overflow
// 2) aritmetic overflow
// 3) divisao por zero


/* IRQ handlers ------------------------------------------------------------- */


