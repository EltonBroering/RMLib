/**
  ******************************************************************************
  * @file    modules/common/pv_typedefs.h
  * @author  Martin Vincent Bloedorn
  * @version V1.0.0
  * @date    10-February-2014
  * @brief   Definições de tipos e estruturas de mensagens para o projeto.
  *
  *****************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PV_TYPEDEFS_H
#define PV_TYPEDEFS_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f4xx_conf.h"
#include "Includes.h"

#ifdef __cplusplus
 extern "C" {
#endif

 /** @addtogroup Common_Components
   * @{
   */

 /** @addtogroup PV_Typedefs
   *
   * Definições de tipagem e estruras de mensagem para o projeto.
   * @{
   */

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/** \brief Um par de floats. */
typedef struct
{
  float accRaw[3];
  float gyrRaw[3];
  float magRaw[3];
  float temp;
  unsigned int  sampleTime;
} pv_type_imuOutput;

/** \brief Informações dos dados do controle remoto.*/
typedef struct 
{
  int  joystick[4];
  unsigned int vrPot;
  bool aButton;
  bool bButton;
  unsigned int  sampleTime;
} pv_type_receiverOutput;

/** \brief Informações do sonar*/
typedef struct 
{
  float altitude;
  unsigned int  sampleTime;
} pv_type_sonarOutput;

/** \brief Tipo do ESC com informações.*/
typedef struct 
{
  char  ID[2];
  float angularSpeed[2];
  float current[2];
  float voltage[2];
  float rpm[2];
  unsigned int  sampleTime;
} pv_type_escOutput;

/** \brief Estrutura para orientação do VANT.*/
typedef struct
{
  float roll, pitch, yaw;
  float dotRoll, dotPitch, dotYaw;
} pv_type_datapr_attitude;

/** \brief Estrutura para posição do VANT.*/
typedef struct
{
  float x, y, z;
  float dotX, dotY, dotZ;
} pv_type_datapr_position;

/** \brief Estrutura para stados dos servo do VANT.*/
typedef struct
{
  float alphal, alphar;
  float dotAlphal, dotAlphar;
} pv_type_datapr_servos;

/** \brief Dados do servo*/
typedef struct
{
  char  idRight;
  char  idLeft;
  pv_type_datapr_servos servo;
  uint8_t status_errorRight;
  uint8_t status_errorLeft;
  uint8_t status_detaiRight;
  uint8_t status_detaiLeft;
} pv_type_servoOutput;

/** \brief Estrutura para dados de atuação.*/
typedef struct
{
  bool  servoTorqueControlEnable;
  float servoLeft;
  float escLeftNewtons;
  float escRightSpeed;
  float servoRight;
  float escRightNewtons;
  float escLeftSpeed;
} pv_type_actuation;

/** \brief Estrutura para dados de comportamento.*/
typedef struct
{
  float rpy[3];
  float drpy[3];
  float xyz[3];
  float dxyz[3];
} pv_type_vantBehavior;

/** \brief Estrutura para dados de comportamento.*/
typedef struct
{
  int timeStamp;
  bool validity;
  float lat;
  bool latDirection;
  float lon;
  bool lonDirection;
  float speedOverGround;
  float trueCourse;
  int dateStamp;
  float variation;
  bool variationDirection;
  char ModeIndicator;
  unsigned int sampleTime;
} pv_type_gpsOutput;

/** \brief Integral do erro dos angulos de orientacao VANT.*/
typedef struct {
	float z, roll, pitch, yaw;
} pv_type_stability_error;

typedef struct {
	float x, y, z, yaw;
} pv_type_pathtrack_error;

/* Exported messages ---------------------------------------------------------*/

/** \brief Estrutura de mensagens para os dois RX24f e ESCs.
 *
 * Pacote prevê passagem de referências de velocidade para os ESCs em <b>rad/s</b>,
 * referências agulares em <b>rad</b> e torque em <b>N.m</b>. Para o servo, o módulo de
 * IO automaticamente alternará entre referências de ângulo para torque se a flag
 *  \b servoTorqueControlEnable estiver setada.
 */

/** \brief Estrutura de mensagem de saida da estrutura thread input.*/
typedef struct
{
  pv_type_imuOutput      imuOutput;
  pv_type_receiverOutput receiverOutput;
  pv_type_sonarOutput    sonarOutput;
  pv_type_escOutput      escOutput;
  pv_type_servoOutput    servosOutput;
  pv_type_datapr_attitude attitude;
  pv_type_datapr_position position;
  pv_type_datapr_attitude attitude_reference;
  pv_type_datapr_position position_reference;
  unsigned int cicleTime;
  unsigned int heartBeat;
  bool init;
  bool securityStop;
  bool flightmode;
  bool enableintegration;
} pv_msg_input;

/** \brief Estrutura de mensagem de saida da estrutura thread de controle.*/
typedef struct
{
  pv_type_actuation    actuation;
  pv_type_vantBehavior vantBehavior;
  unsigned int cicleTime;
  unsigned int heartBeat;
} pv_msg_controlOutput;

/** \brief Estrutura de mensagem de saida da estrutura thread de gps.*/
typedef struct
{
  pv_type_gpsOutput gpsOutput;
  unsigned int cicleTime;
  unsigned int heartBeat;
} pv_msg_gps;

/** \brief Estrutura de mensagem de saida da estrutura thread de state machine.*/
typedef struct
{
  unsigned int cicleTime;
  unsigned int heartBeat;
} pv_msg_sm;

/** \brief Tipo do Servo com informações. */
typedef struct
{
	uint8_t servo_id;
	uint32_t heartBeat;
	int16_t  pwm;
	float angularSpeed;
	float position;
} pv_msg_servo;

/* Estruturas de mensagems do multiwii*/
//GPS Raw Data
typedef struct
{
	uint8_t gpsFix;
	uint8_t gpsNumSat;
	uint32_t gpsCoordLat;
	uint32_t gpsCoordLong;
	uint16_t gpsAltitude;
	uint16_t gpsSpeed;
	uint16_t gpsGroundCourse;
}pv_type_gpsRawData;

//GPS Comp Data
typedef struct
{
	uint16_t gpsDistanceToHome;
	uint16_t gpsDirectionToHome;
	uint8_t gpsUpdate;
}pv_type_gpsCompData;

typedef struct
{
	uint8_t vbat;
	uint16_t intPowerMeterSum;
	uint16_t rssi;
	uint16_t amperage;
} pv_type_analog;

typedef struct
{
	int32_t estAlt;
	int16_t vario;
}pv_type_altitude;

typedef struct
{
	uint16_t cycleTime;
	uint16_t i2cErrorsCount;
	uint16_t sensor;
	uint32_t flag;
}pv_type_status;

typedef struct
{
	uint16_t channels[12];
	uint16_t normChannels[12];
}pv_type_rc;

//IDENT
typedef struct
{
	uint8_t version;
	uint8_t multitype;
	uint8_t mspVersion;
	uint32_t capability;
}pv_type_ident;

typedef struct
	{
	//Motor Pins
	uint8_t pwmPin[8];
	//Motor Speed
	uint16_t motorSpeed[6];
	//Servos
	int16_t servos[6];
}pv_type_motors;

typedef struct
	{
	//Debug
	int16_t debug[4];
}pv_type_debug;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* Header-defined wrapper functions ----------------------------------------- */

 /**
   * @}
   */
 /**
   * @}
   */

#ifdef __cplusplus
}
#endif

#endif //PV_TYPEDEFS_H

