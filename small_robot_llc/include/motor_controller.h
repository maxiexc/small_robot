
/**
 * @defgroup   MOTOR_CONTROLLER motor controller
 *
 * @brief      This file implements motor controller.
 *
 * @author     Maxie
 * @date       2020
 */
#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H
#ifdef __cplusplus
extern "C" {
#endif
/*Stamdard includes*/


/*Platform specific includes*/

/*Aplication includes*/
#include "pd_controller.h"

/*Predefien values*/
/*Debug Macro*/
#ifdef DEBUG
#define MOTC_DEBUG
#endif

/**
 * @brief      Motor control mode
 */
typedef enum
{
  MOTC_MODE_ENUM_POS_ABS,
  MOTC_MODE_ENUM_POS_INC,
  MOTC_MODE_ENUM_SPEED,  
} MOTC_MODE_ENUM_T;

typedef struct
{
  float max;
  float min;
} MOTC_MINMAX_F_T;

typedef struct
{
  int32_t max;
  int32_t min;
} MOTC_MINMAX_I32_T;

/**
 * @brief      Motor configuration
 */
typedef struct
{
  MOTC_MODE_ENUM_T mode;
  uint8_t is_reverse;
  int motor_id;
  int encoder_id;
  uint32_t pulse_per_rev;
//  float d_duty_max_001;           /**< Maximum duty chage per iteration with unit of 0.01*/
  MOTC_MINMAX_F_T range_dduty;   /**< Maximum duty change per iteration with unit of 0.01, only max is used*/
  MOTC_MINMAX_F_T range_duty;     /**< Range of output duty in absolute format*/
  MOTC_MINMAX_I32_T range_s;      /**<Range of speed*/
  MOTC_MINMAX_I32_T range_p_inc;  /**/
  MOTC_MINMAX_I32_T range_p_abs;  
//  float abs_duty_max;           /**< Maximum outptu duty*/
//  float abs_duty_min;           /**< Minimal output duty*/
} MOTC_CONFIG_T;

typedef struct
{
  int32_t pos_abs;  /*Absolute position, unit in pulse*/
  int32_t pos_inc;  /*Increment position, unit in pulse*/
  int32_t speed;    /*Speed, unit in PPS*/
} MOTC_INPUT_T;

typedef struct
{
  int32_t now;
  int32_t last;
  int32_t diff;
} MOTC_SIG_I32_T;

typedef struct
{
  MOTC_CONFIG_T cfg;
  MOTC_INPUT_T  cmd;
  MOTC_INPUT_T  act;
  PD_HANDLE_T position_pdc; /*Position controller handle*/
  PD_HANDLE_T speed_pdc; /*Speed controller handle*/
  MOTC_SIG_I32_T time;
  MOTC_SIG_I32_T speed;
  MOTC_SIG_I32_T pos;
  float cmd_out; /*Output command*/
} MOTC_HANDLE_T;


uint32_t MOTC_Init(MOTC_HANDLE_T *ph,
                   MOTC_MODE_ENUM_T mode,
                   uint8_t is_reverse,
                   int motor_id,
                   int encoder_id,
                   uint32_t pulse_per_rev,
                   float d_duty_max_001,
                   float abs_duty_max,
                   float abs_duty_min
                   );
uint8_t MOTC_SetPosController(MOTC_HANDLE_T *ph, uint32_t cfg1, float kp, float kd);
uint8_t MOTC_SetSpeedController(MOTC_HANDLE_T *ph, uint32_t cfg1, float kp, float kd);
uint8_t MOTC_SetCmd(MOTC_HANDLE_T *ph, int32_t val);
uint8_t MOTC_SetEncoder(MOTC_HANDLE_T *ph, int32_t enc_now, int32_t time_now);
uint8_t MOTC_RunPositionController(MOTC_HANDLE_T *ph);
uint8_t MOTC_RunSpeedController(MOTC_HANDLE_T *ph);
uint32_t MOTC_Run(MOTC_HANDLE_T *ph);
float MOTC_LimiterSym(float in, float max_abs);
float MOTC_DeadZoneSym(float in, float thd);

#ifdef __cplusplus
}
#endif

#endif