<<<<<<< HEAD

#include <stdio.h>
#include <stdint.h>

#include "motor_controller.h"
#include "pd_controller.h"


/**
 * @brief      { function_description }
 *
 * @param      ph    { parameter_description }
 * @param[in]  mode  The mode
 *
 * @return     This function set mode of MOTC_HANDLE_T.
 */

/**
 * @brief      { function_description }
 *
 * @param      ph              { parameter_description }
 * @param[in]  mode            The mode
 * @param[in]  is_reverse      Indicates if reverse
 * @param[in]  motor_id        The motor identifier
 * @param[in]  encoder_id      The encoder identifier
 * @param[in]  pulse_per_rev   The pulse per reverse
 * @param[in]  d_duty_max_001  The d duty maximum 001
 * @param[in]  abs_duty_max    The absolute duty maximum
 * @param[in]  abs_duty_min    The absolute duty minimum
 *
 * @return     { description_of_the_return_value }
 */
uint32_t MOTC_Init(MOTC_HANDLE_T *ph,
                   MOTC_MODE_ENUM_T mode,
                   uint8_t is_reverse,
                   int motor_id,
                   int encoder_id,
                   uint32_t pulse_per_rev,
                   float d_duty_max_001,
                   float abs_duty_max,
                   float abs_duty_min
                   )
{
  if(NULL == ph)
  {
    return 0xFF;
  }
  else
  {
    switch(mode)
    {
      case MOTC_MODE_ENUM_POS_ABS:
      ph->cfg.mode = mode;
      break;

      case MOTC_MODE_ENUM_POS_INC:
      ph->cfg.mode = mode;
      break;

      case MOTC_MODE_ENUM_SPEED:
      ph->cfg.mode = mode;
      break;

      default:
      return 0xFE;
      break;
    }
#define PRIV_HANDLE ph->cfg
    PRIV_HANDLE.is_reverse = is_reverse;
    PRIV_HANDLE.motor_id = motor_id;
    PRIV_HANDLE.encoder_id = encoder_id;
    PRIV_HANDLE.pulse_per_rev = pulse_per_rev;
    PRIV_HANDLE.range_dduty.max = d_duty_max_001;
    PRIV_HANDLE.range_duty.max = abs_duty_max;
    PRIV_HANDLE.range_duty.min = abs_duty_min;
#undef PRIV_HANDLE
  }

  return 0;
}

/**
 * @brief      { function_description }
 *
 * @param      ph    { parameter_description }
 * @param[in]  cfg  The configuration 1
 * @param[in]  kp    { parameter_description }
 * @param[in]  kd    { parameter_description }
 *
 * @return     { description_of_the_return_value }
 * 
 * @note       kd is not use at the moment
 */
uint8_t MOTC_SetPosController(MOTC_HANDLE_T *ph, uint32_t cfg1, float kp, float kd)
{
#ifdef MOTC_DEBUG
  if(NULL == ph)
  {
    return 0xFE;
  }
#endif 

if(PDC_Init(&(ph->position_pdc), cfg1, kp, kd))
{
  return 0x0FD;
}
  return 0;
}
/**
 * @brief      { function_description }
 *
 * @param      ph    { parameter_description }
 * @param[in]  cfg  The configuration 1
 * @param[in]  kp    { parameter_description }
 * @param[in]  kd    { parameter_description }
 *
 * @return     { description_of_the_return_value }
 */
uint8_t MOTC_SetSpeedController(MOTC_HANDLE_T *ph, uint32_t cfg1, float kp, float kd)
{
#ifdef MOTC_DEBUG
  if(NULL == ph)
  {
    return 0xFE;
  }
#endif

if(PDC_Init(&(ph->speed_pdc), cfg1, kp, kd))
{
  return 0x0FD;
}
  return 0;
}

uint8_t MOTC_SetCmd(MOTC_HANDLE_T *ph, int32_t val)
{
#ifdef MOTC_DEBUG
  if(NULL == ph)
  {
    return 0xFE;
  }
#endif 

  switch (ph->cfg.mode)
  {
    case MOTC_MODE_ENUM_POS_ABS:
    /*Not yet implement*/
    ph->cmd.pos_abs = val;
    break;

    case MOTC_MODE_ENUM_POS_INC:
    ph->cmd.pos_inc = val;
    if(ph->cfg.is_reverse)
    {
      /*Reverse pos_inc command*/
      ph->cmd.pos_inc *= (-1);
    }
    break;

    case MOTC_MODE_ENUM_SPEED:
    ph->cmd.speed = val;
    if(ph->cfg.is_reverse)
    {
      /*Reverse speed command*/
      ph->cmd.speed *= (-1);
    }
    break;

    default:
    /*Exception*/
    return 0xFE;     
    break;
  }

  return 0U;
}

uint8_t MOTC_SetEncoder(MOTC_HANDLE_T *ph, int32_t enc_now, int32_t time_now)
{
#ifdef MOTC_DEBUG
  if(NULL == ph)
  {
    return 0xFE;
  }
#endif 

  /*Update and check*/
  ph->time.last = ph->time.now;
  ph->time.now = time_now;
  ph->time.diff = ph->time.now - ph->time.last;
  
  ph->pos.last = ph->pos.now;
  ph->pos.now = enc_now;
  ph->pos.diff = ph->pos.now - ph->pos.last;

  ph->speed.last = ph->speed.now;
  /*Derive speed*/
  if(!ph->time.diff)
  {
    return 0xFE;
  }
  else
  {
    ph->act.speed = (ph->pos.diff * 1000)/ph->time.diff;
    ph->speed.now = ph->act.speed;
  }

  return 0U;
}

uint8_t MOTC_RunPositionController(MOTC_HANDLE_T *ph)
{
#ifdef MOTC_DEBUG
  if(NULL == ph)
  {
    return 0xFE;
  }
#endif 

  if(PDC_RunP(&(ph->position_pdc), (float)ph->cmd.pos_abs, (float)ph->act.pos_abs))
  {
    /*Error occurred during running speed controller*/
    return 0xFD;
  }

  return 0x0U;
}

uint8_t MOTC_RunSpeedController(MOTC_HANDLE_T *ph)
{
#ifdef MOTC_DEBUG
  if(NULL == ph)
  {
    return 0xFF;
  }
#endif 
  
  //if(ph->cfg.is_reverse)
  //{
  //  /*Reverse speed command*/
  //  ph->cmd.speed *= (-1);
  //}

  /*Run Speed controller*/
  if(PDC_RunP(&(ph->speed_pdc), (float)ph->cmd.speed, (float)ph->act.speed))
  {
    /*Error occurred during running speed controller*/
    return 0xFD;
  }

  return 0x0U;
}

/**
 * @brief      Run motor controller. Function represent 1 iteration of control process.
 *
 * @param      ph         { parameter_description }
 * @param[in]  <unnamed>  { parameter_description }
 *
 * @return     { description_of_the_return_value }
 */
uint32_t MOTC_Run(MOTC_HANDLE_T *ph)
{
#ifdef MOTC_DEBUG
  if(NULL == ph)
  {
    return 0xFE;
  }
#endif 
  /*Mode selection*/

  if(ph->cfg.mode != MOTC_MODE_ENUM_SPEED)
  {
    /*Process position controller required input*/
    /*Nothing to process so far*/

    /*Run positon controller*/
    if(MOTC_RunPositionController(ph))
    {
      /*Error occurred during running position controller*/
      return 0x01;
    }
    /*Process positon controller output*/
    ph->cmd.speed = MOTC_LimiterSym(ph->cmd_out, ph->cfg.range_s.max);
  }
  else
  {
    /*Process speed controller required input*/
    //if(ph->cfg.is_reverse)
    //{
    //  /*Reverse speed command*/
    //  ph->cmd.speed *= (-1);
    //}
  }

  /*Run speed controller*/
  if(MOTC_RunSpeedController(ph))
  {
    /*Error occurred during running speed controller*/
    return 0x02;
  }

  /*Process speed controller output*/
  /*Limit value of d_duty*/
  ph->cmd_out += (MOTC_LimiterSym(ph->speed_pdc.out, ph->cfg.range_dduty.max) * 0.01f) ;
  /*Limit value of duty within -1.0 and 1.0*/
  ph->cmd_out = MOTC_LimiterSym(ph->cmd_out, ph->cfg.range_duty.max);
  /*Dead zone detection*/
  ph->cmd_out = MOTC_DeadZoneSym(ph->cmd_out, ph->cfg.range_duty.min);

 return 0;
}


/**
 * @defgroup   MOTOR_CONTROLLER motor controller
 *
 * @brief      This file implements motor controller.
 *
 * @author     Maxie
 * @date       2020
 */

float MOTC_LimiterSym(float in, float max_abs)
{
  if(in > max_abs)
  {
    return max_abs;
  }
  else if(in < ((-1.0) * max_abs))
  {
    return ((-1.0) * max_abs);
  }
  else
  {
    return in;
  }
}

float MOTC_DeadZoneSym(float in, float thd)
{
  if((in < thd) && (in > 0.0f))
  {
    return thd;
  }
  else if((in > ((-1.0) * thd)) && (in < 0.0f))
  {
    return ((-1.0) * thd);
  }
  else
  {
    return in;
  }
=======

#include <stdio.h>
#include <stdint.h>

#include "motor_controller.h"
#include "pd_controller.h"


/**
 * @brief      { function_description }
 *
 * @param      ph    { parameter_description }
 * @param[in]  mode  The mode
 *
 * @return     This function set mode of MOTC_HANDLE_T.
 */

/**
 * @brief      { function_description }
 *
 * @param      ph              { parameter_description }
 * @param[in]  mode            The mode
 * @param[in]  is_reverse      Indicates if reverse
 * @param[in]  motor_id        The motor identifier
 * @param[in]  encoder_id      The encoder identifier
 * @param[in]  pulse_per_rev   The pulse per reverse
 * @param[in]  d_duty_max_001  The d duty maximum 001
 * @param[in]  abs_duty_max    The absolute duty maximum
 * @param[in]  abs_duty_min    The absolute duty minimum
 *
 * @return     { description_of_the_return_value }
 */
uint32_t MOTC_Init(MOTC_HANDLE_T *ph,
                   MOTC_MODE_ENUM_T mode,
                   uint8_t is_reverse,
                   int motor_id,
                   int encoder_id,
                   uint32_t pulse_per_rev,
                   float d_duty_max_001,
                   float abs_duty_max,
                   float abs_duty_min
                   )
{
  if(NULL == ph)
  {
    return 0xFF;
  }
  else
  {
    switch(mode)
    {
      case MOTC_MODE_ENUM_POS_ABS:
      ph->cfg.mode = mode;
      break;

      case MOTC_MODE_ENUM_POS_INC:
      ph->cfg.mode = mode;
      break;

      case MOTC_MODE_ENUM_SPEED:
      ph->cfg.mode = mode;
      break;

      default:
      return 0xFE;
      break;
    }
#define PRIV_HANDLE ph->cfg
    PRIV_HANDLE.is_reverse = is_reverse;
    PRIV_HANDLE.motor_id = motor_id;
    PRIV_HANDLE.encoder_id = encoder_id;
    PRIV_HANDLE.pulse_per_rev = pulse_per_rev;
    PRIV_HANDLE.range_dduty.max = d_duty_max_001;
    PRIV_HANDLE.range_duty.max = abs_duty_max;
    PRIV_HANDLE.range_duty.min = abs_duty_min;
#undef PRIV_HANDLE
  }

  return 0;
}

/**
 * @brief      { function_description }
 *
 * @param      ph    { parameter_description }
 * @param[in]  cfg  The configuration 1
 * @param[in]  kp    { parameter_description }
 * @param[in]  kd    { parameter_description }
 *
 * @return     { description_of_the_return_value }
 * 
 * @note       kd is not use at the moment
 */
uint8_t MOTC_SetPosController(MOTC_HANDLE_T *ph, uint32_t cfg1, float kp, float kd)
{
#ifdef MOTC_DEBUG
  if(NULL == ph)
  {
    return 0xFE;
  }
#endif 

if(PDC_Init(&(ph->position_pdc), cfg1, kp, kd))
{
  return 0x0FD;
}
  return 0;
}
/**
 * @brief      { function_description }
 *
 * @param      ph    { parameter_description }
 * @param[in]  cfg  The configuration 1
 * @param[in]  kp    { parameter_description }
 * @param[in]  kd    { parameter_description }
 *
 * @return     { description_of_the_return_value }
 */
uint8_t MOTC_SetSpeedController(MOTC_HANDLE_T *ph, uint32_t cfg1, float kp, float kd)
{
#ifdef MOTC_DEBUG
  if(NULL == ph)
  {
    return 0xFE;
  }
#endif

if(PDC_Init(&(ph->speed_pdc), cfg1, kp, kd))
{
  return 0x0FD;
}
  return 0;
}

uint8_t MOTC_SetCmd(MOTC_HANDLE_T *ph, int32_t val)
{
#ifdef MOTC_DEBUG
  if(NULL == ph)
  {
    return 0xFE;
  }
#endif 

  switch (ph->cfg.mode)
  {
    case MOTC_MODE_ENUM_POS_ABS:
    /*Not yet implement*/
    ph->cmd.pos_abs = val;
    break;

    case MOTC_MODE_ENUM_POS_INC:
    ph->cmd.pos_inc = val;
    if(ph->cfg.is_reverse)
    {
      /*Reverse pos_inc command*/
      ph->cmd.pos_inc *= (-1);
    }
    break;

    case MOTC_MODE_ENUM_SPEED:
    ph->cmd.speed = val;
    if(ph->cfg.is_reverse)
    {
      /*Reverse speed command*/
      ph->cmd.speed *= (-1);
    }
    break;

    default:
    /*Exception*/
    return 0xFE;     
    break;
  }

  return 0U;
}

uint8_t MOTC_SetEncoder(MOTC_HANDLE_T *ph, int32_t enc_now, int32_t time_now)
{
#ifdef MOTC_DEBUG
  if(NULL == ph)
  {
    return 0xFE;
  }
#endif 

  /*Update and check*/
  ph->time.last = ph->time.now;
  ph->time.now = time_now;
  ph->time.diff = ph->time.now - ph->time.last;
  
  ph->pos.last = ph->pos.now;
  ph->pos.now = enc_now;
  ph->pos.diff = ph->pos.now - ph->pos.last;

  ph->speed.last = ph->speed.now;
  /*Derive speed*/
  if(!ph->time.diff)
  {
    return 0xFE;
  }
  else
  {
    ph->act.speed = (ph->pos.diff * 1000)/ph->time.diff;
    ph->speed.now = ph->act.speed;
  }

  return 0U;
}

uint8_t MOTC_RunPositionController(MOTC_HANDLE_T *ph)
{
#ifdef MOTC_DEBUG
  if(NULL == ph)
  {
    return 0xFE;
  }
#endif 

  if(PDC_RunP(&(ph->position_pdc), (float)ph->cmd.pos_abs, (float)ph->act.pos_abs))
  {
    /*Error occurred during running speed controller*/
    return 0xFD;
  }

  return 0x0U;
}

uint8_t MOTC_RunSpeedController(MOTC_HANDLE_T *ph)
{
#ifdef MOTC_DEBUG
  if(NULL == ph)
  {
    return 0xFF;
  }
#endif 
  
  //if(ph->cfg.is_reverse)
  //{
  //  /*Reverse speed command*/
  //  ph->cmd.speed *= (-1);
  //}

  /*Run Speed controller*/
  if(PDC_RunP(&(ph->speed_pdc), (float)ph->cmd.speed, (float)ph->act.speed))
  {
    /*Error occurred during running speed controller*/
    return 0xFD;
  }

  return 0x0U;
}

/**
 * @brief      Run motor controller. Function represent 1 iteration of control process.
 *
 * @param      ph         { parameter_description }
 * @param[in]  <unnamed>  { parameter_description }
 *
 * @return     { description_of_the_return_value }
 */
uint32_t MOTC_Run(MOTC_HANDLE_T *ph)
{
#ifdef MOTC_DEBUG
  if(NULL == ph)
  {
    return 0xFE;
  }
#endif 
  /*Mode selection*/

  if(ph->cfg.mode != MOTC_MODE_ENUM_SPEED)
  {
    /*Process position controller required input*/
    /*Nothing to process so far*/

    /*Run positon controller*/
    if(MOTC_RunPositionController(ph))
    {
      /*Error occurred during running position controller*/
      return 0x01;
    }
    /*Process positon controller output*/
    ph->cmd.speed = MOTC_LimiterSym(ph->cmd_out, ph->cfg.range_s.max);
  }
  else
  {
    /*Process speed controller required input*/
    //if(ph->cfg.is_reverse)
    //{
    //  /*Reverse speed command*/
    //  ph->cmd.speed *= (-1);
    //}
  }

  /*Run speed controller*/
  if(MOTC_RunSpeedController(ph))
  {
    /*Error occurred during running speed controller*/
    return 0x02;
  }

  /*Process speed controller output*/
  /*Limit value of d_duty*/
  ph->cmd_out += (MOTC_LimiterSym(ph->speed_pdc.out, ph->cfg.range_dduty.max) * 0.01f) ;
  /*Limit value of duty within -1.0 and 1.0*/
  ph->cmd_out = MOTC_LimiterSym(ph->cmd_out, ph->cfg.range_duty.max);
  /*Dead zone detection*/
  ph->cmd_out = MOTC_DeadZoneSym(ph->cmd_out, ph->cfg.range_duty.min);

 return 0;
}


/**
 * @defgroup   MOTOR_CONTROLLER motor controller
 *
 * @brief      This file implements motor controller.
 *
 * @author     Maxie
 * @date       2020
 */

float MOTC_LimiterSym(float in, float max_abs)
{
  if(in > max_abs)
  {
    return max_abs;
  }
  else if(in < ((-1.0) * max_abs))
  {
    return ((-1.0) * max_abs);
  }
  else
  {
    return in;
  }
}

float MOTC_DeadZoneSym(float in, float thd)
{
  if((in < thd) && (in > 0.0f))
  {
    return thd;
  }
  else if((in > ((-1.0) * thd)) && (in < 0.0f))
  {
    return ((-1.0) * thd);
  }
  else
  {
    return in;
  }
>>>>>>> a1aecd2... Initial commit
}