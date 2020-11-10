<<<<<<< HEAD
/**
 * @defgroup   PD_CONTROLLER pd controller
 *
 * @brief      This file implements pd controller.
 *
 * @author     Maxie
 * @date       2020
 */
/*Standard include*/
#include <stdint.h>
#include <string.h>

/*Self include*/
#include "pd_controller.h"

/*Debug include*/
#ifdef PDC_DEBUG
#include <stdio.h>
#endif


/**
 * @brief      This function reset PD_HANDLE_T and set pcfg
 *
 * @param      ph    Pointer to PD_HANDLE_T
 * @param      pcfg  Pointer to PD_CFG_T
 *
 * @return     Result of initialization
 */
uint8_t PDC_Init(PD_HANDLE_T *ph, uint32_t cfg1, float kp, float kd)
{
  uint8_t result = 0xFF;
#ifdef PDC_DEBUG
  if(NULL == ph)
  {
    /*Invalid pointer*/
    result = 0xFE;
  }
#endif

  if(0xFF == result)
  {
    /*Proceed*/
    /*Reset the handle*/
    memset(ph, 0U, sizeof(*ph));
    /*Setup config*/
#define PRIV_HANDLE ph->cfg
    PRIV_HANDLE.cfg1 = cfg1;
    PRIV_HANDLE.kp = kp;
    PRIV_HANDLE.kd = kd;
#undef PRIV_HANDLE

    result = 0U;
  }
#ifdef PDC_DEBUG
  PDC_PrintHandle(ph);
#endif
  return result;
}

/**
 * @brief      This function implements a P controller
 *
 * @param      ph    Pointer to PD_HANDLE_T
 * @param[in]  cmd   Command value
 * @param[in]  act   Feedback value
 *
 * @return     { description_of_the_return_value }
 */
uint8_t PDC_RunP(PD_HANDLE_T *ph, float cmd, float act)
{
  uint8_t result = 0xFFU;
  if(NULL == ph)
  {
    result = 0xFE;
  }
  else
  {
    ph->cmd = cmd;
    ph->act = act;
    ph->data.err_last = ph->data.err_now;
    ph->data.err_now = cmd - act;
    ph->data.val_aft_kp = ph->data.err_now * ph->cfg.kp;
    ph->out = ph->data.val_aft_kp;

    result = 0U;
  }
#ifdef PDC_DEBUG
    /*Print out process*/
    PDC_PrintData(&(ph->data));
    PDC_PrintInterface(ph);
#endif
  return result;
}

/**
 * @brief      This function implements a PD controller. This funtion will calculate dt automatically.
 *
 * @param      ph        Pointer to PD_HANDLE_T
 * @param[in]  cmd       Command value
 * @param[in]  act       Feedback value
 * @param[in]  time_now  Time stamp now
 *
 * @return     { description_of_the_return_value }
 * 
 * @note       Unit of time_now is irrelevant as long as kd is set properlly.
 */
uint8_t PDC_RunPD(PD_HANDLE_T *ph, float cmd, float act, int time_now)
{
  uint8_t result = 0xFFU;
  if(NULL == ph)
  {
    result = 0xFE;
  }
  else
  {
    ph->data.err_last = ph->data.err_now;
    ph->data.t_last   = ph->data.t_now;
    ph->data.derr     = ph->data.err_now - ph->data.err_last;
    ph->data.dt       = ph->data.t_now - ph->data.t_last;
    
    if(!ph->data.dt)
    {
      /*dt is zero return error*/
      result = 0x01;
    }
    else
    {
      ph->data.err_now = cmd - act;
      ph->data.val_aft_kp = ph->data.err_now * ph->cfg.kp;
      ph->data.val_aft_kd = (ph->data.derr * (ph->cfg.kd)) / ((float)ph->data.dt);
      ph->out = (ph->data.val_aft_kp) + (ph->data.val_aft_kd);
      result = 0U;
    }
  }
#ifdef PDC_DEBUG
    /*Print out process*/
    PDC_PrintData(&(ph->data));
    PDC_PrintInterface(ph);
#endif
  return result;
}

/**
 * @brief      This function return output value of the PD_HANDLE_T
 *
 * @param      ph    Pointer to PD_HANDLE_T
 *
 * @return     The output value
 */
float PDC_GetOutput(PD_HANDLE_T *ph)
{
#ifdef PDC_DEBUG
  if(NULL == ph)
  {
    return 0.0f;
  }
#endif
  return ph->out;
}


/*Definition of debug related funcitons*/
#ifdef PDC_DEBUG
void PDC_PrintHandle(PD_HANDLE_T *ph)
{
  if(NULL == ph)
  {
    printf("!! Invalid pointer address of pointer to PD_HANDLE_T !!\n");
  }
  else
  {
    PDC_PrintCfg(&(ph->cfg));
    PDC_PrintData(&(ph->data));
    PDC_PrintInterface(ph);
  }
}

void PDC_PrintCfg(PD_CFG_T *ph)
{
  if(NULL == ph)
  {
    printf("!! Invalid pointer address of pointer to PD_CFG_T !!\n");
  }
  else
  {
    printf("cfg1 is %d\n", ph->cfg1);
    printf("kp is %5.5f\n", ph->kp);
    printf("kd is %5.5f\n", ph->kd);
  }
}

void PDC_PrintData(PD_DATA_T *ph)
{
  if(NULL == ph)
  {
    printf("!! Invalid pointer address of pointer to PD_DATA_T !!\n");
  }
  else
  {
    printf("err_now is: %5.5f\n",ph->err_now);
    printf("err_last is: %5.5f\n",ph->err_last);
    printf("derr is: %5.5f\n",ph->derr);
    printf("t_now is: %5d\n",ph->t_now);
    printf("t_last is: %5d\n",ph->t_last);
    printf("dt is: %5d\n",ph->dt);
    printf("val_aft_kp is: %5.5f\n",ph->val_aft_kp);
    printf("val_aft_kd is: %5.5f\n",ph->val_aft_kd);
  }
}

void PDC_PrintInterface(PD_HANDLE_T *ph)
{
  if(NULL == ph)
  {
    printf("!! Invalid pointer address of pointer to PD_HANDLE_T !!\n");
  }
  else
  {
    printf("cmd is:%5.5f\n",ph->cmd);
    printf("act is:%5.5f\n",ph->act);
    printf("out is:%5.5f\n\n",ph->out);
  }
}
=======
/**
 * @defgroup   PD_CONTROLLER pd controller
 *
 * @brief      This file implements pd controller.
 *
 * @author     Maxie
 * @date       2020
 */
/*Standard include*/
#include <stdint.h>
#include <string.h>

/*Self include*/
#include "pd_controller.h"

/*Debug include*/
#ifdef PDC_DEBUG
#include <stdio.h>
#endif


/**
 * @brief      This function reset PD_HANDLE_T and set pcfg
 *
 * @param      ph    Pointer to PD_HANDLE_T
 * @param      pcfg  Pointer to PD_CFG_T
 *
 * @return     Result of initialization
 */
uint8_t PDC_Init(PD_HANDLE_T *ph, uint32_t cfg1, float kp, float kd)
{
  uint8_t result = 0xFF;
#ifdef PDC_DEBUG
  if(NULL == ph)
  {
    /*Invalid pointer*/
    result = 0xFE;
  }
#endif

  if(0xFF == result)
  {
    /*Proceed*/
    /*Reset the handle*/
    memset(ph, 0U, sizeof(*ph));
    /*Setup config*/
#define PRIV_HANDLE ph->cfg
    PRIV_HANDLE.cfg1 = cfg1;
    PRIV_HANDLE.kp = kp;
    PRIV_HANDLE.kd = kd;
#undef PRIV_HANDLE

    result = 0U;
  }
#ifdef PDC_DEBUG
  PDC_PrintHandle(ph);
#endif
  return result;
}

/**
 * @brief      This function implements a P controller
 *
 * @param      ph    Pointer to PD_HANDLE_T
 * @param[in]  cmd   Command value
 * @param[in]  act   Feedback value
 *
 * @return     { description_of_the_return_value }
 */
uint8_t PDC_RunP(PD_HANDLE_T *ph, float cmd, float act)
{
  uint8_t result = 0xFFU;
  if(NULL == ph)
  {
    result = 0xFE;
  }
  else
  {
    ph->cmd = cmd;
    ph->act = act;
    ph->data.err_last = ph->data.err_now;
    ph->data.err_now = cmd - act;
    ph->data.val_aft_kp = ph->data.err_now * ph->cfg.kp;
    ph->out = ph->data.val_aft_kp;

    result = 0U;
  }
#ifdef PDC_DEBUG
    /*Print out process*/
    PDC_PrintData(&(ph->data));
    PDC_PrintInterface(ph);
#endif
  return result;
}

/**
 * @brief      This function implements a PD controller. This funtion will calculate dt automatically.
 *
 * @param      ph        Pointer to PD_HANDLE_T
 * @param[in]  cmd       Command value
 * @param[in]  act       Feedback value
 * @param[in]  time_now  Time stamp now
 *
 * @return     { description_of_the_return_value }
 * 
 * @note       Unit of time_now is irrelevant as long as kd is set properlly.
 */
uint8_t PDC_RunPD(PD_HANDLE_T *ph, float cmd, float act, int time_now)
{
  uint8_t result = 0xFFU;
  if(NULL == ph)
  {
    result = 0xFE;
  }
  else
  {
    ph->data.err_last = ph->data.err_now;
    ph->data.t_last   = ph->data.t_now;
    ph->data.derr     = ph->data.err_now - ph->data.err_last;
    ph->data.dt       = ph->data.t_now - ph->data.t_last;
    
    if(!ph->data.dt)
    {
      /*dt is zero return error*/
      result = 0x01;
    }
    else
    {
      ph->data.err_now = cmd - act;
      ph->data.val_aft_kp = ph->data.err_now * ph->cfg.kp;
      ph->data.val_aft_kd = (ph->data.derr * (ph->cfg.kd)) / ((float)ph->data.dt);
      ph->out = (ph->data.val_aft_kp) + (ph->data.val_aft_kd);
      result = 0U;
    }
  }
#ifdef PDC_DEBUG
    /*Print out process*/
    PDC_PrintData(&(ph->data));
    PDC_PrintInterface(ph);
#endif
  return result;
}

/**
 * @brief      This function return output value of the PD_HANDLE_T
 *
 * @param      ph    Pointer to PD_HANDLE_T
 *
 * @return     The output value
 */
float PDC_GetOutput(PD_HANDLE_T *ph)
{
#ifdef PDC_DEBUG
  if(NULL == ph)
  {
    return 0.0f;
  }
#endif
  return ph->out;
}


/*Definition of debug related funcitons*/
#ifdef PDC_DEBUG
void PDC_PrintHandle(PD_HANDLE_T *ph)
{
  if(NULL == ph)
  {
    printf("!! Invalid pointer address of pointer to PD_HANDLE_T !!\n");
  }
  else
  {
    PDC_PrintCfg(&(ph->cfg));
    PDC_PrintData(&(ph->data));
    PDC_PrintInterface(ph);
  }
}

void PDC_PrintCfg(PD_CFG_T *ph)
{
  if(NULL == ph)
  {
    printf("!! Invalid pointer address of pointer to PD_CFG_T !!\n");
  }
  else
  {
    printf("cfg1 is %d\n", ph->cfg1);
    printf("kp is %5.5f\n", ph->kp);
    printf("kd is %5.5f\n", ph->kd);
  }
}

void PDC_PrintData(PD_DATA_T *ph)
{
  if(NULL == ph)
  {
    printf("!! Invalid pointer address of pointer to PD_DATA_T !!\n");
  }
  else
  {
    printf("err_now is: %5.5f\n",ph->err_now);
    printf("err_last is: %5.5f\n",ph->err_last);
    printf("derr is: %5.5f\n",ph->derr);
    printf("t_now is: %5d\n",ph->t_now);
    printf("t_last is: %5d\n",ph->t_last);
    printf("dt is: %5d\n",ph->dt);
    printf("val_aft_kp is: %5.5f\n",ph->val_aft_kp);
    printf("val_aft_kd is: %5.5f\n",ph->val_aft_kd);
  }
}

void PDC_PrintInterface(PD_HANDLE_T *ph)
{
  if(NULL == ph)
  {
    printf("!! Invalid pointer address of pointer to PD_HANDLE_T !!\n");
  }
  else
  {
    printf("cmd is:%5.5f\n",ph->cmd);
    printf("act is:%5.5f\n",ph->act);
    printf("out is:%5.5f\n\n",ph->out);
  }
}
>>>>>>> a1aecd2... Initial commit
#endif