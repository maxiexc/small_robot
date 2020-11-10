
/**
 * @defgroup   PD_CONTROLLER pd controller
 *
 * @brief      This file implements pd controller.
 *
 * @author     Maxie
 * @date       2020
 * 
 */
#ifndef PD_CONTROLLER_H
#define PD_CONTROLLER_H
#ifdef __cplusplus
extern "C" {
#endif
/*Standard include*/
#include <stdint.h>

/*Configuration deifne*/
/*Debug Macro*/
#ifdef DEBUG
#define PDC_DEBUG
#endif

/*Structs*/
typedef struct
{
  uint32_t  cfg1;
  float     kp;
  float     kd;
} PD_CFG_T;

typedef struct
{
  float err_now;
  float err_last;
  float derr;
  int32_t   t_now;
  int32_t   t_last;
  int32_t   dt;
  float val_aft_kp;
  float val_aft_kd;
} PD_DATA_T;

typedef struct
{
  PD_CFG_T cfg;
  PD_DATA_T data;
  float cmd;
  float act;
  float out;
} PD_HANDLE_T;


uint8_t PDC_Init(PD_HANDLE_T *ph, uint32_t cfg1, float kp, float kd);

uint8_t PDC_RunP(PD_HANDLE_T *ph, float cmd, float act);

uint8_t PDC_RunPD(PD_HANDLE_T *ph, float cmd, float act, int32_t time_now);

float PDC_GetOutput(PD_HANDLE_T *ph);

#ifdef PDC_DEBUG
void PDC_PrintHandle(PD_HANDLE_T *ph);
void PDC_PrintCfg(PD_CFG_T *ph);
void PDC_PrintData(PD_DATA_T *ph);
void PDC_PrintInterface(PD_HANDLE_T *ph);
#endif

#ifdef __cplusplus
}
#endif

#endif