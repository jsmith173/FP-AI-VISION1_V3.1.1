/**
 ******************************************************************************
 * @file    fp_vision_utils.h
 * @author  MCD Application Team
 * @brief   Header for fp_vision_utils.c module
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FP_VISION_UTILS_H
#define __FP_VISION_UTILS_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "fp_vision_global.h"
#include "camera.h"

typedef enum
{
  INVALIDATE           = 0x01,   
  CLEAN                = 0x02  
}DCache_Coherency_TypeDef;

typedef struct
{
  uint32_t operation_exec_time[APP_FRAMEOPERATION_NUM];
  uint32_t Tfps;
  uint32_t tcapturestart1;
  uint32_t tcapturestart2; 
  uint32_t tcapturestart; 
  uint32_t tcapturestop; 
}ExecTimingContext_TypeDef;

typedef struct
{
  JOYPin_TypeDef joyState; 
  int32_t camContrast;   /* Camera brightness. */
  int32_t camBrightness;/* Camera contrast. */
  int32_t lcdBrightness; /* LCD brightness. */
} JoystickContext_TypeDef;

typedef struct
{
  ExecTimingContext_TypeDef ExecTimingContext;
  JoystickContext_TypeDef   JoystickContext;
  void* AppCtxPtr;
} UtilsContext_TypeDef;

/* Exported constants --------------------------------------------------------*/
#include "fp_vision_app.h"

/* External variables --------------------------------------------------------*/
extern UtilsContext_TypeDef UtilsContext;

/* Exported functions ------------------------------------------------------- */
void UTILS_Init(UtilsContext_TypeDef *);
void UTILS_Joystick_Check(UtilsContext_TypeDef *);
void UTILS_Dma2d_Memcpy(uint32_t *pSrc, uint32_t *pDst, uint16_t x, uint16_t y, uint16_t xsize, uint16_t ysize,
                     uint32_t rowStride, uint32_t input_color_format, uint32_t output_color_format, int pfc,
                     int red_blue_swap);
void UTILS_DCache_Coherency_Maintenance(uint32_t *, int32_t , DCache_Coherency_TypeDef );
void UTILS_Bubblesort(float *, int *, int );
void UTILS_Compute_ExecutionTiming(UtilsContext_TypeDef *);
uint32_t UTILS_GetTimeStamp(UtilsContext_TypeDef *);

#ifdef __cplusplus
}
#endif

#endif /*__FP_VISION_UTILS_H */

