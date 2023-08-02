/**
 ******************************************************************************
 * @file    fp_vision_global.h
 * @author  MCD Application Team
 * @brief   Header contianing global define used across different modules
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
#ifndef __FP_VISION_GLOBAL_H
#define __FP_VISION_GLOBAL_H

#ifdef __cplusplus
extern "C"
{
#endif
  
/* Includes ------------------------------------------------------------------*/
#include "stm32h747i_discovery.h"
#include "ai_interface.h"
  
//number of iteration per image in valid mode
#define NBOFVALID 1  
  
#define FULL_EXTERNAL 1
#define SPLIT_INT_EXT 2
#define FULL_INTERNAL_FPS_OPT 3 // used for max opt see compil option define for modifcation
#define FULL_INTERNAL_MEM_OPT 4
  
#define NN_TOP_N_DISPLAY 1
#define NN_TOP_N_DISPLAY_STRING_1 "Top-"
#define NN_TOP_N_DISPLAY_STRING_2 xstr(NN_TOP_N_DISPLAY)
#define NN_TOP_N_DISPLAY_MSG NN_TOP_N_DISPLAY_STRING_1 NN_TOP_N_DISPLAY_STRING_2
#define NN_OUTPUT_DISPLAY_REFRESH_RATE 1
  
#define NN_OUTPUT_CLASS_NUMBER AI_NET_OUTPUT_SIZE
  
typedef enum
{
  NOMINAL              = 0x01,    /* Nominal  */
  VALID                = 0x02,    /* Validation  */
  DUMP                 = 0x03,    /* Memory buffer dumping  */
  CAPTURE              = 0x04     /* Frame capture  */
}AppOperatingMode_TypeDef;
  
typedef enum 
{
  FRAME_CAPTURE    = 0x00,
  FRAME_RESIZE     = 0x01,
  FRAME_PFC        = 0x02,
  FRAME_PVC        = 0x03,
  FRAME_INFERENCE  = 0x04,
  
  APP_FRAMEOPERATION_NUM  
} AppFrameOperation_TypeDef;

typedef enum 
{
  CAM_BUFF        = 0x00,
  PFC_OUT_BUFF    = 0x01,
  RESIZE_OUT_BUFF = 0x02,
  NN_INPUT_BUFF   = 0x03,
  NN_OUTPUT_BUFF  = 0x04,
  
  APP_BUFF_NUM  
} AppBuffer_TypeDef;

extern void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /*__FP_VISION_GLOBAL_H */

