/**
 ******************************************************************************
 * @file    fp_vision_camera.h
 * @author  MCD Application Team
 * @brief   Library to manage camera related operation
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

#ifndef __FP_VISION_CAMERA_H
#define __FP_VISION_CAMERA_H

#ifdef __cplusplus
extern "C"
{
#endif
  
/* Includes ------------------------------------------------------------------*/
#include "fp_vision_global.h"
#include "stm32h747i_discovery_camera.h"
#include "ov9655_reg.h"
  
typedef struct
{
  uint8_t* camera_capture_buffer;
  uint8_t* camera_frame_buffer;
  uint32_t vsync_it; 
  uint32_t Tframe_evt;
  uint32_t Tvsync_evt;
  volatile uint8_t new_frame_ready;
  void* AppCtxPtr;
  uint32_t mirror_flip;
} CameraContext_TypeDef;
 
#include "fp_vision_app.h"

/****************************/
/***CAMERA related defines***/
/****************************/
  /*The camera resolution, CAMERA_CAPTURE_RES, is configured in the preprocessor project's option:
  * 1: VGA_640_480_RES
  * 2: QVGA_320_240_RES
  */
#define VGA_640_480_RES 1
#define QVGA_320_240_RES 2
  
  
#if CAMERA_CAPTURE_RES == VGA_640_480_RES
 #define CAMERA_RESOLUTION CAMERA_R640x480
 #define CAM_RES_WIDTH 640
 #define CAM_RES_HEIGHT 480
#elif CAMERA_CAPTURE_RES == QVGA_320_240_RES
 #define CAMERA_RESOLUTION CAMERA_R320x240
 #define CAM_RES_WIDTH 320
 #define CAM_RES_HEIGHT 240
#else
 #error Please check definition of CAMERA_CAPTURE_RES define
#endif
  
extern CameraContext_TypeDef CameraContext;
  
/* Exported functions ------------------------------------------------------- */
void CAMERA_Context_Init(CameraContext_TypeDef *);
void CAMERA_Init(CameraContext_TypeDef* );
void CAMERA_Set_MirrorFlip(CameraContext_TypeDef* , uint32_t );
void CAMERA_Enable_TestBar_Mode(CameraContext_TypeDef*);
void CAMERA_Disable_TestBar_Mode(CameraContext_TypeDef*);
 
#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* __FP_VISION_CAMERA_H */

