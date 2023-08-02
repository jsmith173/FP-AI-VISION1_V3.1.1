/**
 ******************************************************************************
 * @file    fp_vision_preproc.h
 * @author  MCD Application Team
 * @brief   Header for fp_vision_preproc.c module
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
#ifndef __FP_VISION_PREPROC_H
#define __FP_VISION_PREPROC_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "fp_vision_global.h"
#include "stm32ipl.h"

/* Exported types ------------------------------------------------------------*/
typedef struct
{
uint16_t x;
uint16_t y;
uint32_t rowStride;
}Dma2dCfg_TypeDef;

typedef struct
{
 Dma2dCfg_TypeDef Dma2dcfg;
 uint32_t red_blue_swap;
 image_t Pfc_Src_Img;
 image_t Pfc_Dst_Img;
 image_t Resize_Src_Img;
 image_t Resize_Dst_Img;
 void*    AppCtxPtr;
}PreprocContext_TypeDef;
  

/* Exported constants --------------------------------------------------------*/
/* External variables --------------------------------------------------------*/
extern PreprocContext_TypeDef Preproc_Context;

/*******************/
/****PFC defines****/
/*******************/
/*The Pixel Format Conversion method, PIXEL_FMT_CONV, is configured in the preprocessor project's option:
* 1: HW_PFC : PFC performed by mean of HW engine like DMA2D
* 2: SW_PFC : PFC is performed by mean of SW routine and LUT
*/
#define HW_PFC 1
#define SW_PFC 2

/* Exported functions ------------------------------------------------------- */
void PREPROC_ImageResize(PreprocContext_TypeDef*);
void PREPROC_PixelFormatConversion(PreprocContext_TypeDef*);
void PREPROC_Pixel_RB_Swap(void *, void *, uint32_t );
void PREPROC_Init(PreprocContext_TypeDef * );

#ifdef __cplusplus
}
#endif

#endif /*__FP_VISION_PREPROC_H */

