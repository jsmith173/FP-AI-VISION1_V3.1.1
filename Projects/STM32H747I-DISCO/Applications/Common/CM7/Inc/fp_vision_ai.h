/**
 ******************************************************************************
 * @file    fp_vision_ai.h
 * @author  MCD Application Team
 * @brief   Header for fp_vision_ai.c module
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
#ifndef __FP_VISION_AI_H
#define __FP_VISION_AI_H

#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "fp_vision_global.h"
#include "ai_interface.h"

#ifdef OBJECTDETECT
#include "peopledetect_network_pp_if.h"
#endif

/* Private macros ------------------------------------------------------------*/
#define _MIN(x_, y_) \
  ( ((x_)<(y_)) ? (x_) : (y_) )

#define _MAX(x_, y_) \
    ( ((x_)>(y_)) ? (x_) : (y_) )

#define _CLAMP(x_, min_, max_, type_) \
      (type_) (_MIN(_MAX(x_, min_), max_))

#define _ROUND(v_, type_) \
        (type_) ( ((v_)<0) ? ((v_)-0.5f) : ((v_)+0.5f) )


/* Exported types ------------------------------------------------------------*/
typedef struct
{
  void* nn_output_buffer;
  void* nn_input_buffer;
  void* activation_buffer;
  uint8_t* lut;
  float    nn_input_norm_scale;
  int32_t  nn_input_norm_zp;
  void*    AppCtxPtr;
} AiContext_TypeDef;

#include "fp_vision_app.h"

#ifndef OBJECTDETECT
/* Exported constants --------------------------------------------------------*/
extern AiContext_TypeDef Ai_Context;

/* Exported functions ------------------------------------------------------- */
void AI_Deinit(void);
void AI_Run(AiContext_TypeDef* );
void AI_Init(AiContext_TypeDef*);
void AI_PixelValueConversion_QuantizedNN(AiContext_TypeDef* , uint8_t *);
void AI_PixelValueConversion_FloatNN(AiContext_TypeDef* , uint8_t *, uint32_t );
void AI_Output_Dequantize(AiContext_TypeDef* );
void AI_Softmax(AiContext_TypeDef* Ai_Context_Ptr);
void AI_PixelValueConversion(AiContext_TypeDef* , void *);
#endif /* !OBJECTDETECT */

#ifdef __cplusplus
}
#endif

#endif /*__FP_VISION_AI_H */

