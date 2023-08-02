/**
 ******************************************************************************
 * @file    libpeopledetect.h
 * @author  MCD Application Team
 * @brief   Header for libpeopledetect library
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
#ifndef __LIBPEOPLE_DETECT_H
#define __LIBPEOPLE_DETECT_H

#ifdef __cplusplus
extern "C"
{
#endif
  /* Includes ------------------------------------------------------------------*/
#include "ai_interface.h"
#define NN_OUTPUT_CLASS_NUMBER AI_NET_OUTPUT_SIZE
#include "peopledetect_network_pp_if.h"   
  
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
  uint32_t nn_width;
  uint32_t nn_height;
  uint32_t nn_channels;
  float    nn_input_norm_scale;
  int32_t  nn_input_norm_zp;
  void*    AppCtxPtr;
  
  network_pp_static_param_t input_static_param;
  network_pp_in_t  *pInput;
  network_pp_out_t output; 
  /* contains results of the NN from previous the iteration in VALID mode */
  network_pp_out_t previousVal; 
} PeopleContext_TypeDef;

/* Exported constants --------------------------------------------------------*/
extern PeopleContext_TypeDef People_Context;

/* Internal functions ------------------------------------------------------- */

void AI_Run(PeopleContext_TypeDef* );
void AI_PixelValueConversion_QuantizedNN(PeopleContext_TypeDef* , uint8_t *);
void AI_PixelValueConversion_FloatNN(PeopleContext_TypeDef* , uint8_t *, uint32_t );
void AI_Output_Dequantize(PeopleContext_TypeDef* );
void AI_Softmax(PeopleContext_TypeDef* People_Context_Ptr);
void AI_PixelValueConversion(PeopleContext_TypeDef* , void *);
void AI_NetworkInference(PeopleContext_TypeDef *);
void AI_PostProcess(PeopleContext_TypeDef *);
void AI_Ppnetwork(PeopleContext_TypeDef *);

/* Exported functions ------------------------------------------------------- */
void People_Init(PeopleContext_TypeDef*);
void People_Run(PeopleContext_TypeDef*);
void People_Deinit(void);

#ifdef __cplusplus
}
#endif
#endif /*__LIBPEOPLE_DETECT_H*/
