 /**
  ******************************************************************************
  * @file    ai_utilities.h
  * @author  MCD Application Team
  * @brief   Header for ai_utilities.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AI_UTILITIES_H
#define __AI_UTILITIES_H

#ifdef __cplusplus
extern "C"
{
#endif
  
/* Includes ------------------------------------------------------------------*/
#include "fp_vision_app.h"
#include "ai_interface.h"

void Run_Preprocessing(AppContext_TypeDef *);
void Init_DataMemoryLayout(AppContext_TypeDef *);
bool Image_CheckResizeMemoryLayout(image_t *src_img, image_t *dst_img);
bool Image_CheckPfcMemoryLayout(image_t *src_img, image_t *dst_img);

#ifdef __cplusplus
}
#endif

#endif /*__AI_UTILITIES_H */
