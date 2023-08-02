/**
 ******************************************************************************
 * @file    ai_utilities.h
 * @author  MCD Application Team
 * @brief   Header for ai_utilities.c module
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
#ifndef __AI_UTILITIES_H
#define __AI_UTILITIES_H

#ifdef __cplusplus
extern "C"
{
#endif
  
/* Includes ------------------------------------------------------------------*/
#include "fp_vision_app.h"
#include "img_preprocess.h"
#include "ai_interface.h"

void Run_Preprocessing(AppContext_TypeDef *);
void Init_PeopleDataMemoryLayout(AppContext_TypeDef *);
void Resize_Frame(Image_TypeDef *, Image_TypeDef *, Roi_TypeDef *);

#ifdef __cplusplus
}
#endif

#endif /*__AI_UTILITIES_H */

