/**
  ******************************************************************************
  * @file    ai_utilities.c
  * @author  MCD Application Team
  * @brief   Module containing optimized functions
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

/* Includes ------------------------------------------------------------------*/
#include "ai_utilities.h"

/** @addtogroup Middlewares
  * @{
  */

/** @addtogroup STM32_AI_Utilities
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Functions Definition ------------------------------------------------------*/
/**
* @brief Initializes the application data memory layout
* @param  Pointer to Application context
*/
void Init_DataMemoryLayout(AppContext_TypeDef *App_Context_Ptr)
{
#ifdef OBJECTDETECT
  PeopleContext_TypeDef *AIContext = App_Context_Ptr->People_ContextPtr;
#else
  AiContext_TypeDef *AIContext = App_Context_Ptr->Ai_ContextPtr;
#endif /* OBJECTDETECT */

#if MEMORY_SCHEME == FULL_INTERNAL_MEM_OPT
  /* Bottom align camera_frame_buff buffer and resize_output_buff buffer */
  #define  RESIZE_OUTPUT_BUFFER_OFFSET (CAM_FRAME_BUFFER_SIZE - RESIZE_OUTPUT_BUFFER_SIZE)
  App_Context_Ptr->Camera_ContextPtr->camera_capture_buffer = ai_fp_global_memory;
  App_Context_Ptr->Camera_ContextPtr->camera_frame_buffer = ai_fp_global_memory;
  App_Context_Ptr->Preproc_ContextPtr->Pfc_Dst_Img.data = ai_fp_global_memory;
  App_Context_Ptr->Preproc_ContextPtr->Resize_Dst_Img.data = ai_fp_global_memory + RESIZE_OUTPUT_BUFFER_OFFSET;
  AIContext->activation_buffer = ai_fp_global_memory;
  #ifdef AI_NETWORK_INPUTS_IN_ACTIVATIONS
  AIContext->nn_input_buffer = NULL;
  #else
  AIContext->nn_input_buffer = ai_fp_global_memory + AI_ACTIVATION_BUFFER_SIZE;
  #endif /* AI_NETWORK_INPUTS_IN_ACTIVATIONS */
#elif MEMORY_SCHEME != FULL_INTERNAL_MEM_OPT
  App_Context_Ptr->Camera_ContextPtr->camera_capture_buffer = ai_fp_global_memory;
  App_Context_Ptr->Camera_ContextPtr->camera_frame_buffer = ai_fp_global_memory + CAM_FRAME_BUFFER_SIZE;
  #if MEMORY_SCHEME == SPLIT_INT_EXT
  /* Bottom align pfc_output_buff buffer and resize_output_buff buffer */
  #define  RESIZE_OUTPUT_BUFFER_OFFSET (PFC_OUTPUT_BUFFER_SIZE - RESIZE_OUTPUT_BUFFER_SIZE)
  App_Context_Ptr->Preproc_ContextPtr->Pfc_Dst_Img.data = ai_fp_activation_memory;
  App_Context_Ptr->Preproc_ContextPtr->Resize_Dst_Img.data = ai_fp_activation_memory + RESIZE_OUTPUT_BUFFER_OFFSET;
  AIContext->activation_buffer = ai_fp_activation_memory;
  #ifdef AI_NETWORK_INPUTS_IN_ACTIVATIONS
  AIContext->nn_input_buffer = NULL;
  #else
  AIContext->nn_input_buffer = ai_fp_global_memory + CAM_FRAME_BUFFER_SIZE;
  #endif /* AI_NETWORK_INPUTS_IN_ACTIVATIONS */
  #else /* MEMORY_SCHEME == FULL_EXTERNAL */
  /* Bottom camera_frame_buff buffer and resize_output_buff buffer */
  #define  RESIZE_OUTPUT_BUFFER_OFFSET (CAM_FRAME_BUFFER_SIZE - RESIZE_OUTPUT_BUFFER_SIZE)
  App_Context_Ptr->Preproc_ContextPtr->Pfc_Dst_Img.data = ai_fp_global_memory + CAM_FRAME_BUFFER_SIZE;
  App_Context_Ptr->Preproc_ContextPtr->Resize_Dst_Img.data = ai_fp_global_memory + CAM_FRAME_BUFFER_SIZE + RESIZE_OUTPUT_BUFFER_OFFSET;
  AIContext->activation_buffer = ai_fp_global_memory + CAM_FRAME_BUFFER_SIZE;
  #ifdef AI_NETWORK_INPUTS_IN_ACTIVATIONS
  AIContext->nn_input_buffer = NULL;
  #else
  AIContext->nn_input_buffer = ai_fp_global_memory + CAM_FRAME_BUFFER_SIZE + AI_ACTIVATION_BUFFER_SIZE;
  #endif /* AI_NETWORK_INPUTS_IN_ACTIVATIONS */
  #endif /* MEMORY_SCHEME == SPLIT_INT_EXT */
#else
  #error "Please check definition of MEMORY_SCHEME define"
#endif /* MEMORY_SCHEME == FULL_INTERNAL_MEM_OPT */
}

/**
* @brief  Run preprocessing stages on captured frame
* @param  App context ptr
* @retval None
*/
void Run_Preprocessing(AppContext_TypeDef *App_Context_Ptr)
{
  uint32_t tpfc_start;
  uint32_t tpfc_stop;
  uint32_t tresize_start;
  uint32_t tresize_stop;
  uint32_t tpvc_start;
  uint32_t tpvc_stop;
  TestRunContext_TypeDef* TestRunCtxt_Ptr=&App_Context_Ptr->Test_ContextPtr->TestRunContext;
  PreprocContext_TypeDef* PreprocCtxt_Ptr=App_Context_Ptr->Preproc_ContextPtr;

#if MEMORY_SCHEME == FULL_INTERNAL_MEM_OPT   
  
  if(App_Context_Ptr->Operating_Mode != VALID)
  {
    /*********************************************************************************************/
    /****Coherency purpose: invalidate the source buffer area in L1 D-Cache before CPU reading****/
    /*********************************************************************************************/
    UTILS_DCache_Coherency_Maintenance((void *)(App_Context_Ptr->Camera_ContextPtr->camera_frame_buffer), CAM_FRAME_BUFFER_SIZE, INVALIDATE);
  }
#endif
  
  TestRunCtxt_Ptr->src_buff_addr=(void *)(App_Context_Ptr->Camera_ContextPtr->camera_frame_buffer);
  TestRunCtxt_Ptr->src_buff_name=Test_buffer_names[0];
  TestRunCtxt_Ptr->src_width_size=CAM_RES_WIDTH;
  TestRunCtxt_Ptr->src_height_size=CAM_RES_HEIGHT;
  TestRunCtxt_Ptr->src_bpp=IMAGE_BPP_RGB565;
  TestRunCtxt_Ptr->src_size=CAM_FRAME_BUFFER_SIZE;
  TestRunCtxt_Ptr->PerformCapture=1;
  TestRunCtxt_Ptr->DumpFormat=DATA_FORMAT_BMP;
  TestRunCtxt_Ptr->rb_swap=0;
  TEST_Run(App_Context_Ptr->Test_ContextPtr, App_Context_Ptr->Operating_Mode);
  
  tresize_start=UTILS_GetTimeStamp(App_Context_Ptr->Utils_ContextPtr);
  
  /**********************/
  /****Image resizing****/
  /**********************/
  PreprocCtxt_Ptr->Resize_Src_Img.data=App_Context_Ptr->Camera_ContextPtr->camera_frame_buffer;
  PreprocCtxt_Ptr->Resize_Src_Img.w=CAM_RES_WIDTH;
  PreprocCtxt_Ptr->Resize_Src_Img.h=CAM_RES_HEIGHT;
  PreprocCtxt_Ptr->Resize_Src_Img.bpp=IMAGE_BPP_RGB565;
  PreprocCtxt_Ptr->Resize_Dst_Img.data=App_Context_Ptr->Preproc_ContextPtr->Resize_Dst_Img.data;
  PreprocCtxt_Ptr->Resize_Dst_Img.w=AI_NETWORK_WIDTH;
  PreprocCtxt_Ptr->Resize_Dst_Img.h=AI_NETWORK_HEIGHT;
  PreprocCtxt_Ptr->Resize_Dst_Img.bpp=IMAGE_BPP_RGB565;
  PREPROC_ImageResize(App_Context_Ptr->Preproc_ContextPtr);
  
  tresize_stop=UTILS_GetTimeStamp(App_Context_Ptr->Utils_ContextPtr);
  
#if PIXEL_FMT_CONV == HW_PFC
  /******************************************************************************************/
  /****Coherency purpose: clean the source buffer area in L1 D-Cache before DMA2D reading****/
  /******************************************************************************************/
  UTILS_DCache_Coherency_Maintenance((void *)(App_Context_Ptr->Preproc_ContextPtr->Resize_Dst_Img.data), RESIZE_OUTPUT_BUFFER_SIZE, CLEAN);
#endif
  
  TestRunCtxt_Ptr->src_buff_addr=(void *)(App_Context_Ptr->Preproc_ContextPtr->Resize_Dst_Img.data);
  TestRunCtxt_Ptr->src_buff_name=Test_buffer_names[1];
  TestRunCtxt_Ptr->src_width_size=AI_NETWORK_WIDTH;
  TestRunCtxt_Ptr->src_height_size=AI_NETWORK_HEIGHT;
  TestRunCtxt_Ptr->src_bpp=IMAGE_BPP_RGB565;
  TestRunCtxt_Ptr->src_size=RESIZE_OUTPUT_BUFFER_SIZE;
  TestRunCtxt_Ptr->PerformCapture=0;
  TestRunCtxt_Ptr->DumpFormat=DATA_FORMAT_BMP;
  TestRunCtxt_Ptr->rb_swap=0;
  TEST_Run(App_Context_Ptr->Test_ContextPtr, App_Context_Ptr->Operating_Mode);
  
  tpfc_start=UTILS_GetTimeStamp(App_Context_Ptr->Utils_ContextPtr);
  
  /*************************************/
  /****Image Pixel Format Conversion****/
  /*************************************/
  PreprocCtxt_Ptr->Pfc_Src_Img.data=App_Context_Ptr->Preproc_ContextPtr->Resize_Dst_Img.data;
  PreprocCtxt_Ptr->Pfc_Src_Img.w=AI_NETWORK_WIDTH;
  PreprocCtxt_Ptr->Pfc_Src_Img.h=AI_NETWORK_HEIGHT;
  PreprocCtxt_Ptr->Pfc_Src_Img.bpp=IMAGE_BPP_RGB565;
  PreprocCtxt_Ptr->Pfc_Dst_Img.data=App_Context_Ptr->Preproc_ContextPtr->Pfc_Dst_Img.data;
  PreprocCtxt_Ptr->Pfc_Dst_Img.w=AI_NETWORK_WIDTH;
  PreprocCtxt_Ptr->Pfc_Dst_Img.h=AI_NETWORK_HEIGHT;
  PreprocCtxt_Ptr->Dma2dcfg.x=0;
  PreprocCtxt_Ptr->Dma2dcfg.y=0;
  PreprocCtxt_Ptr->Dma2dcfg.rowStride=AI_NETWORK_WIDTH;
  PreprocCtxt_Ptr->red_blue_swap=1;
  PREPROC_PixelFormatConversion(App_Context_Ptr->Preproc_ContextPtr);
  
  tpfc_stop=UTILS_GetTimeStamp(App_Context_Ptr->Utils_ContextPtr);
  
#if PIXEL_FMT_CONV == HW_PFC 
  /**************************************************************************************/
  /****Coherency purpose: invalidate the source area in L1 D-Cache before CPU reading****/  
  /**************************************************************************************/
  UTILS_DCache_Coherency_Maintenance((void *)(App_Context_Ptr->Preproc_ContextPtr->Pfc_Dst_Img.data), 
                                     PFC_OUTPUT_BUFFER_SIZE, 
                                     INVALIDATE);
#endif
  
  TestRunCtxt_Ptr->src_buff_addr=(void *)(App_Context_Ptr->Preproc_ContextPtr->Pfc_Dst_Img.data);
  TestRunCtxt_Ptr->src_buff_name=Test_buffer_names[2];
  TestRunCtxt_Ptr->src_width_size=AI_NETWORK_WIDTH;
  TestRunCtxt_Ptr->src_height_size=AI_NETWORK_HEIGHT;
  TestRunCtxt_Ptr->src_bpp=App_Context_Ptr->Preproc_ContextPtr->Pfc_Dst_Img.bpp;
  TestRunCtxt_Ptr->src_size=PFC_OUTPUT_BUFFER_SIZE;
  TestRunCtxt_Ptr->PerformCapture=0;
  TestRunCtxt_Ptr->DumpFormat=DATA_FORMAT_BMP;
  TestRunCtxt_Ptr->rb_swap=1;
  TEST_Run(App_Context_Ptr->Test_ContextPtr, App_Context_Ptr->Operating_Mode);

  tpvc_start=UTILS_GetTimeStamp(App_Context_Ptr->Utils_ContextPtr);
  
  /***********************************************************/
  /*********Pixel value convertion and normalisation**********/
  /***********************************************************/
#ifdef OBJECTDETECT
  AI_PixelValueConversion(App_Context_Ptr->People_ContextPtr,
                                      (void*)(App_Context_Ptr->Preproc_ContextPtr->Pfc_Dst_Img.data));
#else
  AI_PixelValueConversion(App_Context_Ptr->Ai_ContextPtr,
                                      (void*)(App_Context_Ptr->Preproc_ContextPtr->Pfc_Dst_Img.data));
#endif /* OBJECTDETECT */

  tpvc_stop=UTILS_GetTimeStamp(App_Context_Ptr->Utils_ContextPtr);
  
  TestRunCtxt_Ptr->src_buff_addr=(void *)(NULL);
  TestRunCtxt_Ptr->src_buff_name="";
  TestRunCtxt_Ptr->src_width_size=0;
  TestRunCtxt_Ptr->src_height_size=0;
  TestRunCtxt_Ptr->src_size=0;
  TestRunCtxt_Ptr->PerformCapture=0;
  TestRunCtxt_Ptr->DumpFormat=DATA_FORMAT_RAW;
  TestRunCtxt_Ptr->rb_swap=0;
  TEST_Run(App_Context_Ptr->Test_ContextPtr, App_Context_Ptr->Operating_Mode);
  
  App_Context_Ptr->Utils_ContextPtr->ExecTimingContext.operation_exec_time[FRAME_PFC]=tpfc_stop-tpfc_start;
  App_Context_Ptr->Utils_ContextPtr->ExecTimingContext.operation_exec_time[FRAME_RESIZE]=tresize_stop-tresize_start;
  App_Context_Ptr->Utils_ContextPtr->ExecTimingContext.operation_exec_time[FRAME_PVC]=tpvc_stop-tpvc_start;
}

bool Image_CheckResizeMemoryLayout(image_t *src_img, image_t *dst_img)
{
  uint32_t src_size = STM32Ipl_ImageDataSize(src_img);
  uint32_t dst_size = STM32Ipl_ImageDataSize(dst_img);
  uint32_t src_start = (uint32_t)src_img->data;
  uint32_t dst_start = (uint32_t)dst_img->data;
  uint32_t src_end = src_start + src_size - 1;
  uint32_t dst_end = dst_start + dst_size - 1;
  bool reverse;

  if ((src_end > dst_start) && (dst_end >= src_end))
  {
    reverse = true;
  }
  else
  {
    reverse = false; /* ((src_start >= dst_start) || (src_end <= dst_start)) */
  }

  return reverse;
}

bool Image_CheckPfcMemoryLayout(image_t *src_img, image_t *dst_img)
{
  uint32_t src_size = STM32Ipl_ImageDataSize(src_img);
  uint32_t dst_size = STM32Ipl_ImageDataSize(dst_img);
  uint32_t src_start = (uint32_t)src_img->data;
  uint32_t dst_start = (uint32_t)dst_img->data;
  uint32_t src_end = src_start + src_size - 1;
  uint32_t dst_end = dst_start + dst_size - 1;

  if (((dst_size - src_size) <  dst_size / 3))
  {
    while (1);
  }
  else
  {
    if (dst_end <= src_end)
    {
      return true;
    }
    else if (dst_start >= src_start)
    {
      return false;
    }
    else
    {
      while (1);
    }
  }
}