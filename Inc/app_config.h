/**
 ******************************************************************************
 * @file    app_config.h
 * @author  G-DC
 *
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 G-DC
 * All rights reserved.
 *
 * This software is provided AS-IS.
 *
 ******************************************************************************
 */
#ifndef APP_CONFIG
#define APP_CONFIG
#define USE_DCACHE

#include "postprocess_conf.h"

/* Define sensor orientation */
#define CAMERA_FLIP CMW_MIRRORFLIP_MIRROR

/* Define display size */
#define LCD_BG_WIDTH 800
#define LCD_BG_HEIGHT 480
/* Delay display by DISPLAY_DELAY frame number */
#define DISPLAY_DELAY 1

/* Model Related Info */
#define POSTPROCESS_TYPE                          POSTPROCESS_OD_ST_SSD_UF

#define AI_OBJDETECT_YOLOV2_PP_MAX_BOXES_LIMIT    (10)

#define NN_WIDTH 256
#define NN_HEIGHT 256
#define NN_BUFFER_OUT_SIZE 81900
 /* set number of outputs and define NN_OUTx_SIZE in bytes for x from 0 to (NN_OUT_NB - 1) */
#define NN_OUT_NB                                 3
#define NN_OUT0_SIZE                              (6825 * 3 * 4)
#define NN_OUT1_SIZE                              (6825 * 4 * 4)
#define NN_OUT2_SIZE                              (6825 * 4 * 4)

#define NN_FORMAT DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1
#define NN_BPP 3
#define NB_CLASSES 3
#define DECLARE_CLASSES_TABLE const char* classes_table[NB_CLASSES] = {\
 "background",   "person" , "fall" }


#endif
