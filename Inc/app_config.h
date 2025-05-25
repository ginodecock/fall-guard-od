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
#define ASPECT_RATIO_CROP       (0)
#define ASPECT_RATIO_FIT        (0)
#define ASPECT_RATIO_FULLSCREEN (0)
#define ASPECT_RATIO_MODE    ASPECT_RATIO_CROP

#define CAMERA_WIDTH 0
#define CAMERA_HEIGHT 0

/* Define display size */
#define LCD_BG_WIDTH 800
#define LCD_BG_HEIGHT 480
/* Delay display by DISPLAY_DELAY frame number */
#define DISPLAY_DELAY 1
#define CAPTURE_FORMAT DCMIPP_PIXEL_PACKER_FORMAT_RGB565_1
#define CAPTURE_BPP 2

/* Model Related Info */
#define POSTPROCESS_TYPE                          POSTPROCESS_OD_ST_SSD_UF
#define LCD_FG_WIDTH             800
#define LCD_FG_HEIGHT            480

#define LCD_FG_FRAMEBUFFER_SIZE  (LCD_FG_WIDTH * LCD_FG_HEIGHT * 2)
#define COLOR_BGR (0)
#define COLOR_RGB (1)
#define COLOR_MODE    COLOR_RGB

// Replace existing normalization with:
#define YOLOX_MEAN {0.485, 0.456, 0.406}
#define YOLOX_STD {0.229, 0.224, 0.225}

#define WELCOME_MSG_1     "fall_guard"
#define WELCOME_MSG_2       "G-DC"


#define NN_WIDTH 256
#define NN_HEIGHT 256
#define NN_BUFFER_OUT_SIZE 81900 //nieuw model
#define NN_FORMAT DCMIPP_PIXEL_PACKER_FORMAT_RGB888_YUV444_1
#define NN_BPP 3
#define NB_CLASSES 3
#define DECLARE_CLASSES_TABLE const char* classes_table[NB_CLASSES] = {\
 "background",   "person" , "fall" }
#endif
