/**
  ******************************************************************************
  * @file    network.h
  * @author  STEdgeAI
  * @date    2025-05-30 14:26:58
  * @brief   Minimal description of the generated c-implemention of the network
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */
#ifndef LL_ATON_DEFAULT_H
#define LL_ATON_DEFAULT_H

/******************************************************************************/
#define LL_ATON_DEFAULT_C_MODEL_NAME        "Default"
#define LL_ATON_DEFAULT_ORIGIN_MODEL_NAME   "fall_guard_st_ssd_mobilenet_v1_256v2"

/************************** USER ALLOCATED IOs ********************************/
#define LL_ATON_DEFAULT_USER_ALLOCATED_INPUTS   (1)  // Number of input buffers not allocated by the compiler
#define LL_ATON_DEFAULT_USER_ALLOCATED_OUTPUTS  (3)  // Number of output buffers not allocated by the compiler

/************************** INPUTS ********************************************/
#define LL_ATON_DEFAULT_IN_NUM        (1)    // Total number of input buffers
// Input buffer 1 -- Input_2_out_0
#define LL_ATON_DEFAULT_IN_1_ALIGNMENT   (32)
#define LL_ATON_DEFAULT_IN_1_SIZE_BYTES  (196608)

/************************** OUTPUTS *******************************************/
#define LL_ATON_DEFAULT_OUT_NUM        (3)    // Total number of output buffers
// Output buffer 1 -- Transpose_285_out_0
#define LL_ATON_DEFAULT_OUT_1_ALIGNMENT   (32)
#define LL_ATON_DEFAULT_OUT_1_SIZE_BYTES  (81900)
// Output buffer 2 -- Transpose_296_out_0
#define LL_ATON_DEFAULT_OUT_2_ALIGNMENT   (32)
#define LL_ATON_DEFAULT_OUT_2_SIZE_BYTES  (109200)
// Output buffer 3 -- Identity_1_out_0
#define LL_ATON_DEFAULT_OUT_3_ALIGNMENT   (32)
#define LL_ATON_DEFAULT_OUT_3_SIZE_BYTES  (109200)

#endif /* LL_ATON_DEFAULT_H */
