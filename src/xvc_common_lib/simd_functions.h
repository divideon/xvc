/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_SIMD_FUNCTIONS_H_
#define XVC_COMMON_LIB_SIMD_FUNCTIONS_H_

#include <set>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/simd_cpu.h"

namespace xvc {

struct SimdFunctions {
  // 0: width < 8, 1: width >= 8
  static const int kW8 = 2;
  // 0: luma, 1: chroma
  static const int kLC = 2;

  explicit SimdFunctions(const std::set<CpuCapability> &capabilities);

  struct InterPredictionFunctions {
    void(*add_avg[kW8])(int width, int height, int offset, int shift,
                        int bitdepth, const int16_t *src1, intptr_t stride1,
                        const int16_t *src2, intptr_t stride2,
                        Sample *dst, intptr_t dst_stride);
    void(*filter_copy_bipred[kW8])(int width, int height,
                                   int16_t offset, int shift,
                                   const Sample *ref, ptrdiff_t ref_stride,
                                   int16_t *pred, ptrdiff_t pred_stride);
    void(*filter_h_sample_sample[kLC])(int width, int height, int bitdepth,
                                       const int16_t *filter,
                                       const Sample *src, ptrdiff_t src_stride,
                                       Sample *dst, ptrdiff_t dst_stride);
  } inter_prediction;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_SIMD_FUNCTIONS_H_
