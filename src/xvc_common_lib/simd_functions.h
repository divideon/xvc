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
  explicit SimdFunctions(const std::set<CpuCapability> &capabilities);

  struct {
    // 0: width < 8, 1: width >= 8
    void(*add_avg[2])(int width, int height, int offset, int shift,
                      int bitdepth, const int16_t *src1, intptr_t stride1,
                      const int16_t *src2, intptr_t stride2,
                      Sample *dst, intptr_t dst_stride);
  } inter_prediction;

private:
  void OverrideNeon(const std::set<CpuCapability> &caps);
  void OverrideX86(const std::set<CpuCapability> &caps);
  void OverrideMips(const std::set<CpuCapability> &caps);
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_SIMD_FUNCTIONS_H_
