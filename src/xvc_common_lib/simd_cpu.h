/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_SIMD_CPU_H_
#define XVC_COMMON_LIB_SIMD_CPU_H_

#include <set>

namespace xvc {

enum class CpuCapability {
  kNeon = 1,
  kMmx,
  kSse,
  kSse2,
  kSse3,
  kSsse3,
  kSse4_1,
  kSse4_2,
  kAvx,
  kAvx2,

  kTotalNumber
};

struct SimdCpu {
  static std::set<CpuCapability> GetMaskedCaps(uint32_t mask);
  static std::set<CpuCapability> GetRuntimeCapabilities();
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_SIMD_CPU_H_
