/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/simd_functions.h"

#include "xvc_common_lib/inter_prediction.h"
#include "xvc_common_lib/simd/inter_prediction_simd.h"

namespace xvc {

SimdFunctions::SimdFunctions(const std::set<CpuCapability> &capabilities) {
  InterPrediction::RegisterSimdFunctions(this);
  OverrideNeon(capabilities);
  OverrideX86(capabilities);
  OverrideMips(capabilities);
}

void SimdFunctions::OverrideNeon(const std::set<CpuCapability> &caps) {
#if XVC_HAVE_NEON
  if (caps.find(CpuCapability::kNeon) != caps.end()) {
    inter_prediction.add_avg[1] = &simd::AddAvgNeon;
  }
#endif  // XVC_HAVE_NEON
}

void SimdFunctions::OverrideX86(const std::set<CpuCapability> &caps) {
#if XVC_ARCH_X86
  if (caps.find(CpuCapability::kSse2) != caps.end()) {
    inter_prediction.add_avg[1] = &simd::AddAvgSse2;
  }
#endif  // XVC_ARCH_X86
}

void SimdFunctions::OverrideMips(const std::set<CpuCapability> &caps) {
#if XVC_ARCH_MIPS
#endif  // XVC_ARCH_MIPS
}

}   // namespace xvc
