/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_SIMD_INTER_PREDICTION_SIMD_H_
#define XVC_COMMON_LIB_SIMD_INTER_PREDICTION_SIMD_H_

#include <set>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/simd_cpu.h"

namespace xvc {

struct SimdFunctions;

namespace simd {

struct InterPredictionSimd {
  static void Register(const std::set<CpuCapability> &caps,
                       xvc::SimdFunctions *simd);
};

}   // namespace simd
}   // namespace xvc

#endif  // XVC_COMMON_LIB_SIMD_INTER_PREDICTION_SIMD_H_
