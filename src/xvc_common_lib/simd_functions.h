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
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_SIMD_FUNCTIONS_H_
