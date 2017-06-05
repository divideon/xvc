/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include <stdint.h>

#include "xvc_common_lib/simd_cpu.h"

namespace xvc {

std::set<CpuCapability> SimdCpu::GetMaskedCaps(uint32_t mask) {
  const auto caps = GetRuntimeCapabilities();
  std::set<CpuCapability> out;
  for (auto it = caps.begin(); it != caps.end(); ++it) {
    CpuCapability cap = *it;
    if (((1 << static_cast<int>(cap)) & mask) != 0) {
      out.insert(cap);
    }
  }
  return out;
}

std::set<CpuCapability> SimdCpu::GetRuntimeCapabilities() {
  return{};
}

}   // namespace xvc
