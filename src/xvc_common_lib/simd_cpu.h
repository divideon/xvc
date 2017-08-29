/******************************************************************************
* Copyright (C) 2017, Divideon.
*
* Redistribution and use in source and binary form, with or without
* modifications is permitted only under the terms and conditions set forward
* in the xvc License Agreement. For commercial redistribution and use, you are
* required to send a signed copy of the xvc License Agreement to Divideon.
*
* Redistribution and use in source and binary form is permitted free of charge
* for non-commercial purposes. See definition of non-commercial in the xvc
* License Agreement.
*
* All redistribution of source code must retain this copyright notice
* unmodified.
*
* The xvc License Agreement is available at https://xvc.io/license/.
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
