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
