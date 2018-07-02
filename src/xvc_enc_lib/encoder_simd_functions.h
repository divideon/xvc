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

#ifndef XVC_ENC_LIB_ENCODER_SIMD_FUNCTIONS_H_
#define XVC_ENC_LIB_ENCODER_SIMD_FUNCTIONS_H_

#include <set>

#include "xvc_common_lib/simd_cpu.h"
#include "xvc_enc_lib/sample_metric.h"
#include "xvc_common_lib/simd_functions.h"

namespace xvc {

struct EncoderSimdFunctions : public SimdFunctions {
  explicit EncoderSimdFunctions(const std::set<CpuCapability> &capabilities,
                                int internal_bitdepth);

  SampleMetric::SimdFunc sample_metric;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_ENCODER_SIMD_FUNCTIONS_H_
