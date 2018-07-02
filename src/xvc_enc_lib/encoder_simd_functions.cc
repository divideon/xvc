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

#include "xvc_enc_lib/encoder_simd_functions.h"

#if defined(XVC_ARCH_ARM) || defined(XVC_ARCH_X86) || defined(XVC_ARCH_MIPS)
#include "xvc_enc_lib/simd/sample_metric_simd.h"
#endif

namespace xvc {

EncoderSimdFunctions::EncoderSimdFunctions(const std::set<CpuCapability> &caps,
                                           int internal_bitdepth)
  : SimdFunctions(caps),
  sample_metric() {
#if defined(XVC_ARCH_ARM) || defined(XVC_ARCH_X86) || defined(XVC_ARCH_MIPS)
  simd::SampleMetricSimd::Register(caps, internal_bitdepth, this);
#endif
}

}   // namespace xvc
