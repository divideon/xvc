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

#include "xvc_enc_lib/simd/sample_metric_simd.h"

#if XVC_ARCH_X86
#include <emmintrin.h>
#endif
#if XVC_HAVE_NEON
#include <arm_neon.h>
#endif

#include <type_traits>

#include "xvc_enc_lib/encoder_simd_functions.h"
#include "xvc_enc_lib/sample_metric.h"

namespace xvc {
namespace simd {

#if XVC_ARCH_ARM
void SampleMetricSimd::Register(const std::set<CpuCapability> &caps,
                                xvc::EncoderSimdFunctions *simd_functions) {
#if XVC_HAVE_NEON
#endif  // XVC_HAVE_NEON
}
#endif  // XVC_ARCH_ARM

#if XVC_ARCH_X86
void SampleMetricSimd::Register(const std::set<CpuCapability> &caps,
                                xvc::EncoderSimdFunctions *simd_functions) {
}
#endif  // XVC_ARCH_X86

#if XVC_ARCH_MIPS
void SampleMetricSimd::Register(const std::set<CpuCapability> &caps,
                                xvc::EncoderSimdFunctions *simd_functions) {
}
#endif  // XVC_ARCH_MIPS

}   // namespace simd
}   // namespace xvc
