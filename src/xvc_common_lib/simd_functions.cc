/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/simd_functions.h"

#if defined(XVC_ARCH_ARM) || defined(XVC_ARCH_X86) || defined(XVC_ARCH_MIPS)
#include "xvc_common_lib/simd/inter_prediction_simd.h"
#endif

namespace xvc {

SimdFunctions::SimdFunctions(const std::set<CpuCapability> &capabilities)
  : inter_prediction() {
#if defined(XVC_ARCH_ARM) || defined(XVC_ARCH_X86) || defined(XVC_ARCH_MIPS)
  simd::InterPredictionSimd::Register(capabilities, this);
#endif
}

}   // namespace xvc
