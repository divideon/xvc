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
