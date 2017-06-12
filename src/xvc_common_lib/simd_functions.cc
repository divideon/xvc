/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/simd_functions.h"

#include "xvc_common_lib/inter_prediction.h"
#include "xvc_common_lib/simd/inter_prediction_simd.h"

namespace xvc {

SimdFunctions::SimdFunctions(const std::set<CpuCapability> &capabilities) {
  InterPrediction::RegisterDefaultFunctions(this);
  simd::InterPredictionSimd::Register(capabilities, this);
}

}   // namespace xvc
