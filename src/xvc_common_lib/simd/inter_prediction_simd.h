/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_SIMD_INTER_PREDICTION_SIMD_H_
#define XVC_COMMON_LIB_SIMD_INTER_PREDICTION_SIMD_H_

#include "xvc_common_lib/common.h"

namespace xvc {
namespace simd {

#if XVC_ARCH_X86
void AddAvgSse2(int width, int height, int offset, int shift, int bitdepth,
                const int16_t *src1, intptr_t stride1,
                const int16_t *src2, intptr_t stride2,
                Sample *dst, intptr_t dst_stride);
#endif

#if XVC_HAVE_NEON
void AddAvgNeon(int width, int height, int offset, int shift, int bitdepth,
                const int16_t *src1, intptr_t stride1,
                const int16_t *src2, intptr_t stride2,
                Sample *dst, intptr_t dst_stride);
#endif

}   // namespace simd
}   // namespace xvc

#endif  // XVC_COMMON_LIB_SIMD_INTER_PREDICTION_SIMD_H_
