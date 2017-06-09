/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/simd/inter_prediction_simd.h"

#if XVC_ARCH_X86
#include <emmintrin.h>
#endif
#if XVC_HAVE_NEON
#include <arm_neon.h>
#endif

namespace xvc {
namespace simd {

#if XVC_ARCH_X86
void AddAvgSse2(int width, int height, int offset, int shift, int bitdepth,
                const int16_t *src1, intptr_t stride1,
                const int16_t *src2, intptr_t stride2,
                Sample *dst, intptr_t dst_stride) {
#if XVC_HIGH_BITDEPTH
  __m128i min = _mm_set1_epi16(0);
  __m128i max = _mm_set1_epi16((1 << bitdepth) - 1);
#endif
  __m128i s3 = _mm_set1_epi16((int16_t)offset);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x += 8) {
      __m128i s1 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(src1 + x));
      __m128i s2 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(src2 + x));
      __m128i sum =
        _mm_srai_epi16(_mm_adds_epi16(_mm_add_epi16(s1, s2), s3), shift);
#if XVC_HIGH_BITDEPTH
      __m128i out = _mm_max_epi16(min, _mm_min_epi16(sum, max));
      _mm_storeu_si128(reinterpret_cast<__m128i*>(dst + x), out);
#else
      __m128i out = _mm_packus_epi16(sum, _mm_setzero_si128());
      _mm_storel_epi64(reinterpret_cast<__m128i*>(dst + x), out);
#endif
    }
    src1 += stride1;
    src2 += stride2;
    dst += dst_stride;
  }
}
#endif  // XVC_ARCH_X86

#if XVC_HAVE_NEON
void AddAvgNeon(int width, int height, int offset, int shift, int bitdepth,
                const int16_t *src1, intptr_t stride1,
                const int16_t *src2, intptr_t stride2,
                Sample *dst, intptr_t dst_stride) {
  int16x8_t vec_shift = vdupq_n_s16((int16_t)shift * -1);
#if XVC_HIGH_BITDEPTH
  int16x8_t min = vdupq_n_s16(0);
  int16x8_t max = vdupq_n_s16((1 << bitdepth) - 1);
#endif
  int16x8_t s3 = vdupq_n_s16((int16_t)offset);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x += 8) {
      int16x8_t s1 = vld1q_s16(src1 + x);
      int16x8_t s2 = vld1q_s16(src2 + x);
      int16x8_t sum = vshlq_s16(vqaddq_s16(vaddq_s16(s1, s2), s3), vec_shift);
#if XVC_HIGH_BITDEPTH
      int16x8_t out = vmaxq_s16(min, vminq_s16(sum, max));
      vst1q_u16(dst + x, vreinterpretq_u16_s16(out));
#else
      uint8x8_t out = vqmovun_s16(sum);
      vst1_u8(dst + x, out);
#endif
    }
    src1 += stride1;
    src2 += stride2;
    dst += dst_stride;
  }
}
#endif  // XVC_HAVE_NEON

}   // namespace simd
}   // namespace xvc
