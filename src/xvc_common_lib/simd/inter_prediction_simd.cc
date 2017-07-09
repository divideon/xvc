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

#include <type_traits>

#include "xvc_common_lib/simd_functions.h"
#include "xvc_common_lib/inter_prediction.h"

#ifdef _MSC_VER
#define __attribute__(SPEC)
#endif

#ifdef XVC_ARCH_X86
// Formatting helper
#define CAST_M128_CONST(VAL) reinterpret_cast<const __m128i*>((VAL))
#endif

namespace xvc {
namespace simd {

constexpr int kBin8_01_00_11_10 = (1 << 6) | (0 << 4) | (3 << 2) | (2 << 0);
constexpr int kBin8_10_11_00_01 = (2 << 6) | (3 << 4) | (0 << 2) | (1 << 0);
constexpr int kBin8_11_01_10_00 = (3 << 6) | (1 << 4) | (2 << 2) | (0 << 0);

#if XVC_ARCH_X86 || XVC_HAVE_NEON
constexpr int Width8(int width) { return width & ~7; }
constexpr int Width4(int width) { return width & 7; }
#endif

#if XVC_ARCH_X86
__attribute__((target("sse2")))
static void AddAvgSse2(int width, int height,
                       int offset, int shift, int bitdepth,
                       const int16_t *src1, intptr_t stride1,
                       const int16_t *src2, intptr_t stride2,
                       Sample *dst, intptr_t dst_stride) {
  const int width8 = Width8(width);
  const __m128i voffset = _mm_set1_epi16((int16_t)offset);
#if XVC_HIGH_BITDEPTH
  const __m128i min = _mm_set1_epi16(0);
  const __m128i max = _mm_set1_epi16((1 << bitdepth) - 1);
#endif
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width8; x += 8) {
      __m128i s1 = _mm_loadu_si128(CAST_M128_CONST(src1 + x));
      __m128i s2 = _mm_loadu_si128(CAST_M128_CONST(src2 + x));
      __m128i sum =
        _mm_srai_epi16(_mm_adds_epi16(_mm_add_epi16(s1, s2), voffset), shift);
#if XVC_HIGH_BITDEPTH
      __m128i out = _mm_max_epi16(min, _mm_min_epi16(sum, max));
      _mm_storeu_si128(reinterpret_cast<__m128i*>(dst + x), out);
#else
      __m128i out = _mm_packus_epi16(sum, sum);
      _mm_storel_epi64(reinterpret_cast<__m128i*>(dst + x), out);
#endif
    }
    if (Width4(width)) {
      __m128i s1 = _mm_loadl_epi64(CAST_M128_CONST(src1 + width8));
      __m128i s2 = _mm_loadl_epi64(CAST_M128_CONST(src2 + width8));
      __m128i sum =
        _mm_srai_epi16(_mm_adds_epi16(_mm_add_epi16(s1, s2), voffset), shift);
#if XVC_HIGH_BITDEPTH
      __m128i out = _mm_max_epi16(min, _mm_min_epi16(sum, max));
      _mm_storel_epi64(reinterpret_cast<__m128i*>(dst + width8), out);
#else
      __m128i out = _mm_packus_epi16(sum, sum);
      *reinterpret_cast<int32_t*>(dst + width8) = _mm_cvtsi128_si32(out);
#endif
    }
    src1 += stride1;
    src2 += stride2;
    dst += dst_stride;
  }
}
#endif  // XVC_ARCH_X86

#if XVC_HAVE_NEON
static void AddAvgNeon(int width, int height,
                       int offset, int shift, int bitdepth,
                       const int16_t *src1, intptr_t stride1,
                       const int16_t *src2, intptr_t stride2,
                       Sample *dst, intptr_t dst_stride) {
  const int width8 = Width8(width);
  const int16x8_t vshift = vdupq_n_s16((int16_t)shift * -1);
  const int16x8_t voffset = vdupq_n_s16((int16_t)offset);
#if XVC_HIGH_BITDEPTH
  const int16x8_t min = vdupq_n_s16(0);
  const int16x8_t max = vdupq_n_s16((1 << bitdepth) - 1);
#endif
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width8; x += 8) {
      int16x8_t s1 = vld1q_s16(src1 + x);
      int16x8_t s2 = vld1q_s16(src2 + x);
      int16x8_t sum = vshlq_s16(vqaddq_s16(vaddq_s16(s1, s2), voffset), vshift);
#if XVC_HIGH_BITDEPTH
      int16x8_t out = vmaxq_s16(min, vminq_s16(sum, max));
      vst1q_u16(dst + x, vreinterpretq_u16_s16(out));
#else
      uint8x8_t out = vqmovun_s16(sum);
      vst1_u8(dst + x, out);
#endif
    }
    if (Width4(width)) {
      int16x4_t s1 = vld1_s16(src1 + width8);
      int16x4_t s2 = vld1_s16(src2 + width8);
      int16x4_t tmp = vadd_s16(s1, s2);
      int16x4_t sum = vshl_s16(vqadd_s16(tmp, vget_low_s16(voffset)),
                               vget_low_s16(vshift));
#if XVC_HIGH_BITDEPTH
      int16x4_t out = vmax_s16(vget_low_s16(min),
                               vmin_s16(sum, vget_low_s16(max)));
      vst1_u16(dst + width8, vreinterpret_u16_s16(out));
#else
      uint8x8_t out = vqmovun_s16(vcombine_s16(sum, sum));
      vst1_lane_u32(reinterpret_cast<uint32_t*>(dst + width8),
                    vreinterpret_u32_u8(out), 0);
#endif
    }
    src1 += stride1;
    src2 += stride2;
    dst += dst_stride;
  }
}
#endif  // XVC_HAVE_NEON

#if XVC_ARCH_X86
__attribute__((target("sse2")))
static void FilterCopyBipredSse2(int width, int height,
                                 int16_t offset, int shift,
                                 const Sample *ref, ptrdiff_t ref_stride,
                                 int16_t *dst, ptrdiff_t dst_stride) {
  const int width8 = Width8(width);
  __m128i voffset = _mm_set1_epi16(offset);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width8; x += 8) {
#if XVC_HIGH_BITDEPTH
      __m128i s1 = _mm_loadu_si128(CAST_M128_CONST(ref + x));
#else
      __m128i vref = _mm_loadl_epi64(CAST_M128_CONST(ref + x));
      __m128i s1 = _mm_unpacklo_epi8(vref, _mm_setzero_si128());
#endif
      __m128i out = _mm_sub_epi16(_mm_slli_epi16(s1, shift), voffset);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(dst + x), out);
    }
    if (Width4(width)) {
#if XVC_HIGH_BITDEPTH
      __m128i s1 = _mm_loadl_epi64(CAST_M128_CONST(ref + width8));
#else
      // TODO(PH) Loading twice as much data as needed
      __m128i vref = _mm_loadl_epi64(CAST_M128_CONST(ref + width8));
      __m128i s1 = _mm_unpacklo_epi8(vref, _mm_setzero_si128());
#endif
      __m128i out = _mm_sub_epi16(_mm_slli_epi16(s1, shift), voffset);
      _mm_storel_epi64(reinterpret_cast<__m128i*>(dst + width8), out);
    }
    ref += ref_stride;
    dst += dst_stride;
  }
}
#endif  // #if XVC_ARCH_X86

#if XVC_HAVE_NEON
static void FilterCopyBipredNeon(int width, int height,
                                 int16_t offset, int shift,
                                 const Sample *ref, ptrdiff_t ref_stride,
                                 int16_t *dst, ptrdiff_t dst_stride) {
  const int width8 = Width8(width);
  const int16x8_t vshift = vdupq_n_s16((int16_t)shift);
  const int16x8_t voffset = vdupq_n_s16(offset);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width8; x += 8) {
#if XVC_HIGH_BITDEPTH
      int16x8_t s1 = vreinterpretq_s16_u16(vld1q_u16(ref + x));
#else
      uint8x8_t vref = vld1_u8(ref + x);
      int16x8_t s1 = vreinterpretq_s16_u16(vmovl_u8(vref));
#endif
      int16x8_t out = vsubq_s16(vshlq_s16(s1, vshift), voffset);
      vst1q_s16(dst + x, out);
    }
    if (Width4(width)) {
#if XVC_HIGH_BITDEPTH
      int16x4_t s1 = vreinterpret_s16_u16(vld1_u16(ref + width8));
#else
      // TODO(PH) Loading twice as much data as needed
      uint8x8_t vref = vld1_u8(ref + width8);
      int16x4_t s1 = vreinterpret_s16_u16(vget_low_u16(vmovl_u8(vref)));
#endif
      int16x4_t out = vsub_s16(vshl_s16(s1, vget_low_s16(vshift)),
                               vget_low_s16(voffset));
      vst1_s16(dst + width8, out);
    }
    ref += ref_stride;
    dst += dst_stride;
  }
}
#endif

#if XVC_ARCH_X86
template<typename DstT, bool Clip>
__attribute__((target("sse2")))
static
void FilterHorSampleTLumaSse2(int width, int height, int bitdepth,
                              const int16_t *filter,
                              const Sample *src, ptrdiff_t src_stride,
                              DstT *dst, ptrdiff_t dst_stride) {
  const int width8 = Width8(width);
  const int shift = InterPrediction::GetFilterShift<Sample, Clip>(bitdepth);
  const int offset = InterPrediction::GetFilterOffset<Sample, Clip>(shift);
  static_assert(InterPrediction::kNumTapsLuma == 8, "8 tap filter");
  const __m128i voffset = _mm_set1_epi32(static_cast<int32_t>(offset));
  const __m128i vfilter = _mm_loadu_si128(CAST_M128_CONST(filter));
#if XVC_HIGH_BITDEPTH
  const __m128i min = _mm_set1_epi16(0);
  const __m128i max = _mm_set1_epi16((1 << bitdepth) - 1);
#endif

  src -= InterPrediction::kNumTapsLuma / 2 - 1;

  for (int y = 0; y < height; y++) {
    auto fir_2samples_epi32 = [&vfilter](const Sample *sample)
      __attribute__((target("sse2"))) {
#if XVC_HIGH_BITDEPTH
      __m128i ref_a = _mm_loadu_si128(CAST_M128_CONST(sample + 0));
      __m128i ref_b = _mm_loadu_si128(CAST_M128_CONST(sample + 1));
#else
      __m128i ref_a64 = _mm_loadl_epi64(CAST_M128_CONST(sample + 0));
      __m128i ref_b64 = _mm_loadl_epi64(CAST_M128_CONST(sample + 1));
      __m128i ref_a = _mm_unpacklo_epi8(ref_a64, _mm_setzero_si128());
      __m128i ref_b = _mm_unpacklo_epi8(ref_b64, _mm_setzero_si128());
#endif
      // | a01 | a12 | a45 | a67 |
      __m128i prod_a128 = _mm_madd_epi16(ref_a, vfilter);
      __m128i prod_a128_rot = _mm_shuffle_epi32(prod_a128, kBin8_01_00_11_10);
      __m128i prod_a_lo = _mm_add_epi32(prod_a128, prod_a128_rot);
      // | b01 | b12 | b45 | b67 |
      __m128i prod_b128 = _mm_madd_epi16(ref_b, vfilter);
      __m128i prod_b128_rot = _mm_shuffle_epi32(prod_b128, kBin8_01_00_11_10);
      __m128i prod_b_lo = _mm_add_epi32(prod_b128, prod_b128_rot);
      // | a0145 | a2367 | b0145 | b2367 |
      __m128i sum_ab = _mm_unpacklo_epi64(prod_a_lo, prod_b_lo);
      __m128i sum_ab_rot = _mm_shuffle_epi32(sum_ab, kBin8_10_11_00_01);
      __m128i sumAB_lo = _mm_add_epi32(sum_ab, sum_ab_rot);
      // | a01452367 | b01452367 | X | X |
      return _mm_shuffle_epi32(sumAB_lo, kBin8_11_01_10_00);
    };  // NOLINT
    auto fir_4samples_epi32 = [&](const Sample *sample)
      __attribute__((target("sse2"))) {
      __m128i prod_ab_lo = fir_2samples_epi32(sample + 0);
      __m128i prod_cd_lo = fir_2samples_epi32(sample + 2);
      __m128i prod_abcd = _mm_unpacklo_epi64(prod_ab_lo, prod_cd_lo);
      __m128i sum_abcd_offset = _mm_add_epi32(prod_abcd, voffset);
      return _mm_srai_epi32(sum_abcd_offset, shift);
    };  // NOLINT
    for (int x = 0; x < width8; x += 8) {
      __m128i sum_abcd = fir_4samples_epi32(src + x + 0);
      __m128i sum_efgh = fir_4samples_epi32(src + x + 4);
      __m128i sum = _mm_packs_epi32(sum_abcd, sum_efgh);
      if (Clip) {
#if XVC_HIGH_BITDEPTH
        __m128i out = _mm_max_epi16(min, _mm_min_epi16(sum, max));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(dst + x), out);
#else
        __m128i out = _mm_packus_epi16(sum, sum);
        _mm_storel_epi64(reinterpret_cast<__m128i*>(dst + x), out);
#endif
      } else {
        _mm_storeu_si128(reinterpret_cast<__m128i*>(dst + x), sum);
      }
    }
    if (Width4(width)) {
      __m128i sum_abcd = fir_4samples_epi32(src + width8);
      __m128i sum = _mm_packs_epi32(sum_abcd, sum_abcd);
      if (Clip) {
#if XVC_HIGH_BITDEPTH
        __m128i out = _mm_max_epi16(min, _mm_min_epi16(sum, max));
        _mm_storel_epi64(reinterpret_cast<__m128i*>(dst + width8), out);
#else
        __m128i out = _mm_packus_epi16(sum, sum);
        *reinterpret_cast<int32_t*>(dst + width8) = _mm_cvtsi128_si32(out);
#endif
      } else {
        _mm_storel_epi64(reinterpret_cast<__m128i*>(dst + width8), sum);
      }
    }
    src += src_stride;
    dst += dst_stride;
  }
}
#endif  // XVC_ARCH_X86

#if XVC_HAVE_NEON
template<typename DstT, bool Clip>
static
void FilterHorSampleTLumaNeon(int width, int height, int bitdepth,
                              const int16_t *filter,
                              const Sample *src, ptrdiff_t src_stride,
                              DstT *dst, ptrdiff_t dst_stride) {
  const int width8 = Width8(width);
  const int shift = InterPrediction::GetFilterShift<Sample, Clip>(bitdepth);
  const int offset = InterPrediction::GetFilterOffset<Sample, Clip>(shift);
  const int32x4_t vshift = vdupq_n_s32(static_cast<int32_t>(shift) * -1);
  const int32x4_t voffset = vdupq_n_s32(static_cast<int32_t>(offset));
  static_assert(InterPrediction::kNumTapsLuma == 8, "8 tap filter");
  const int16x8_t vfilter = vld1q_s16(filter);
  const int16x4_t vfilter_lo = vget_low_s16(vfilter);
  const int16x4_t vfilter_hi = vget_high_s16(vfilter);
#if XVC_HIGH_BITDEPTH
  const int16x8_t min = vdupq_n_s16(0);
  const int16x8_t max = vdupq_n_s16((1 << bitdepth) - 1);
#endif
  auto load_row = [](const Sample *sample) {
#if XVC_HIGH_BITDEPTH
    return vreinterpretq_s16_u16(vld1q_u16(sample));
#else
    uint8x8_t vref_a8 = vld1_u8(sample);
    return vreinterpretq_s16_u16(vmovl_u8(vref_a8));
#endif
  };

  src -= InterPrediction::kNumTapsLuma / 2 - 1;

  for (int y = 0; y < height; y++) {
    auto fir_1sample_s32 = [&vfilter_lo, &vfilter_hi](int16x8_t vref_a) {
      int32x4_t prod_a_lo = vmull_s16(vget_low_s16(vref_a), vfilter_lo);
      int32x4_t prod_a_hi = vmull_s16(vget_high_s16(vref_a), vfilter_hi);
      int32x4_t prod_a = vaddq_s32(prod_a_lo, prod_a_hi);
      return vadd_s32(vget_low_s32(prod_a), vget_high_s32(prod_a));
    };
    auto fir_2samples_s32 = [&fir_1sample_s32](int16x8_t vref_a,
                                               int16x8_t vref_b) {
      int32x2_t sum_a_lo = fir_1sample_s32(vref_a);  // a0426 a1537
      int32x2_t sum_b_lo = fir_1sample_s32(vref_b);  // b0426 b1537
      return vpadd_s32(sum_a_lo, sum_b_lo);
    };
    for (int x = 0; x < width8; x += 8) {
      int16x8_t vref_lo = load_row(src + x + 0);
      int16x8_t vref_hi = load_row(src + x + 8);
      int32x2_t prod_ab = fir_2samples_s32(vref_lo,
                                           vextq_s16(vref_lo, vref_hi, 1));
      int32x2_t prod_cd = fir_2samples_s32(vextq_s16(vref_lo, vref_hi, 2),
                                           vextq_s16(vref_lo, vref_hi, 3));
      int32x2_t prod_ef = fir_2samples_s32(vextq_s16(vref_lo, vref_hi, 4),
                                           vextq_s16(vref_lo, vref_hi, 5));
      int32x2_t prod_gh = fir_2samples_s32(vextq_s16(vref_lo, vref_hi, 6),
                                           vextq_s16(vref_lo, vref_hi, 7));
      int32x4_t prod_abcd = vcombine_s32(prod_ab, prod_cd);
      int32x4_t prod_efgh = vcombine_s32(prod_ef, prod_gh);
      int32x4_t sum_abcd_offset = vaddq_s32(prod_abcd, voffset);
      int32x4_t sum_efgh_offset = vaddq_s32(prod_efgh, voffset);
      int16x4_t sum_abcd = vqmovn_s32(vshlq_s32(sum_abcd_offset, vshift));
      int16x4_t sum_efgh = vqmovn_s32(vshlq_s32(sum_efgh_offset, vshift));
      int16x8_t sum = vcombine_s16(sum_abcd, sum_efgh);
      if (Clip) {
#if XVC_HIGH_BITDEPTH
        int16x8_t out = vmaxq_s16(min, vminq_s16(sum, max));
        vst1q_u16(reinterpret_cast<uint16_t*>(dst + x),
                  vreinterpretq_u16_s16(out));
#else
        uint8x8_t out = vqmovun_s16(sum);
        vst1_u8(reinterpret_cast<uint8_t*>(dst + x), out);
#endif
      } else {
        // Replace DstT reinterpret_cast with C++17 if constexpr
        vst1q_s16(reinterpret_cast<int16_t*>(dst + x), sum);
      }
    }
    if (Width4(width)) {
#if XVC_HIGH_BITDEPTH
      int16x8_t vref_lo = vreinterpretq_s16_u16(vld1q_u16(src + width8 + 0));
      int16x4_t vref_hi_lo = vreinterpret_s16_u16(vld1_u16(src + width8 + 8));
      int16x8_t vref_hi = vcombine_s16(vref_hi_lo, vref_hi_lo);
#else
      uint8x16_t vref_all = vld1q_u8(src + width8 + 0);
      int16x8_t vref_lo =
        vreinterpretq_s16_u16(vmovl_u8(vget_low_u8(vref_all)));
      int16x8_t vref_hi =
        vreinterpretq_s16_u16(vmovl_u8(vget_high_u8(vref_all)));
#endif
      int32x2_t prod_ab = fir_2samples_s32(vref_lo,
                                           vextq_s16(vref_lo, vref_hi, 1));
      int32x2_t prod_cd = fir_2samples_s32(vextq_s16(vref_lo, vref_hi, 2),
                                           vextq_s16(vref_lo, vref_hi, 3));
      int32x4_t prod_abcd = vcombine_s32(prod_ab, prod_cd);
      int32x4_t sum_abcd_offset = vaddq_s32(prod_abcd, voffset);
      int16x4_t sum = vqmovn_s32(vshlq_s32(sum_abcd_offset, vshift));
      if (Clip) {
#if XVC_HIGH_BITDEPTH
        int16x4_t out = vmax_s16(vget_low_s16(min),
                                 vmin_s16(sum, vget_low_s16(max)));
        vst1_u16(reinterpret_cast<uint16_t*>(dst + width8),
                 vreinterpret_u16_s16(out));
#else
        uint8x8_t out = vqmovun_s16(vcombine_s16(sum, sum));
        vst1_lane_u32(reinterpret_cast<uint32_t*>(dst + width8),
                      vreinterpret_u32_u8(out), 0);
#endif
      } else {
        // Replace DstT reinterpret_cast with C++17 if constexpr
        vst1_s16(reinterpret_cast<int16_t*>(dst + width8), sum);
      }
    }
    src += src_stride;
    dst += dst_stride;
  }
}
#endif  // XVC_HAVE_NEON

#if XVC_ARCH_X86
template<typename DstT, bool Clip>
__attribute__((target("sse2")))
static
void FilterHorSampleTChromaSse2(int width, int height, int bitdepth,
                                const int16_t *filter,
                                const Sample *src, ptrdiff_t src_stride,
                                DstT *dst, ptrdiff_t dst_stride) {
  const int width8 = Width8(width);
  const int shift = InterPrediction::GetFilterShift<Sample, Clip>(bitdepth);
  const int offset = InterPrediction::GetFilterOffset<Sample, Clip>(shift);
  static_assert(InterPrediction::kNumTapsChroma == 4, "4 tap filter");
  const __m128i voffset = _mm_set1_epi32(static_cast<int32_t>(offset));
  const __m128i vfilter4 = _mm_loadl_epi64(CAST_M128_CONST(filter));
  const __m128i vfilter = _mm_unpacklo_epi64(vfilter4, vfilter4);
#if XVC_HIGH_BITDEPTH
  const __m128i min = _mm_set1_epi16(0);
  const __m128i max = _mm_set1_epi16((1 << bitdepth) - 1);
#endif

  src -= InterPrediction::kNumTapsChroma / 2 - 1;

  for (int y = 0; y < height; y++) {
    auto fir_4samples_epi32 = [&](const Sample *sample)
      __attribute__((target("sse2"))) {
      __m128i ref_a = _mm_loadl_epi64(CAST_M128_CONST(sample + 0));
      __m128i ref_b = _mm_loadl_epi64(CAST_M128_CONST(sample + 1));
      __m128i ref_c = _mm_loadl_epi64(CAST_M128_CONST(sample + 2));
      __m128i ref_d = _mm_loadl_epi64(CAST_M128_CONST(sample + 3));
#if !XVC_HIGH_BITDEPTH
      ref_a = _mm_unpacklo_epi8(ref_a, _mm_setzero_si128());
      ref_b = _mm_unpacklo_epi8(ref_b, _mm_setzero_si128());
      ref_c = _mm_unpacklo_epi8(ref_c, _mm_setzero_si128());
      ref_d = _mm_unpacklo_epi8(ref_d, _mm_setzero_si128());
#endif
      __m128i ref_ab = _mm_unpacklo_epi64(ref_a, ref_b);
      __m128i ref_cd = _mm_unpacklo_epi64(ref_c, ref_d);
      __m128i prod_ab = _mm_madd_epi16(ref_ab, vfilter);
      __m128i prod_cd = _mm_madd_epi16(ref_cd, vfilter);
      // | a01 | c01 | a23 | c23 |
      __m128i prod_ac_mix = _mm_unpacklo_epi32(prod_ab, prod_cd);
      __m128i prod_bd_mix = _mm_unpackhi_epi32(prod_ab, prod_cd);
      __m128i prod_abcd_lo = _mm_unpacklo_epi32(prod_ac_mix, prod_bd_mix);
      __m128i prod_abcd_hi = _mm_unpackhi_epi32(prod_ac_mix, prod_bd_mix);
      __m128i prod_abcd = _mm_add_epi32(prod_abcd_lo, prod_abcd_hi);
      __m128i sum_abcd_offset = _mm_add_epi32(prod_abcd, voffset);
      return _mm_srai_epi32(sum_abcd_offset, shift);
    };  // NOLINT
    for (int x = 0; x < width8; x += 8) {
      __m128i sum_abcd = fir_4samples_epi32(src + x + 0);
      __m128i sum_efgh = fir_4samples_epi32(src + x + 4);
      __m128i sum = _mm_packs_epi32(sum_abcd, sum_efgh);
      if (Clip) {
#if XVC_HIGH_BITDEPTH
        __m128i out = _mm_max_epi16(min, _mm_min_epi16(sum, max));
        _mm_storeu_si128(reinterpret_cast<__m128i*>(dst + x), out);
#else
        __m128i out = _mm_packus_epi16(sum, sum);
        _mm_storel_epi64(reinterpret_cast<__m128i*>(dst + x), out);
#endif
      } else {
        _mm_storeu_si128(reinterpret_cast<__m128i*>(dst + x), sum);
      }
    }
#if XVC_HIGH_BITDEPTH
    if (Width4(width)) {
#else
    if (width & 4) {
#endif
      __m128i sum_abcd = fir_4samples_epi32(src + width8);
      __m128i sum = _mm_packs_epi32(sum_abcd, sum_abcd);
      if (Clip) {
#if XVC_HIGH_BITDEPTH
        __m128i out = _mm_max_epi16(min, _mm_min_epi16(sum, max));
        _mm_storel_epi64(reinterpret_cast<__m128i*>(dst + width8), out);
#else
        __m128i out = _mm_packus_epi16(sum, sum);
        *reinterpret_cast<int32_t*>(dst + width8) = _mm_cvtsi128_si32(out);
#endif
      } else {
        _mm_storel_epi64(reinterpret_cast<__m128i*>(dst + width8), sum);
      }
    }
#if !XVC_HIGH_BITDEPTH
    // Special handling of non 4-byte aligned stores
    // This happens for non high bitdepth builds and when CU width is 2
    if (width & 2) {
      const int width2 = width & ~2;
      for (int x = 0; x < 2; x++) {
        int sum = src[width2 + x + 0] * filter[0];
        sum += src[width2 + x + 1] * filter[1];
        sum += src[width2 + x + 2] * filter[2];
        sum += src[width2 + x + 3] * filter[3];
        if (Clip) {
          dst[width2 + x] =
            util::ClipBD((sum + offset) >> shift, (1 << bitdepth) - 1);
        } else {
          dst[width2 + x] = static_cast<DstT>((sum + offset) >> shift);
        }
      }
    }
#endif
    src += src_stride;
    dst += dst_stride;
  }
}
#endif  // XVC_ARCH_X86

#if XVC_HAVE_NEON
template<typename DstT, bool Clip>
static
void FilterHorSampleTChromaNeon(int width, int height, int bitdepth,
                                const int16_t *filter,
                                const Sample *src, ptrdiff_t src_stride,
                                DstT *dst, ptrdiff_t dst_stride) {
  const int width8 = Width8(width);
  const int shift = InterPrediction::GetFilterShift<Sample, Clip>(bitdepth);
  const int offset = InterPrediction::GetFilterOffset<Sample, Clip>(shift);
  const int32x4_t vshift = vdupq_n_s32(static_cast<int32_t>(shift) * -1);
  const int32x4_t voffset = vdupq_n_s32(static_cast<int32_t>(offset));
  static_assert(InterPrediction::kNumTapsChroma == 4, "4 tap filter");
  const int16x4_t vfilter = vld1_s16(filter);
#if XVC_HIGH_BITDEPTH
  const int16x8_t min = vdupq_n_s16(0);
  const int16x8_t max = vdupq_n_s16((1 << bitdepth) - 1);
#endif

  src -= InterPrediction::kNumTapsChroma / 2 - 1;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width8; x += 8) {
      auto fir_2samples_s32 = [&vfilter](int16x8_t vref_a, int16x8_t vref_b) {
        int32x4_t prod_a = vmull_s16(vget_low_s16(vref_a), vfilter);
        int32x4_t prod_b = vmull_s16(vget_low_s16(vref_b), vfilter);
        int32x2_t sum_a = vadd_s32(vget_low_s32(prod_a), vget_high_s32(prod_a));
        int32x2_t sum_b = vadd_s32(vget_low_s32(prod_b), vget_high_s32(prod_b));
        return vpadd_s32(sum_a, sum_b);
      };
#if XVC_HIGH_BITDEPTH
      int16x8_t vref_lo = vreinterpretq_s16_u16(vld1q_u16(src + x + 0));
      int16x8_t vref_hi = vreinterpretq_s16_u16(vld1q_u16(src + x + 8));
#else
      int16x8_t vref_lo = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(src + x + 0)));
      int16x8_t vref_hi = vreinterpretq_s16_u16(vmovl_u8(vld1_u8(src + x + 8)));
#endif
      int32x2_t prod_ab = fir_2samples_s32(vref_lo,
                                           vextq_s16(vref_lo, vref_hi, 1));
      int32x2_t prod_cd = fir_2samples_s32(vextq_s16(vref_lo, vref_hi, 2),
                                           vextq_s16(vref_lo, vref_hi, 3));
      int32x2_t prod_ef = fir_2samples_s32(vextq_s16(vref_lo, vref_hi, 4),
                                           vextq_s16(vref_lo, vref_hi, 5));
      int32x2_t prod_gh = fir_2samples_s32(vextq_s16(vref_lo, vref_hi, 6),
                                           vextq_s16(vref_lo, vref_hi, 7));
      int32x4_t prod_abcd = vcombine_s32(prod_ab, prod_cd);
      int32x4_t prod_efgh = vcombine_s32(prod_ef, prod_gh);
      int32x4_t sum_abcd_offset = vaddq_s32(prod_abcd, voffset);
      int32x4_t sum_efgh_offset = vaddq_s32(prod_efgh, voffset);
      int16x4_t sum_abcd = vqmovn_s32(vshlq_s32(sum_abcd_offset, vshift));
      int16x4_t sum_efgh = vqmovn_s32(vshlq_s32(sum_efgh_offset, vshift));
      int16x8_t sum = vcombine_s16(sum_abcd, sum_efgh);
      if (Clip) {
#if XVC_HIGH_BITDEPTH
        int16x8_t out = vmaxq_s16(min, vminq_s16(sum, max));
        vst1q_u16(reinterpret_cast<uint16_t*>(dst + x),
                  vreinterpretq_u16_s16(out));
#else
        uint8x8_t out = vqmovun_s16(sum);
        vst1_u8(reinterpret_cast<uint8_t*>(dst + x), out);
#endif
      } else {
        vst1q_s16(reinterpret_cast<int16_t*>(dst + x), sum);
      }
    }
    if (Width4(width)) {
      auto fir_2samples_s32 = [&vfilter](int16x4_t vref_a, int16x4_t vref_b) {
        int32x4_t prod_a = vmull_s16(vref_a, vfilter);
        int32x4_t prod_b = vmull_s16(vref_b, vfilter);
        int32x2_t sum_a = vadd_s32(vget_low_s32(prod_a), vget_high_s32(prod_a));
        int32x2_t sum_b = vadd_s32(vget_low_s32(prod_b), vget_high_s32(prod_b));
        return vpadd_s32(sum_a, sum_b);
      };
#if XVC_HIGH_BITDEPTH
      int16x4_t vref_lo = vreinterpret_s16_u16(vld1_u16(src + width8 + 0));
      int16x4_t vref_hi = vreinterpret_s16_u16(vld1_u16(src + width8 + 4));
#else
      int16x8_t vref_both =
        vreinterpretq_s16_u16(vmovl_u8(vld1_u8(src + width8 + 0)));
      int16x4_t vref_lo = vget_low_s16(vref_both);
      int16x4_t vref_hi = vget_high_s16(vref_both);
#endif
      int32x2_t prod_ab = fir_2samples_s32(vref_lo,
                                           vext_s16(vref_lo, vref_hi, 1));
      int32x2_t prod_cd = fir_2samples_s32(vext_s16(vref_lo, vref_hi, 2),
                                           vext_s16(vref_lo, vref_hi, 3));
      int32x4_t prod_abcd = vcombine_s32(prod_ab, prod_cd);
      int32x4_t sum_abcd_offset = vaddq_s32(prod_abcd, voffset);
      int16x4_t sum = vqmovn_s32(vshlq_s32(sum_abcd_offset, vshift));
      if (Clip) {
#if XVC_HIGH_BITDEPTH
        int16x4_t out = vmax_s16(vget_low_s16(min),
                                 vmin_s16(sum, vget_low_s16(max)));
        vst1_u16(reinterpret_cast<uint16_t*>(dst + width8),
                 vreinterpret_u16_s16(out));
#else
        uint8x8_t out = vqmovun_s16(vcombine_s16(sum, sum));
        vst1_lane_u32(reinterpret_cast<uint32_t*>(dst + width8),
                      vreinterpret_u32_u8(out), 0);
#endif
      } else {
        vst1_s16(reinterpret_cast<int16_t*>(dst + width8), sum);
      }
    }
    src += src_stride;
    dst += dst_stride;
  }
}
#endif  // XVC_HAVE_NEON

#if XVC_ARCH_X86
template<typename SrcT, typename DstT, bool Clip>
__attribute__((target("sse2")))
static
void FilterVerLumaSse2(int width, int height, int bitdepth,
                       const int16_t *filter,
                       const SrcT *src, ptrdiff_t src_stride,
                       DstT *dst, ptrdiff_t dst_stride) {
  const int shift = InterPrediction::GetFilterShift<SrcT, Clip>(bitdepth);
  const int offset = InterPrediction::GetFilterOffset<SrcT, Clip>(shift);
  const __m128i voffset = _mm_set1_epi32(static_cast<int32_t>(offset));
  const __m128i min = _mm_set1_epi16(0);
  const __m128i max = _mm_set1_epi16((1 << bitdepth) - 1);
  static_assert(InterPrediction::kNumTapsLuma == 8, "8 tap filter");
  const __m128i vfilter01 = _mm_set1_epi32(
    (filter[0] & 0xFFFF) | (static_cast<uint16_t>(filter[1]) << 16));
  const __m128i vfilter23 = _mm_set1_epi32(
    (filter[2] & 0xFFFF) | (static_cast<uint16_t>(filter[3]) << 16));
  const __m128i vfilter45 = _mm_set1_epi32(
    (filter[4] & 0xFFFF) | (static_cast<uint16_t>(filter[5]) << 16));
  const __m128i vfilter67 = _mm_set1_epi32(
    (filter[6] & 0xFFFF) | (static_cast<uint16_t>(filter[7]) << 16));
  const __m128i zero = _mm_setzero_si128();

  src -= (InterPrediction::kNumTapsLuma / 2 - 1) * src_stride;

  for (int y = 0; y < height; y += 4) {
    for (int x = 0; x < width; x += 4) {
      auto combine_rows_epi16 = [&](__m128i r0, __m128i r1)
        __attribute__((target("sse2"))) {
        if (std::is_same<SrcT, uint16_t>::value ||
            std::is_same<SrcT, int16_t>::value) {
          return _mm_unpacklo_epi16(r0, r1);
        } else if (std::is_same<SrcT, uint8_t>::value) {
          return _mm_unpacklo_epi8(_mm_unpacklo_epi8(r0, r1), zero);
        } else {
          assert(0);
          return __m128i();
        }
      };  // NOLINT
      auto filter_8rows_lo = [&](const __m128i &data01, const __m128i &data23,
                                 const __m128i &data56, const __m128i &data67)
        __attribute__((target("sse2"))) {
        __m128i prod0_01 = _mm_madd_epi16(data01, vfilter01);
        __m128i prod0_23 = _mm_madd_epi16(data23, vfilter23);
        __m128i prod0_45 = _mm_madd_epi16(data56, vfilter45);
        __m128i prod0_67 = _mm_madd_epi16(data67, vfilter67);
        __m128i prod_sum = _mm_add_epi32(prod0_01, prod0_23);
        prod_sum = _mm_add_epi32(prod_sum, prod0_45);
        prod_sum = _mm_add_epi32(prod_sum, prod0_67);
        __m128i sum_offset = _mm_add_epi32(prod_sum, voffset);
        return _mm_srai_epi32(sum_offset, shift);
      };  // NOLINT
      __m128i row0 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 0 * src_stride));
      __m128i row1 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 1 * src_stride));
      __m128i row2 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 2 * src_stride));
      __m128i row3 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 3 * src_stride));
      __m128i row4 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 4 * src_stride));
      __m128i row5 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 5 * src_stride));
      __m128i row6 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 6 * src_stride));
      __m128i row7 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 7 * src_stride));
      __m128i row8 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 8 * src_stride));
      __m128i row9 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 9 * src_stride));
      __m128i row10 =
        _mm_loadl_epi64(CAST_M128_CONST(src + x + 10 * src_stride));
      __m128i row01 = combine_rows_epi16(row0, row1);
      __m128i row12 = combine_rows_epi16(row1, row2);
      __m128i row23 = combine_rows_epi16(row2, row3);
      __m128i row34 = combine_rows_epi16(row3, row4);
      __m128i row45 = combine_rows_epi16(row4, row5);
      __m128i row56 = combine_rows_epi16(row5, row6);
      __m128i row67 = combine_rows_epi16(row6, row7);
      __m128i row78 = combine_rows_epi16(row7, row8);
      __m128i row89 = combine_rows_epi16(row8, row9);
      __m128i row90 = combine_rows_epi16(row9, row10);
      __m128i sum0 = filter_8rows_lo(row01, row23, row45, row67);
      __m128i sum1 = filter_8rows_lo(row12, row34, row56, row78);
      __m128i sum2 = filter_8rows_lo(row23, row45, row67, row89);
      __m128i sum3 = filter_8rows_lo(row34, row56, row78, row90);
      __m128i sum01 = _mm_packs_epi32(sum0, sum1);
      __m128i sum23 = _mm_packs_epi32(sum2, sum3);

      if (std::is_same<DstT, uint16_t>::value ||
          std::is_same<DstT, int16_t>::value) {
        __m128i out01 =
          Clip ? _mm_max_epi16(min, _mm_min_epi16(sum01, max)) : sum01;
        __m128i out23 =
          Clip ? _mm_max_epi16(min, _mm_min_epi16(sum23, max)) : sum23;
        _mm_storel_epi64(
          reinterpret_cast<__m128i*>(dst + x + 0 * dst_stride), out01);
        __m128i out1 = _mm_shuffle_epi32(out01, kBin8_01_00_11_10);
        _mm_storel_epi64(
          reinterpret_cast<__m128i*>(dst + x + 1 * dst_stride), out1);
        _mm_storel_epi64(
          reinterpret_cast<__m128i*>(dst + x + 2 * dst_stride), out23);
        __m128i out3 = _mm_shuffle_epi32(out23, kBin8_01_00_11_10);
        _mm_storel_epi64(
          reinterpret_cast<__m128i*>(dst + x + 3 * dst_stride), out3);
      } else if (std::is_same<DstT, uint8_t>::value) {
        __m128i out01 = _mm_packus_epi16(sum01, sum01);
        __m128i out23 = _mm_packus_epi16(sum23, sum23);
        *reinterpret_cast<int32_t*>(dst + x + 0 * dst_stride) =
          _mm_cvtsi128_si32(out01);
        __m128i out1 = _mm_srli_si128(out01, 4);
        *reinterpret_cast<int32_t*>(dst + x + 1 * dst_stride) =
          _mm_cvtsi128_si32(out1);
        *reinterpret_cast<int32_t*>(dst + x + 2 * dst_stride) =
          _mm_cvtsi128_si32(out23);
        __m128i out3 = _mm_srli_si128(out23, 4);
        *reinterpret_cast<int32_t*>(dst + x + 3 * dst_stride) =
          _mm_cvtsi128_si32(out3);
      }
    }
    src += src_stride * 4;
    dst += dst_stride * 4;
  }
}
#endif  // XVC_ARCH_X86

#if XVC_HAVE_NEON
template<typename SrcT, typename DstT, bool Clip>
static
void FilterVerLumaNeon(int width, int height, int bitdepth,
                       const int16_t *filter,
                       const SrcT *src, ptrdiff_t src_stride,
                       DstT *dst, ptrdiff_t dst_stride) {
  const int shift = InterPrediction::GetFilterShift<SrcT, Clip>(bitdepth);
  const int offset = InterPrediction::GetFilterOffset<SrcT, Clip>(shift);
  const int32x4_t vshift = vdupq_n_s32(static_cast<int32_t>(shift) * -1);
  const int32x4_t voffset = vdupq_n_s32(static_cast<int32_t>(offset));
  const int16x8_t min = vdupq_n_s16(0);
  const int16x8_t max = vdupq_n_s16((1 << bitdepth) - 1);
  static_assert(InterPrediction::kNumTapsLuma == 8, "8 tap filter");
  const int16x4_t vfilter0 = vdup_n_s16(filter[0]);
  const int16x4_t vfilter1 = vdup_n_s16(filter[1]);
  const int16x4_t vfilter2 = vdup_n_s16(filter[2]);
  const int16x4_t vfilter3 = vdup_n_s16(filter[3]);
  const int16x4_t vfilter4 = vdup_n_s16(filter[4]);
  const int16x4_t vfilter5 = vdup_n_s16(filter[5]);
  const int16x4_t vfilter6 = vdup_n_s16(filter[6]);
  const int16x4_t vfilter7 = vdup_n_s16(filter[7]);

  src -= (InterPrediction::kNumTapsLuma / 2 - 1) * src_stride;

  for (int y = 0; y < height; y += 4) {
    for (int x = 0; x < width; x += 4) {
      auto load_row_int16x4 = [](const SrcT *src2) {
        if (std::is_same<SrcT, uint16_t>::value ||
            std::is_same<SrcT, int16_t>::value) {
          return vld1_s16(reinterpret_cast<const int16_t*>(src2));
        } else if (std::is_same<SrcT, uint8_t>::value) {
          // TODO(PH) Loading twice as much data as needed
          auto *p = reinterpret_cast<const uint8_t*>(src2);
          return vreinterpret_s16_u16(vget_low_u16(vmovl_u8(vld1_u8(p))));
        } else {
          assert(0);
          return int16x4_t();
        }
      };
      auto fir_8r = [&](int16x4_t r0, int16x4_t r1, int16x4_t r2,
                        int16x4_t r3, int16x4_t r4, int16x4_t r5,
                        int16x4_t r6, int16x4_t r7) {
        int32x4_t prod0 = vmull_s16(r0, vfilter0);
        prod0 = vmlal_s16(prod0, r1, vfilter1);
        prod0 = vmlal_s16(prod0, r2, vfilter2);
        prod0 = vmlal_s16(prod0, r3, vfilter3);
        prod0 = vmlal_s16(prod0, r4, vfilter4);
        prod0 = vmlal_s16(prod0, r5, vfilter5);
        prod0 = vmlal_s16(prod0, r6, vfilter6);
        prod0 = vmlal_s16(prod0, r7, vfilter7);
        int32x4_t sum0_offset = vaddq_s32(prod0, voffset);
        int32x4_t sum0 = vshlq_s32(sum0_offset, vshift);
        return vqmovn_s32(sum0);
      };
      int16x4_t row0 = load_row_int16x4(src + x + 0 * src_stride);
      int16x4_t row1 = load_row_int16x4(src + x + 1 * src_stride);
      int16x4_t row2 = load_row_int16x4(src + x + 2 * src_stride);
      int16x4_t row3 = load_row_int16x4(src + x + 3 * src_stride);
      int16x4_t row4 = load_row_int16x4(src + x + 4 * src_stride);
      int16x4_t row5 = load_row_int16x4(src + x + 5 * src_stride);
      int16x4_t row6 = load_row_int16x4(src + x + 6 * src_stride);
      int16x4_t row7 = load_row_int16x4(src + x + 7 * src_stride);
      int16x4_t row8 = load_row_int16x4(src + x + 8 * src_stride);
      int16x4_t row9 = load_row_int16x4(src + x + 9 * src_stride);
      int16x4_t row10 = load_row_int16x4(src + x + 10 * src_stride);
      int16x4_t sum0 = fir_8r(row0, row1, row2, row3, row4, row5, row6, row7);
      int16x4_t sum1 = fir_8r(row1, row2, row3, row4, row5, row6, row7, row8);
      int16x4_t sum2 = fir_8r(row2, row3, row4, row5, row6, row7, row8, row9);
      int16x4_t sum3 = fir_8r(row3, row4, row5, row6, row7, row8, row9, row10);
      if (!Clip) {
        // Replace DstT reinterpret_cast with C++17 if constexpr
        vst1_s16(reinterpret_cast<int16_t*>(dst + x + 0 * dst_stride), sum0);
        vst1_s16(reinterpret_cast<int16_t*>(dst + x + 1 * dst_stride), sum1);
        vst1_s16(reinterpret_cast<int16_t*>(dst + x + 2 * dst_stride), sum2);
        vst1_s16(reinterpret_cast<int16_t*>(dst + x + 3 * dst_stride), sum3);
      } else if (std::is_same<DstT, uint16_t>::value ||
                 std::is_same<DstT, int16_t>::value) {
        int16x8_t sum01 = vcombine_s16(sum0, sum1);
        int16x8_t sum23 = vcombine_s16(sum2, sum3);
        int16x8_t out01 = vmaxq_s16(min, vminq_s16(sum01, max));
        int16x8_t out23 = vmaxq_s16(min, vminq_s16(sum23, max));
        vst1_u16(reinterpret_cast<uint16_t*>(dst + x + 0 * dst_stride),
                 vreinterpret_u16_s16(vget_low_s16(out01)));
        vst1_u16(reinterpret_cast<uint16_t*>(dst + x + 1 * dst_stride),
                 vreinterpret_u16_s16(vget_high_s16(out01)));
        vst1_u16(reinterpret_cast<uint16_t*>(dst + x + 2 * dst_stride),
                 vreinterpret_u16_s16(vget_low_s16(out23)));
        vst1_u16(reinterpret_cast<uint16_t*>(dst + x + 3 * dst_stride),
                 vreinterpret_u16_s16(vget_high_s16(out23)));
      } else if (std::is_same<DstT, uint8_t>::value) {
        int16x8_t sum01 = vcombine_s16(sum0, sum1);
        int16x8_t sum23 = vcombine_s16(sum2, sum3);
        uint32x2_t out01 = vreinterpret_u32_u8(vqmovun_s16(sum01));
        uint32x2_t out23 = vreinterpret_u32_u8(vqmovun_s16(sum23));
        vst1_lane_u32(reinterpret_cast<uint32_t*>(dst + x + 0 * dst_stride),
                      out01, 0);
        vst1_lane_u32(reinterpret_cast<uint32_t*>(dst + x + 1 * dst_stride),
                      out01, 1);
        vst1_lane_u32(reinterpret_cast<uint32_t*>(dst + x + 2 * dst_stride),
                      out23, 0);
        vst1_lane_u32(reinterpret_cast<uint32_t*>(dst + x + 3 * dst_stride),
                      out23, 1);
      }
    }
    src += src_stride * 4;
    dst += dst_stride * 4;
  }
}
#endif  // XVC_HAVE_NEON

#if XVC_ARCH_X86
template<typename SrcT, typename DstT, bool Clip>
__attribute__((target("sse2")))
static
void FilterVerChromaSse2(int width, int height, int bitdepth,
                         const int16_t *filter,
                         const SrcT *src, ptrdiff_t src_stride,
                         DstT *dst, ptrdiff_t dst_stride) {
  const int height4 = height & ~3;
  const int shift = InterPrediction::GetFilterShift<SrcT, Clip>(bitdepth);
  const int offset = InterPrediction::GetFilterOffset<SrcT, Clip>(shift);
  const __m128i voffset = _mm_set1_epi32(static_cast<int32_t>(offset));
  const __m128i min = _mm_set1_epi16(0);
  const __m128i max = _mm_set1_epi16((1 << bitdepth) - 1);
  static_assert(InterPrediction::kNumTapsChroma == 4, "4 tap filter");
  const __m128i vfilter01 = _mm_set1_epi32(
    (filter[0] & 0xFFFF) | (static_cast<uint16_t>(filter[1]) << 16));
  const __m128i vfilter23 = _mm_set1_epi32(
    (filter[2] & 0xFFFF) | (static_cast<uint16_t>(filter[3]) << 16));
  const __m128i zero = _mm_setzero_si128();
  auto combine_rows_epi16 = [&](__m128i r0, __m128i r1)
    __attribute__((target("sse2"))) {
    if (std::is_same<SrcT, uint16_t>::value ||
        std::is_same<SrcT, int16_t>::value) {
      return _mm_unpacklo_epi16(r0, r1);
    } else if (std::is_same<SrcT, uint8_t>::value) {
      return _mm_unpacklo_epi8(_mm_unpacklo_epi8(r0, r1), zero);
    } else {
      assert(0);
      return __m128i();
    }
  };  // NOLINT
  auto filter_4rows_epi32 = [&](const __m128i &data01, const __m128i &data23)
    __attribute__((target("sse2"))) {
    __m128i prod0_01 = _mm_madd_epi16(data01, vfilter01);
    __m128i prod0_23 = _mm_madd_epi16(data23, vfilter23);
    __m128i prod_sum = _mm_add_epi32(prod0_01, prod0_23);
    __m128i sum_offset = _mm_add_epi32(prod_sum, voffset);
    return _mm_srai_epi32(sum_offset, shift);
  };  // NOLINT
#if XVC_HIGH_BITDEPTH
  const int width_reduced = width;
#else
  const int width_reduced = width & ~3;
#endif

  src -= (InterPrediction::kNumTapsChroma / 2 - 1) * src_stride;

  for (int y = 0; y < height4; y += 4) {
    for (int x = 0; x < width_reduced; x += 4) {
      __m128i row0 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 0 * src_stride));
      __m128i row1 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 1 * src_stride));
      __m128i row2 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 2 * src_stride));
      __m128i row3 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 3 * src_stride));
      __m128i row4 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 4 * src_stride));
      __m128i row5 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 5 * src_stride));
      __m128i row6 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 6 * src_stride));
      __m128i row01 = combine_rows_epi16(row0, row1);
      __m128i row12 = combine_rows_epi16(row1, row2);
      __m128i row23 = combine_rows_epi16(row2, row3);
      __m128i row34 = combine_rows_epi16(row3, row4);
      __m128i row45 = combine_rows_epi16(row4, row5);
      __m128i row56 = combine_rows_epi16(row5, row6);
      __m128i sum0 = filter_4rows_epi32(row01, row23);
      __m128i sum1 = filter_4rows_epi32(row12, row34);
      __m128i sum2 = filter_4rows_epi32(row23, row45);
      __m128i sum3 = filter_4rows_epi32(row34, row56);
      __m128i sum01 = _mm_packs_epi32(sum0, sum1);
      __m128i sum23 = _mm_packs_epi32(sum2, sum3);
      if (std::is_same<DstT, uint16_t>::value ||
          std::is_same<DstT, int16_t>::value) {
        __m128i out01 =
          Clip ? _mm_max_epi16(min, _mm_min_epi16(sum01, max)) : sum01;
        __m128i out23 =
          Clip ? _mm_max_epi16(min, _mm_min_epi16(sum23, max)) : sum23;
        _mm_storel_epi64(
          reinterpret_cast<__m128i*>(dst + x + 0 * dst_stride), out01);
        __m128i out1 = _mm_shuffle_epi32(out01, kBin8_01_00_11_10);
        _mm_storel_epi64(
          reinterpret_cast<__m128i*>(dst + x + 1 * dst_stride), out1);
        _mm_storel_epi64(
          reinterpret_cast<__m128i*>(dst + x + 2 * dst_stride), out23);
        __m128i out3 = _mm_shuffle_epi32(out23, kBin8_01_00_11_10);
        _mm_storel_epi64(
          reinterpret_cast<__m128i*>(dst + x + 3 * dst_stride), out3);
      } else if (std::is_same<DstT, uint8_t>::value) {
        __m128i out01 = _mm_packus_epi16(sum01, sum01);
        __m128i out23 = _mm_packus_epi16(sum23, sum23);
        *reinterpret_cast<int32_t*>(dst + x + 0 * dst_stride) =
          _mm_cvtsi128_si32(out01);
        __m128i out1 = _mm_srli_si128(out01, 4);
        *reinterpret_cast<int32_t*>(dst + x + 1 * dst_stride) =
          _mm_cvtsi128_si32(out1);
        *reinterpret_cast<int32_t*>(dst + x + 2 * dst_stride) =
          _mm_cvtsi128_si32(out23);
        __m128i out3 = _mm_srli_si128(out23, 4);
        *reinterpret_cast<int32_t*>(dst + x + 3 * dst_stride) =
          _mm_cvtsi128_si32(out3);
      }
    }
#if !XVC_HIGH_BITDEPTH
    if (width & 2) {
      // Special handling of non 4-byte aligned stores
      // This happens for non high bitdepth builds and when CU width is 2
      for (int y2 = 0; y2 < 4; y2++) {
        for (int x2 = 0; x2 < 2; x2++) {
          int sum = 0;
          sum += src[width_reduced + x2 + (y2 + 0) * src_stride] * filter[0];
          sum += src[width_reduced + x2 + (y2 + 1) * src_stride] * filter[1];
          sum += src[width_reduced + x2 + (y2 + 2) * src_stride] * filter[2];
          sum += src[width_reduced + x2 + (y2 + 3) * src_stride] * filter[3];
          int val = (sum + offset) >> shift;
          if (Clip) {
            dst[y2 * dst_stride + width_reduced + x2] =
              util::ClipBD(val, (1 << bitdepth) - 1);
          } else {
            dst[y2 * dst_stride + width_reduced + x2] =
              static_cast<DstT>(val);
          }
        }
      }
    }
#endif
    src += src_stride * 4;
    dst += dst_stride * 4;
  }
  if (height & 2) {
    for (int x = 0; x < width_reduced; x += 4) {
      __m128i row0 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 0 * src_stride));
      __m128i row1 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 1 * src_stride));
      __m128i row2 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 2 * src_stride));
      __m128i row3 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 3 * src_stride));
      __m128i row4 = _mm_loadl_epi64(CAST_M128_CONST(src + x + 4 * src_stride));
      __m128i row01 = combine_rows_epi16(row0, row1);
      __m128i row12 = combine_rows_epi16(row1, row2);
      __m128i row23 = combine_rows_epi16(row2, row3);
      __m128i row34 = combine_rows_epi16(row3, row4);
      __m128i sum0 = filter_4rows_epi32(row01, row23);
      __m128i sum1 = filter_4rows_epi32(row12, row34);
      __m128i sum01 = _mm_packs_epi32(sum0, sum1);
      if (std::is_same<DstT, uint16_t>::value ||
          std::is_same<DstT, int16_t>::value) {
        __m128i out01 =
          Clip ? _mm_max_epi16(min, _mm_min_epi16(sum01, max)) : sum01;
        _mm_storel_epi64(
          reinterpret_cast<__m128i*>(dst + x + 0 * dst_stride), out01);
        __m128i out1 = _mm_shuffle_epi32(out01, kBin8_01_00_11_10);
        _mm_storel_epi64(
          reinterpret_cast<__m128i*>(dst + x + 1 * dst_stride), out1);
      } else if (std::is_same<DstT, uint8_t>::value) {
        __m128i out01 = _mm_packus_epi16(sum01, sum01);
        *reinterpret_cast<int32_t*>(dst + x + 0 * dst_stride) =
          _mm_cvtsi128_si32(out01);
        __m128i out1 = _mm_srli_si128(out01, 4);
        *reinterpret_cast<int32_t*>(dst + x + 1 * dst_stride) =
          _mm_cvtsi128_si32(out1);
      }
    }
#if !XVC_HIGH_BITDEPTH
    if (width & 2) {
      // Special handling of non 4-byte aligned stores
      // This happens for non high bitdepth builds and when CU width is 2
      for (int y2 = 0; y2 < 2; y2++) {
        for (int x2 = 0; x2 < 2; x2++) {
          int sum = 0;
          sum += src[width_reduced + x2 + (y2 + 0) * src_stride] * filter[0];
          sum += src[width_reduced + x2 + (y2 + 1) * src_stride] * filter[1];
          sum += src[width_reduced + x2 + (y2 + 2) * src_stride] * filter[2];
          sum += src[width_reduced + x2 + (y2 + 3) * src_stride] * filter[3];
          int val = (sum + offset) >> shift;
          if (Clip) {
            dst[y2 * dst_stride + width_reduced + x2] =
              util::ClipBD(val, (1 << bitdepth) - 1);
          } else {
            dst[y2 * dst_stride + width_reduced + x2] =
              static_cast<DstT>(val);
          }
        }
      }
    }
#endif
  }
}
#endif  // XVC_ARCH_X86

#if XVC_HAVE_NEON
template<typename SrcT, typename DstT, bool Clip>
static
void FilterVerChromaNeon(int width, int height, int bitdepth,
                         const int16_t *filter,
                         const SrcT *src, ptrdiff_t src_stride,
                         DstT *dst, ptrdiff_t dst_stride) {
  const int height4 = height & ~3;
  const int shift = InterPrediction::GetFilterShift<SrcT, Clip>(bitdepth);
  const int offset = InterPrediction::GetFilterOffset<SrcT, Clip>(shift);
  const int32x4_t vshift = vdupq_n_s32(static_cast<int32_t>(shift) * -1);
  const int32x4_t voffset = vdupq_n_s32(static_cast<int32_t>(offset));
  const int16x8_t min = vdupq_n_s16(0);
  const int16x8_t max = vdupq_n_s16((1 << bitdepth) - 1);
  static_assert(InterPrediction::kNumTapsChroma == 4, "4 tap filter");
  const int16x4_t vfilter0 = vdup_n_s16(filter[0]);
  const int16x4_t vfilter1 = vdup_n_s16(filter[1]);
  const int16x4_t vfilter2 = vdup_n_s16(filter[2]);
  const int16x4_t vfilter3 = vdup_n_s16(filter[3]);
  auto load_row_int16x4 = [](const SrcT *src2) {
    if (std::is_same<SrcT, uint16_t>::value ||
        std::is_same<SrcT, int16_t>::value) {
      return vld1_s16(reinterpret_cast<const int16_t*>(src2));
    } else if (std::is_same<SrcT, uint8_t>::value) {
      // TODO(PH) Loading twice as much data as needed
      auto *p = reinterpret_cast<const uint8_t*>(src2);
      return vreinterpret_s16_u16(vget_low_u16(vmovl_u8(vld1_u8(p))));
    } else {
      assert(0);
      return int16x4_t();
    }
  };
  auto fir_4tap = [&](int16x4_t r0, int16x4_t r1, int16x4_t r2, int16x4_t r3) {
    int32x4_t prod0 = vmull_s16(r0, vfilter0);
    prod0 = vmlal_s16(prod0, r1, vfilter1);
    prod0 = vmlal_s16(prod0, r2, vfilter2);
    prod0 = vmlal_s16(prod0, r3, vfilter3);
    int32x4_t sum0_offset = vaddq_s32(prod0, voffset);
    int32x4_t sum0 = vshlq_s32(sum0_offset, vshift);
    return vqmovn_s32(sum0);
  };

  src -= (InterPrediction::kNumTapsChroma / 2 - 1) * src_stride;

  for (int y = 0; y < height4; y += 4) {
    for (int x = 0; x < width; x += 4) {
      int16x4_t row0 = load_row_int16x4(src + x + 0 * src_stride);
      int16x4_t row1 = load_row_int16x4(src + x + 1 * src_stride);
      int16x4_t row2 = load_row_int16x4(src + x + 2 * src_stride);
      int16x4_t row3 = load_row_int16x4(src + x + 3 * src_stride);
      int16x4_t row4 = load_row_int16x4(src + x + 4 * src_stride);
      int16x4_t row5 = load_row_int16x4(src + x + 5 * src_stride);
      int16x4_t row6 = load_row_int16x4(src + x + 6 * src_stride);
      int16x4_t sum0 = fir_4tap(row0, row1, row2, row3);
      int16x4_t sum1 = fir_4tap(row1, row2, row3, row4);
      int16x4_t sum2 = fir_4tap(row2, row3, row4, row5);
      int16x4_t sum3 = fir_4tap(row3, row4, row5, row6);
      if (!Clip) {
        // Replace DstT reinterpret_cast with C++17 if constexpr
        vst1_s16(reinterpret_cast<int16_t*>(dst + x + 0 * dst_stride), sum0);
        vst1_s16(reinterpret_cast<int16_t*>(dst + x + 1 * dst_stride), sum1);
        vst1_s16(reinterpret_cast<int16_t*>(dst + x + 2 * dst_stride), sum2);
        vst1_s16(reinterpret_cast<int16_t*>(dst + x + 3 * dst_stride), sum3);
      } else if (std::is_same<DstT, uint16_t>::value ||
                 std::is_same<DstT, int16_t>::value) {
        int16x8_t sum01 = vcombine_s16(sum0, sum1);
        int16x8_t sum23 = vcombine_s16(sum2, sum3);
        int16x8_t out01 = vmaxq_s16(min, vminq_s16(sum01, max));
        int16x8_t out23 = vmaxq_s16(min, vminq_s16(sum23, max));
        vst1_u16(reinterpret_cast<uint16_t*>(dst + x + 0 * dst_stride),
                 vreinterpret_u16_s16(vget_low_s16(out01)));
        vst1_u16(reinterpret_cast<uint16_t*>(dst + x + 1 * dst_stride),
                 vreinterpret_u16_s16(vget_high_s16(out01)));
        vst1_u16(reinterpret_cast<uint16_t*>(dst + x + 2 * dst_stride),
                 vreinterpret_u16_s16(vget_low_s16(out23)));
        vst1_u16(reinterpret_cast<uint16_t*>(dst + x + 3 * dst_stride),
                 vreinterpret_u16_s16(vget_high_s16(out23)));
      } else if (std::is_same<DstT, uint8_t>::value) {
        int16x8_t sum01 = vcombine_s16(sum0, sum1);
        int16x8_t sum23 = vcombine_s16(sum2, sum3);
        uint32x2_t out01 = vreinterpret_u32_u8(vqmovun_s16(sum01));
        uint32x2_t out23 = vreinterpret_u32_u8(vqmovun_s16(sum23));
        vst1_lane_u32(reinterpret_cast<uint32_t*>(dst + x + 0 * dst_stride),
                      out01, 0);
        vst1_lane_u32(reinterpret_cast<uint32_t*>(dst + x + 1 * dst_stride),
                      out01, 1);
        vst1_lane_u32(reinterpret_cast<uint32_t*>(dst + x + 2 * dst_stride),
                      out23, 0);
        vst1_lane_u32(reinterpret_cast<uint32_t*>(dst + x + 3 * dst_stride),
                      out23, 1);
      }
    }
    src += src_stride * 4;
    dst += dst_stride * 4;
  }
  if (height & 3) {
    for (int x = 0; x < width; x += 4) {
      int16x4_t row0 = load_row_int16x4(src + x + 0 * src_stride);
      int16x4_t row1 = load_row_int16x4(src + x + 1 * src_stride);
      int16x4_t row2 = load_row_int16x4(src + x + 2 * src_stride);
      int16x4_t row3 = load_row_int16x4(src + x + 3 * src_stride);
      int16x4_t row4 = load_row_int16x4(src + x + 4 * src_stride);
      int16x4_t sum0 = fir_4tap(row0, row1, row2, row3);
      int16x4_t sum1 = fir_4tap(row1, row2, row3, row4);
      if (!Clip) {
        // Replace DstT reinterpret_cast with C++17 if constexpr
        vst1_s16(reinterpret_cast<int16_t*>(dst + x + 0 * dst_stride), sum0);
        vst1_s16(reinterpret_cast<int16_t*>(dst + x + 1 * dst_stride), sum1);
      } else if (std::is_same<DstT, uint16_t>::value ||
                 std::is_same<DstT, int16_t>::value) {
        int16x8_t sum01 = vcombine_s16(sum0, sum1);
        int16x8_t out01 = vmaxq_s16(min, vminq_s16(sum01, max));
        vst1_u16(reinterpret_cast<uint16_t*>(dst + x + 0 * dst_stride),
                 vreinterpret_u16_s16(vget_low_s16(out01)));
        vst1_u16(reinterpret_cast<uint16_t*>(dst + x + 1 * dst_stride),
                 vreinterpret_u16_s16(vget_high_s16(out01)));
      } else if (std::is_same<DstT, uint8_t>::value) {
        int16x8_t sum01 = vcombine_s16(sum0, sum1);
        uint32x2_t out01 = vreinterpret_u32_u8(vqmovun_s16(sum01));
        vst1_lane_u32(reinterpret_cast<uint32_t*>(dst + x + 0 * dst_stride),
                      out01, 0);
        vst1_lane_u32(reinterpret_cast<uint32_t*>(dst + x + 1 * dst_stride),
                      out01, 1);
      }
    }
  }
}
#endif  // XVC_HAVE_NEON

#if XVC_ARCH_ARM
void InterPredictionSimd::Register(const std::set<CpuCapability> &caps,
                                   xvc::SimdFunctions *simd_functions) {
#if XVC_HAVE_NEON
  auto &ip = simd_functions->inter_prediction;
  if (caps.find(CpuCapability::kNeon) != caps.end()) {
    ip.add_avg[1] = &AddAvgNeon;
    ip.filter_copy_bipred[1] = &FilterCopyBipredNeon;
    ip.filter_h_sample_sample[0] = &FilterHorSampleTLumaNeon<Sample, true>;
    ip.filter_h_sample_sample[1] = &FilterHorSampleTChromaNeon<Sample, true>;
    ip.filter_h_sample_short[0] = &FilterHorSampleTLumaNeon<int16_t, false>;
    ip.filter_h_sample_short[1] = &FilterHorSampleTChromaNeon<int16_t, false>;
    ip.filter_v_sample_sample[0] = &FilterVerLumaNeon<Sample, Sample, true>;
    ip.filter_v_sample_sample[1] = &FilterVerChromaNeon<Sample, Sample, true>;
    ip.filter_v_sample_short[0] = &FilterVerLumaNeon<Sample, int16_t, false>;
    ip.filter_v_sample_short[1] = &FilterVerChromaNeon<Sample, int16_t, false>;
    ip.filter_v_short_sample[0] = &FilterVerLumaNeon<int16_t, Sample, true>;
    ip.filter_v_short_sample[1] = &FilterVerChromaNeon<int16_t, Sample, true>;
    ip.filter_v_short_short[0] = &FilterVerLumaNeon<int16_t, int16_t, false>;
    ip.filter_v_short_short[1] = &FilterVerChromaNeon<int16_t, int16_t, false>;
  }
#endif  // XVC_HAVE_NEON
}
#endif  // XVC_ARCH_ARM

#if XVC_ARCH_X86
void InterPredictionSimd::Register(const std::set<CpuCapability> &caps,
                                   xvc::SimdFunctions *simd_functions) {
  auto &ip = simd_functions->inter_prediction;
  if (caps.find(CpuCapability::kSse2) != caps.end()) {
    ip.add_avg[1] = &AddAvgSse2;
    ip.filter_copy_bipred[1] = &FilterCopyBipredSse2;
    ip.filter_h_sample_sample[0] = &FilterHorSampleTLumaSse2<Sample, true>;
    ip.filter_h_sample_sample[1] = &FilterHorSampleTChromaSse2<Sample, true>;
    ip.filter_h_sample_short[0] = &FilterHorSampleTLumaSse2<int16_t, false>;
    ip.filter_h_sample_short[1] = &FilterHorSampleTChromaSse2<int16_t, false>;
    ip.filter_v_sample_sample[0] = &FilterVerLumaSse2<Sample, Sample, true>;
    ip.filter_v_sample_sample[1] = &FilterVerChromaSse2<Sample, Sample, true>;
    ip.filter_v_sample_short[0] = &FilterVerLumaSse2<Sample, int16_t, false>;
    ip.filter_v_sample_short[1] = &FilterVerChromaSse2<Sample, int16_t, false>;
    ip.filter_v_short_sample[0] = &FilterVerLumaSse2<int16_t, Sample, true>;
    ip.filter_v_short_sample[1] = &FilterVerChromaSse2<int16_t, Sample, true>;
    ip.filter_v_short_short[0] = &FilterVerLumaSse2<int16_t, int16_t, false>;
    ip.filter_v_short_short[1] = &FilterVerChromaSse2<int16_t, int16_t, false>;
  }
}
#endif  // XVC_ARCH_X86

#if XVC_ARCH_MIPS
void InterPredictionSimd::Register(const std::set<CpuCapability> &caps,
                                   xvc::SimdFunctions *simd_functions) {
}
#endif  // XVC_ARCH_MIPS

}   // namespace simd
}   // namespace xvc
