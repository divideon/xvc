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

constexpr int kBin8_01_00_11_10 = 0x4E;
constexpr int kBin8_10_11_00_01 = 0xB1;
constexpr int kBin8_11_01_10_00 = 0xD8;

#if XVC_ARCH_X86
__attribute__((target("sse2")))
static void AddAvgSse2(int width, int height, int offset, int shift,
                       int bitdepth, const int16_t *src1, intptr_t stride1,
                       const int16_t *src2, intptr_t stride2,
                       Sample *dst, intptr_t dst_stride) {
#if XVC_HIGH_BITDEPTH
  __m128i min = _mm_set1_epi16(0);
  __m128i max = _mm_set1_epi16((1 << bitdepth) - 1);
#endif
  __m128i s3 = _mm_set1_epi16((int16_t)offset);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x += 8) {
      __m128i s1 = _mm_loadu_si128(CAST_M128_CONST(src1 + x));
      __m128i s2 = _mm_loadu_si128(CAST_M128_CONST(src2 + x));
      __m128i sum =
        _mm_srai_epi16(_mm_adds_epi16(_mm_add_epi16(s1, s2), s3), shift);
#if XVC_HIGH_BITDEPTH
      __m128i out = _mm_max_epi16(min, _mm_min_epi16(sum, max));
      _mm_storeu_si128(reinterpret_cast<__m128i*>(dst + x), out);
#else
      __m128i out = _mm_packus_epi16(sum, sum);
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
static void AddAvgNeon(int width, int height, int offset, int shift,
                       int bitdepth, const int16_t *src1, intptr_t stride1,
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

#if XVC_ARCH_X86
__attribute__((target("sse2")))
static void FilterCopyBipredSse2(int width, int height,
                                 int16_t offset, int shift,
                                 const Sample *ref, ptrdiff_t ref_stride,
                                 int16_t *dst, ptrdiff_t dst_stride) {
  __m128i vec_offset = _mm_set1_epi16(offset);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x += 8) {
#if XVC_HIGH_BITDEPTH
      __m128i s1 = _mm_loadu_si128(CAST_M128_CONST(ref + x));
#else
      __m128i vec_ref = _mm_loadl_epi64(CAST_M128_CONST(ref + x));
      __m128i s1 = _mm_unpacklo_epi8(vec_ref, _mm_setzero_si128());
#endif
      __m128i out = _mm_sub_epi16(_mm_slli_epi16(s1, shift), vec_offset);
      _mm_storeu_si128(reinterpret_cast<__m128i*>(dst + x), out);
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
  int16x8_t vec_shift = vdupq_n_s16((int16_t)shift);
  int16x8_t vec_offset = vdupq_n_s16(offset);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x += 8) {
#if XVC_HIGH_BITDEPTH
      int16x8_t s1 = vreinterpretq_s16_u16(vld1q_u16(ref + x));
#else
      uint8x8_t vec_ref = vld1_u8(ref + x);
      int16x8_t s1 = vreinterpretq_s16_u16(vmovl_u8(vec_ref));
#endif
      int16x8_t out = vsubq_s16(vshlq_s16(s1, vec_shift), vec_offset);
      vst1q_s16(dst + x, out);
    }
    ref += ref_stride;
    dst += dst_stride;
  }
}
#endif

#if XVC_ARCH_X86
__attribute__((target("sse2")))
static
void FilterHorSampleSampleLumaSse2(int width, int height, int bitdepth,
                                   const int16_t *filter,
                                   const Sample *src, ptrdiff_t src_stride,
                                   Sample *dst, ptrdiff_t dst_stride) {
  const int shift = InterPrediction::kFilterPrecision;
  const int offset = 1 << (shift - 1);
  static_assert(InterPrediction::kNumTapsLuma == 8, "8 tap filter");
  const __m128i voffset = _mm_set1_epi32(static_cast<int16_t>(offset));
  const __m128i vfilter = _mm_loadu_si128(CAST_M128_CONST(filter));
#if XVC_HIGH_BITDEPTH
  const __m128i min = _mm_set1_epi16(0);
  const __m128i max = _mm_set1_epi16((1 << bitdepth) - 1);
#endif

  src -= InterPrediction::kNumTapsLuma / 2 - 1;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x += 8) {
      auto filter_2sample_lo = [&vfilter](const Sample *sample)
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

      __m128i prod_ab_lo = filter_2sample_lo(src + x + 0);
      __m128i prod_cd_lo = filter_2sample_lo(src + x + 2);
      __m128i prod_abcd = _mm_unpacklo_epi64(prod_ab_lo, prod_cd_lo);
      __m128i sum_abcd_offset = _mm_add_epi32(prod_abcd, voffset);
      __m128i sum_abcd = _mm_srai_epi32(sum_abcd_offset, shift);

      __m128i prod_ef_lo = filter_2sample_lo(src + x + 4);
      __m128i prod_gh_lo = filter_2sample_lo(src + x + 6);
      __m128i prod_efgh = _mm_unpacklo_epi64(prod_ef_lo, prod_gh_lo);
      __m128i sum_efgh_offset = _mm_add_epi32(prod_efgh, voffset);
      __m128i sum_efgh = _mm_srai_epi32(sum_efgh_offset, shift);

      __m128i sum = _mm_packs_epi32(sum_abcd, sum_efgh);
#if XVC_HIGH_BITDEPTH
      __m128i out = _mm_max_epi16(min, _mm_min_epi16(sum, max));
      _mm_storeu_si128(reinterpret_cast<__m128i*>(dst + x), out);
#else
      __m128i out = _mm_packus_epi16(sum, sum);
      _mm_storel_epi64(reinterpret_cast<__m128i*>(dst + x), out);
#endif
    }
    src += src_stride;
    dst += dst_stride;
  }
}
#endif  // XVC_ARCH_X86

#if XVC_HAVE_NEON
static
void FilterHorSampleSampleLumaNeon(int width, int height, int bitdepth,
                                   const int16_t *filter,
                                   const Sample *src, ptrdiff_t src_stride,
                                   Sample *dst, ptrdiff_t dst_stride) {
  const int shift = InterPrediction::kFilterPrecision;
  const int offset = 1 << (shift - 1);
  const int32x4_t vshift = vdupq_n_s32((int16_t)shift * -1);
  const int32x4_t voffset = vdupq_n_s32(offset);
  static_assert(InterPrediction::kNumTapsLuma == 8, "8 tap filter");
  const int16x8_t vfilter = vld1q_s16(filter);
  const int16x4_t vfilter_lo = vget_low_s16(vfilter);
  const int16x4_t vfilter_hi = vget_high_s16(vfilter);
#if XVC_HIGH_BITDEPTH
  int16x8_t min = vdupq_n_s16(0);
  int16x8_t max = vdupq_n_s16((1 << bitdepth) - 1);
#endif

  src -= InterPrediction::kNumTapsLuma / 2 - 1;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x += 8) {
      auto filter_4sample = [&](const Sample *ref4) {
        auto filter_2sample = [&](const Sample *ref2) {
          auto filter_1sample_lo = [&](const Sample *ref1) {
#if XVC_HIGH_BITDEPTH
            int16x8_t vref_a = vreinterpretq_s16_u16(vld1q_u16(ref1));
#else
            uint8x8_t vref_a8 = vld1_u8(ref2);
            int16x8_t vref_a = vreinterpretq_s16_u16(vmovl_u8(vref_a8));
#endif
            int32x4_t prod_a_lo = vmull_s16(vget_low_s16(vref_a), vfilter_lo);
            int32x4_t prod_a_hi = vmull_s16(vget_high_s16(vref_a), vfilter_hi);
            int32x4_t prod_a = vaddq_s32(prod_a_lo, prod_a_hi);
            return vadd_s32(vget_low_s16(prod_a), vget_high_s16(prod_a));
          };
          int32x2_t sum_a_lo = filter_1sample_lo(ref2 + 0);  // a0426 a1537
          int32x2_t sum_b_lo = filter_1sample_lo(ref2 + 1);  // b0426 b1537
          return vpadd_s32(sum_a_lo, sum_b_lo);
        };
        int32x2_t prod_ab = filter_2sample(ref4 + 0);
        int32x2_t prod_cd = filter_2sample(ref4 + 2);
        int32x4_t prod_abcd = vcombine_s32(prod_ab, prod_cd);
        int32x4_t sum_abcd_offset = vaddq_s32(prod_abcd, voffset);
        return vqmovn_s32(vshlq_s32(sum_abcd_offset, vshift));
      };
      int16x4_t sum_abcd = filter_4sample(src + x + 0);
      int16x4_t sum_efgh = filter_4sample(src + x + 4);
      int16x8_t sum = vcombine_s16(sum_abcd, sum_efgh);
#if XVC_HIGH_BITDEPTH
      int16x8_t out = vmaxq_s16(min, vminq_s16(sum, max));
      vst1q_u16(dst + x, vreinterpretq_u16_s16(out));
#else
      uint8x8_t out = vqmovun_s16(sum);
      vst1_u8(dst + x, out);
#endif
    }
    src += src_stride;
    dst += dst_stride;
  }
}
#endif  // XVC_HAVE_NEON

#if XVC_ARCH_X86
__attribute__((target("sse2")))
static
void FilterVerSampleSampleLumaSse2(int width, int height, int bitdepth,
                                   const int16_t *filter,
                                   const Sample *src, ptrdiff_t src_stride,
                                   Sample *dst, ptrdiff_t dst_stride) {
  const int shift = InterPrediction::kFilterPrecision;
  const int offset = 1 << (shift - 1);
  const __m128i voffset = _mm_set1_epi32(static_cast<int16_t>(offset));
#if XVC_HIGH_BITDEPTH
  const __m128i min = _mm_set1_epi16(0);
  const __m128i max = _mm_set1_epi16((1 << bitdepth) - 1);
#endif
  static_assert(InterPrediction::kNumTapsLuma == 8, "8 tap filter");
  const __m128i vfilter01 =
    _mm_set1_epi32((filter[0] & 0xFFFF) | (filter[1] << 16));
  const __m128i vfilter23 =
    _mm_set1_epi32((filter[2] & 0xFFFF) | (filter[3] << 16));
  const __m128i vfilter45 =
    _mm_set1_epi32((filter[4] & 0xFFFF) | (filter[5] << 16));
  const __m128i vfilter67 =
    _mm_set1_epi32((filter[6] & 0xFFFF) | (filter[7] << 16));

  src -= (InterPrediction::kNumTapsLuma / 2 - 1) * src_stride;

  for (int y = 0; y < height; y += 4) {
    for (int x = 0; x < width; x += 4) {
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
#if XVC_HIGH_BITDEPTH
      __m128i row01 = _mm_unpacklo_epi16(row0, row1);
      __m128i row12 = _mm_unpacklo_epi16(row1, row2);
      __m128i row23 = _mm_unpacklo_epi16(row2, row3);
      __m128i row34 = _mm_unpacklo_epi16(row3, row4);
      __m128i row45 = _mm_unpacklo_epi16(row4, row5);
      __m128i row56 = _mm_unpacklo_epi16(row5, row6);
      __m128i row67 = _mm_unpacklo_epi16(row6, row7);
      __m128i row78 = _mm_unpacklo_epi16(row7, row8);
      __m128i row89 = _mm_unpacklo_epi16(row8, row9);
      __m128i row90 = _mm_unpacklo_epi16(row9, row10);
#else
      // TODO(PH) Loaded twice as much data as needed
      __m128i zero = _mm_setzero_si128();
      __m128i row01 = _mm_unpacklo_epi8(_mm_unpacklo_epi8(row0, row1), zero);
      __m128i row12 = _mm_unpacklo_epi8(_mm_unpacklo_epi8(row1, row2), zero);
      __m128i row23 = _mm_unpacklo_epi8(_mm_unpacklo_epi8(row2, row3), zero);
      __m128i row34 = _mm_unpacklo_epi8(_mm_unpacklo_epi8(row3, row4), zero);
      __m128i row45 = _mm_unpacklo_epi8(_mm_unpacklo_epi8(row4, row5), zero);
      __m128i row56 = _mm_unpacklo_epi8(_mm_unpacklo_epi8(row5, row6), zero);
      __m128i row67 = _mm_unpacklo_epi8(_mm_unpacklo_epi8(row6, row7), zero);
      __m128i row78 = _mm_unpacklo_epi8(_mm_unpacklo_epi8(row7, row8), zero);
      __m128i row89 = _mm_unpacklo_epi8(_mm_unpacklo_epi8(row8, row9), zero);
      __m128i row90 = _mm_unpacklo_epi8(_mm_unpacklo_epi8(row9, row10), zero);
#endif

      auto filter_8rows_lo = [&](__m128i data01, __m128i data23,
                                 __m128i data56, __m128i data67)
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

      __m128i sum0 = filter_8rows_lo(row01, row23, row45, row67);
      __m128i sum1 = filter_8rows_lo(row12, row34, row56, row78);
      __m128i sum2 = filter_8rows_lo(row23, row45, row67, row89);
      __m128i sum3 = filter_8rows_lo(row34, row56, row78, row90);

      __m128i sum01 = _mm_packs_epi32(sum0, sum1);
      __m128i sum23 = _mm_packs_epi32(sum2, sum3);

#if XVC_HIGH_BITDEPTH
      __m128i out01 = _mm_max_epi16(min, _mm_min_epi16(sum01, max));
      __m128i out23 = _mm_max_epi16(min, _mm_min_epi16(sum23, max));
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
#else
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
#endif
    }
    src += src_stride * 4;
    dst += dst_stride * 4;
  }
}
#endif  // XVC_ARCH_X86

#if XVC_HAVE_NEON
static
void FilterVerSampleSampleLumaNeon(int width, int height, int bitdepth,
                                   const int16_t *filter,
                                   const Sample *src, ptrdiff_t src_stride,
                                   Sample *dst, ptrdiff_t dst_stride) {
  const int shift = InterPrediction::kFilterPrecision;
  const int offset = 1 << (shift - 1);
  const int32x4_t vshift = vdupq_n_s32((int16_t)shift * -1);
  const int32x4_t voffset = vdupq_n_s32(offset);
#if XVC_HIGH_BITDEPTH
  int16x8_t min = vdupq_n_s16(0);
  int16x8_t max = vdupq_n_s16((1 << bitdepth) - 1);
#endif
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
#if XVC_HIGH_BITDEPTH
      auto load_row = [](const Sample *src2) {
        return vreinterpret_s16_u16(vld1_u16(src2));
      };
#else
      auto load_row = [](const Sample *src2) {
        // TODO(PH) Loading twice as much data as needed
        return vreinterpret_s16_u16(vget_low_u16(vmovl_u8(vld1_u8(src2))));
      };
#endif
      int16x4_t row0 = load_row(src + x + 0 * src_stride);
      int16x4_t row1 = load_row(src + x + 1 * src_stride);
      int16x4_t row2 = load_row(src + x + 2 * src_stride);
      int16x4_t row3 = load_row(src + x + 3 * src_stride);
      int16x4_t row4 = load_row(src + x + 4 * src_stride);
      int16x4_t row5 = load_row(src + x + 5 * src_stride);
      int16x4_t row6 = load_row(src + x + 6 * src_stride);
      int16x4_t row7 = load_row(src + x + 7 * src_stride);
      int16x4_t row8 = load_row(src + x + 8 * src_stride);
      int16x4_t row9 = load_row(src + x + 9 * src_stride);
      int16x4_t row10 = load_row(src + x + 10 * src_stride);
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
      int16x4_t sum0 = fir_8r(row0, row1, row2, row3, row4, row5, row6, row7);
      int16x4_t sum1 = fir_8r(row1, row2, row3, row4, row5, row6, row7, row8);
      int16x4_t sum2 = fir_8r(row2, row3, row4, row5, row6, row7, row8, row9);
      int16x4_t sum3 = fir_8r(row3, row4, row5, row6, row7, row8, row9, row10);
      int16x8_t sum01 = vcombine_s16(sum0, sum1);
      int16x8_t sum23 = vcombine_s16(sum2, sum3);

#if XVC_HIGH_BITDEPTH
      int16x8_t out01 = vmaxq_s16(min, vminq_s16(sum01, max));
      int16x8_t out23 = vmaxq_s16(min, vminq_s16(sum23, max));
      vst1_u16(dst + x + 0 * dst_stride,
               vreinterpret_u16_s16(vget_low_s16(out01)));
      vst1_u16(dst + x + 1 * dst_stride,
               vreinterpret_u16_s16(vget_high_s16(out01)));
      vst1_u16(dst + x + 2 * dst_stride,
               vreinterpret_u16_s16(vget_low_s16(out23)));
      vst1_u16(dst + x + 3 * dst_stride,
               vreinterpret_u16_s16(vget_high_s16(out23)));
#else
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
#endif
    }
    src += src_stride * 4;
    dst += dst_stride * 4;
  }
}
#endif  // XVC_HAVE_NEON


#if XVC_ARCH_ARM
void InterPredictionSimd::Register(const std::set<CpuCapability> &caps,
                                   xvc::SimdFunctions *simd_functions) {
#if XVC_HAVE_NEON
  auto &simd = simd_functions->inter_prediction;
  if (caps.find(CpuCapability::kNeon) != caps.end()) {
    simd.add_avg[1] = &AddAvgNeon;
    simd.filter_copy_bipred[1] = &FilterCopyBipredNeon;
    simd.filter_h_sample_sample[0] = &FilterHorSampleSampleLumaNeon;
    simd.filter_v_sample_sample[0] = &FilterVerSampleSampleLumaNeon;
  }
#endif  // XVC_HAVE_NEON
}
#endif  // XVC_ARCH_ARM

#if XVC_ARCH_X86
void InterPredictionSimd::Register(const std::set<CpuCapability> &caps,
                                   xvc::SimdFunctions *simd_functions) {
  auto &simd = simd_functions->inter_prediction;
  if (caps.find(CpuCapability::kSse2) != caps.end()) {
    simd.add_avg[1] = &AddAvgSse2;
    simd.filter_copy_bipred[1] = &FilterCopyBipredSse2;
    simd.filter_h_sample_sample[0] = &FilterHorSampleSampleLumaSse2;
    simd.filter_v_sample_sample[0] = &FilterVerSampleSampleLumaSse2;
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
