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
#include <emmintrin.h>    // SSE2
#include <immintrin.h>    // AVX2
#endif

#include <type_traits>

#include "xvc_enc_lib/encoder_simd_functions.h"
#include "xvc_enc_lib/sample_metric.h"

#ifdef _MSC_VER
#define __attribute__(SPEC)
#endif

#ifdef XVC_ARCH_X86
#define CAST_M128i_CONST(VAL) reinterpret_cast<const __m128i*>((VAL))
#define CAST_M256i_CONST(VAL) reinterpret_cast<const __m256i*>((VAL))
#endif

static const std::array<int16_t, 16> kOnes16bit = { {
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
} };

namespace xvc {
namespace simd {

#if XVC_ARCH_X86
#if XVC_HIGH_BITDEPTH
template<typename SampleT1>
__attribute__((target("sse2")))
static int ComputeSad_8x2_sse2(int width, int height,
                               const SampleT1 *src1, ptrdiff_t stride1,
                               const Sample *src2, ptrdiff_t stride2) {
  static_assert(std::is_same<Sample, uint16_t>::value, "assume high bitdepth");
  auto sad_8x1_epi16 = [](const SampleT1 *ptr1, const Sample *ptr2)
    __attribute__((target("sse2"))) {
    __m128i src1_row0 = _mm_loadu_si128(CAST_M128i_CONST(ptr1));
    __m128i src2_row0 = _mm_loadu_si128(CAST_M128i_CONST(ptr2));
    __m128i row0 = _mm_sub_epi16(src1_row0, src2_row0);
    __m128i neg0 = _mm_sub_epi16(_mm_setzero_si128(), row0);
    return _mm_max_epi16(row0, neg0);
  };  // NOLINT
  auto sad_8x2_epi32 = [&](const SampleT1 *ptr1, const Sample *ptr2)
    __attribute__((target("sse2"))) {
    __m128i abs0 = sad_8x1_epi16(ptr1, ptr2);
    __m128i abs1 = sad_8x1_epi16(ptr1 + stride1, ptr2 + stride2);
    __m128i sum01_epi16 = _mm_add_epi16(abs0, abs1);
    __m128i ones_epi16 = _mm_load_si128(CAST_M128i_CONST(&kOnes16bit[0]));
    return _mm_madd_epi16(sum01_epi16, ones_epi16);
  };  // NOLINT
  __m128i sum = _mm_setzero_si128();
  for (int y = 0; y < height; y += 2) {
    for (int x = 0; x < width; x += 8) {
      __m128i sum_epi32 = sad_8x2_epi32(src1 + x, src2 + x);
      sum = _mm_add_epi32(sum, sum_epi32);
    }
    src1 += stride1 * 2;
    src2 += stride2 * 2;
  }
  __m128i sum64_hi = _mm_shuffle_epi32(sum, _MM_SHUFFLE(1, 0, 3, 2));
  __m128i sum32 = _mm_add_epi32(sum, sum64_hi);
  __m128i sum32_hi = _mm_shufflelo_epi16(sum32, _MM_SHUFFLE(1, 0, 3, 2));
  __m128i out = _mm_add_epi32(sum32, sum32_hi);
  return _mm_cvtsi128_si32(out);
}
#endif
#endif

#if XVC_ARCH_X86
#if XVC_HIGH_BITDEPTH
template<typename SampleT1>
__attribute__((target("avx2")))
static int ComputeSad_16x2_avx2(int width, int height,
                                const SampleT1 *src1, ptrdiff_t stride1,
                                const Sample *src2, ptrdiff_t stride2) {
  static_assert(std::is_same<Sample, uint16_t>::value, "assume high bitdepth");
  auto sad_8x1_epi16 = [&](const SampleT1 *ptr1, const Sample *ptr2)
    __attribute__((target("avx2"))) {
    __m256i  src1_row0 = _mm256_lddqu_si256(CAST_M256i_CONST(ptr1));
    __m256i  src2_row0 = _mm256_lddqu_si256(CAST_M256i_CONST(ptr2));
    __m256i  row0 = _mm256_sub_epi16(src1_row0, src2_row0);
    return _mm256_abs_epi16(row0);
  };  // NOLINT
  auto sad_8x2_epi32 = [&](const SampleT1 *ptr1, const Sample *ptr2)
    __attribute__((target("avx2"))) {
    __m256i  abs0 = sad_8x1_epi16(ptr1, ptr2);
    __m256i  abs1 = sad_8x1_epi16(ptr1 + stride1, ptr2 + stride2);
    __m256i  sum01_epi16 = _mm256_add_epi16(abs0, abs1);
    __m256i ones_epi16 = _mm256_load_si256(CAST_M256i_CONST(&kOnes16bit[0]));
    return _mm256_madd_epi16(sum01_epi16, ones_epi16);
  };  // NOLINT
  __m256i  sum = _mm256_setzero_si256();
  for (int y = 0; y < height; y += 2) {
    for (int x = 0; x < width; x += 16) {
      __m256i  sum_epi32 = sad_8x2_epi32(src1 + x, src2 + x);
      sum = _mm256_add_epi32(sum, sum_epi32);
    }
    src1 += stride1 * 2;
    src2 += stride2 * 2;
  }
  __m256i vzero = _mm256_setzero_si256();
  sum = _mm256_hadd_epi32(sum, vzero);
  sum = _mm256_hadd_epi32(sum, vzero);
  __m256i sum_hi = _mm256_permute2x128_si256(sum, sum, _MM_SHUFFLE(0, 1, 0, 1));
  return _mm_cvtsi128_si32(_mm256_castsi256_si128(sum)) +
    _mm_cvtsi128_si32(_mm256_castsi256_si128(sum_hi));
}
#endif
#endif

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
  SampleMetric::SimdFunc &sm = simd_functions->sample_metric;
  // TODO(PH) Check for 16-bit samples and bitdepth <= 12
  if (caps.find(CpuCapability::kSse2) != caps.end()) {
#if XVC_HIGH_BITDEPTH
    sm.sad_sample_sample[3] = &ComputeSad_8x2_sse2<Sample>;   // 8
    sm.sad_sample_sample[4] = &ComputeSad_8x2_sse2<Sample>;   // 16
    sm.sad_sample_sample[5] = &ComputeSad_8x2_sse2<Sample>;   // 32
    sm.sad_sample_sample[6] = &ComputeSad_8x2_sse2<Sample>;   // 64
    sm.sad_short_sample[3] = &ComputeSad_8x2_sse2<int16_t>;   // 8
    sm.sad_short_sample[4] = &ComputeSad_8x2_sse2<int16_t>;   // 16
    sm.sad_short_sample[5] = &ComputeSad_8x2_sse2<int16_t>;   // 32
    sm.sad_short_sample[6] = &ComputeSad_8x2_sse2<int16_t>;   // 64
#endif
  }
  if (caps.find(CpuCapability::kAvx2) != caps.end()) {
#if XVC_HIGH_BITDEPTH
    sm.sad_sample_sample[4] = &ComputeSad_16x2_avx2<Sample>;   // 16
    sm.sad_sample_sample[5] = &ComputeSad_16x2_avx2<Sample>;   // 32
    sm.sad_sample_sample[6] = &ComputeSad_16x2_avx2<Sample>;   // 64
#endif
  }
}
#endif  // XVC_ARCH_X86

#if XVC_ARCH_MIPS
void SampleMetricSimd::Register(const std::set<CpuCapability> &caps,
                                xvc::EncoderSimdFunctions *simd_functions) {
}
#endif  // XVC_ARCH_MIPS

}   // namespace simd
}   // namespace xvc
