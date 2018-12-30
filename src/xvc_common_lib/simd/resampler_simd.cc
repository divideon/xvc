/******************************************************************************
* Copyright (C) 2018, Divideon.
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
* This library is also available under a commercial license.
* Please visit https://xvc.io/license/ for more information.
******************************************************************************/

#include "xvc_common_lib/simd/resampler_simd.h"

#ifdef XVC_ARCH_X86
#include <immintrin.h>    // AVX2
#endif  // XVC_ARCH_X86

#ifdef XVC_HAVE_NEON
#include <arm_neon.h>
#endif  // XVC_HAVE_NEON

#include <type_traits>

#include "xvc_common_lib/simd_functions.h"
#include "xvc_common_lib/resample.h"

#ifdef _MSC_VER
#define __attribute__(SPEC)
#endif  // _MSC_VER

#ifdef XVC_ARCH_X86
// Formatting helper
#define CAST_M128_CONST(VAL) reinterpret_cast<const __m128i*>((VAL))
#endif  // XVC_ARCH_X86

namespace xvc {
namespace simd {

#if XVC_HAVE_NEON && XVC_HIGH_BITDEPTH
static void CopySampleToByteNeon(int width, int height,
                                 const Sample *src, ptrdiff_t src_stride,
                                 uint8_t *out, ptrdiff_t out_stride) {
  const int width8 = width & ~7;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width8; x += 8) {
      vst1_u8(out + x, vmovn_u16(vld1q_u16(src + x)));
    }
    if ((width & 7) != 0) {
      for (int x = width8; x < width; x++) {
        out[x] = static_cast<uint8_t>(src[x]);
      }
    }
    src += src_stride;
    out += out_stride;
  }
}
#endif

#if XVC_HAVE_NEON && XVC_HIGH_BITDEPTH
static
void DownshiftSampleToByteFastNeon(int width, int height, int shift,
                                   int out_bitdepth,
                                   const Sample *src, ptrdiff_t src_stride,
                                   uint8_t *out, ptrdiff_t out_stride) {
  const int width8 = width & ~7;
  const int add = 1 << (shift - 1);
  const int16x8_t vshift = vdupq_n_s16(static_cast<int16_t>(shift * -1));
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width8; x += 8) {
      uint16x8_t vsrc = vld1q_u16(src + x);
      uint8x8_t vtmp = vmovn_u16(vrshlq_u16(vsrc, vshift));
      vst1_u8(out + x, vtmp);
    }
    if ((width & 7) != 0) {
      for (int x = width8; x < width; x++) {
        out[x] = static_cast<uint8_t>((src[x] + add) >> shift);
      }
    }
    src += src_stride;
    out += out_stride;
  }
}
#endif

#if XVC_HAVE_NEON && XVC_HIGH_BITDEPTH
static
void DownshiftSampleToByteDitherNeon(int width, int height, int shift,
                                     int out_bitdepth,
                                     const Sample *src, ptrdiff_t src_stride,
                                     uint8_t *out, ptrdiff_t out_stride) {
  const int width8 = width & ~7;
  const int mask = (1 << shift) - 1;
  const uint16x8_t vmask = vdupq_n_u16(static_cast<uint16_t>(mask));
  const int16x8_t vshift = vdupq_n_s16(static_cast<int16_t>(shift * -1));
  uint16x8_t vsum = vdupq_n_u16(0);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width8; x += 8) {
      uint16x8_t vsrc = vld1q_u16(src + x);
      vsum = vqaddq_u16(vsum, vsrc);
      uint8x8_t vtmp = vmovn_u16(vshlq_u16(vsum, vshift));
      vst1_u8(out + x, vtmp);
      vsum = vandq_u16(vsum, vmask);
    }
    if ((width & 7) != 0) {
      int sample = 0;
      for (int x = width8; x < width; x++) {
        sample += src[x];
        out[x] = static_cast<uint8_t>(sample >> shift);
        sample &= mask;
      }
    }
    src += src_stride;
    out += out_stride;
  }
}
#endif

#ifdef XVC_ARCH_X86
#if XVC_HIGH_BITDEPTH
__attribute__((target("sse2")))
static void CopySampleToByteSse2(int width, int height,
                                 const Sample *src, ptrdiff_t src_stride,
                                 uint8_t *out, ptrdiff_t out_stride) {
  const int width8 = width & ~7;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width8; x += 8) {
      __m128i s16 = _mm_loadu_si128(CAST_M128_CONST(src + x));
      __m128i s8 = _mm_packus_epi16(s16, s16);
      _mm_storel_epi64(reinterpret_cast<__m128i*>(out + x), s8);
    }
    if ((width & 7) != 0) {
      for (int x = width8; x < width; x++) {
        out[x] = static_cast<uint8_t>(src[x]);
      }
    }
    src += src_stride;
    out += out_stride;
  }
}
#endif  // XVC_HIGH_BITDEPTH
#endif  // XVC_ARCH_X86

#ifdef XVC_ARCH_X86
#if XVC_HIGH_BITDEPTH
__attribute__((target("sse2")))
static
void DownshiftSampleToByteFastSse2(int width, int height, int shift,
                                   int out_bitdepth,
                                   const Sample *src, ptrdiff_t src_stride,
                                   uint8_t *out, ptrdiff_t out_stride) {
  const int width8 = width & ~7;
  const int add = 1 << (shift - 1);
  const __m128i vadd = _mm_set1_epi16(static_cast<int16_t>(add));
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width8; x += 8) {
      __m128i vsrc = _mm_loadu_si128(CAST_M128_CONST(src + x));
      __m128i vtmp = _mm_srai_epi16(_mm_adds_epi16(vsrc, vadd), shift);
      __m128i vout = _mm_packus_epi16(vtmp, vtmp);
      _mm_storel_epi64(reinterpret_cast<__m128i*>(out + x), vout);
    }
    if ((width & 7) != 0) {
      for (int x = width8; x < width; x++) {
        out[x] = static_cast<uint8_t>((src[x] + add) >> shift);
      }
    }
    src += src_stride;
    out += out_stride;
  }
}
#endif  // XVC_HIGH_BITDEPTH
#endif  // XVC_ARCH_X86

#ifdef XVC_ARCH_X86
#if XVC_HIGH_BITDEPTH
__attribute__((target("sse2")))
static
void DownshiftSampleToByteDitherSse2(int width, int height, int shift,
                                     int out_bitdepth,
                                     const Sample *src, ptrdiff_t src_stride,
                                     uint8_t *out, ptrdiff_t out_stride) {
  const int width8 = width & ~7;
  const int mask = (1 << shift) - 1;
  const __m128i vmask = _mm_set1_epi16(static_cast<int16_t>(mask));
  __m128i vsum = _mm_setzero_si128();
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width8; x += 8) {
      __m128i vsrc = _mm_loadu_si128(CAST_M128_CONST(src + x));
      vsum = _mm_adds_epi16(vsum, vsrc);
      __m128i vtmp = _mm_srai_epi16(vsum, shift);
      __m128i vout = _mm_packus_epi16(vtmp, vtmp);
      _mm_storel_epi64(reinterpret_cast<__m128i*>(out + x), vout);
      vsum = _mm_and_si128(vsum, vmask);
    }
    if ((width & 7) != 0) {
      int sample = 0;
      for (int x = width8; x < width; x++) {
        sample += src[x];
        out[x] = static_cast<uint8_t>(sample >> shift);
        sample &= mask;
      }
    }
    src += src_stride;
    out += out_stride;
  }
}
#endif  // XVC_HIGH_BITDEPTH
#endif  // XVC_ARCH_X86

#ifdef XVC_ARCH_ARM
void ResamplerSimd::Register(const std::set<CpuCapability> &caps,
                             xvc::SimdFunctions *simd_functions) {
#if XVC_HAVE_NEON && XVC_HIGH_BITDEPTH
  if (caps.find(CpuCapability::kNeon) != caps.end()) {
    Resampler::SimdFunc &simd = simd_functions->resampler;
    simd.copy_sample_byte = &CopySampleToByteNeon;
    simd.downshift_sample_byte[0] = &DownshiftSampleToByteFastNeon;
    simd.downshift_sample_byte[1] = &DownshiftSampleToByteDitherNeon;
  }
#endif  // XVC_HAVE_NEON
}
#endif  // XVC_ARCH_ARM

#ifdef XVC_ARCH_X86
void ResamplerSimd::Register(const std::set<CpuCapability> &caps,
                             xvc::SimdFunctions *simd_functions) {
#if XVC_HIGH_BITDEPTH
  Resampler::SimdFunc &simd = simd_functions->resampler;
  if (caps.find(CpuCapability::kSse2) != caps.end()) {
    simd.copy_sample_byte = &CopySampleToByteSse2;
    simd.downshift_sample_byte[0] = &DownshiftSampleToByteFastSse2;
    simd.downshift_sample_byte[1] = &DownshiftSampleToByteDitherSse2;
  }
#endif  // XVC_HIGH_BITDEPTH
}
#endif  // XVC_ARCH_X86

#ifdef XVC_ARCH_MIPS
void ResamplerSimd::Register(const std::set<CpuCapability> &caps,
                             xvc::SimdFunctions *simd_functions) {
}
#endif  // XVC_ARCH_MIPS

}   // namespace simd
}   // namespace xvc
