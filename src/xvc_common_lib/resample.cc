/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/resample.h"

#include <algorithm>
#include <vector>
#include <limits>

#include "xvc_common_lib/utils.h"

namespace xvc {

namespace resample {

static const int kFilterPrecision = 6;
static const int kInternalPrecision = 16;
static const int kPositionPrecision = 16;

static const int16_t kUpsampleFilter[16][8] = {
  { 0,  0,   0, 64,  0,   0,  0,  0 },
  { 0,  1,  -3, 63,  4,  -2,  1,  0 },
  { -1, 2,  -5, 62,  8,  -3,  1,  0 },
  { -1, 3,  -8, 60, 13,  -4,  1,  0 },
  { -1, 4, -10, 58, 17,  -5,  1,  0 },
  { -1, 4, -11, 52, 26,  -8,  3, -1 },
  { -1, 3,  -9, 47, 31, -10,  4, -1 },
  { -1, 4, -11, 45, 34, -10,  4, -1 },
  { -1, 4, -11, 40, 40, -11,  4, -1 },
  { -1, 4, -10, 34, 45, -11,  4, -1 },
  { -1, 4, -10, 31, 47,  -9,  3, -1 },
  { -1, 3,  -8, 26, 52, -11,  4, -1 },
  { 0,  1,  -5, 17, 58, -10,  4, -1 },
  { 0,  1,  -4, 13, 60,  -8,  3, -1 },
  { 0,  1,  -3,  8, 62,  -5,  2, -1 },
  { 0,  1,  -2,  4, 63,  -3,  1,  0 }
};

static const int16_t kDownsampleFilters[8][16][12] = {
  {
    {  0,   0,   0,   0,   0, 128,   0,   0,   0,   0,   0,   0 },
    {  0,   0,   0,   2,  -6, 127,   7,  -2,   0,   0,   0,   0 },
    {  0,   0,   0,   3, -12, 125,  16,  -5,   1,   0,   0,   0 },
    {  0,   0,   0,   4, -16, 120,  26,  -7,   1,   0,   0,   0 },
    {  0,   0,   0,   5, -18, 114,  36, -10,   1,   0,   0,   0 },
    {  0,   0,   0,   5, -20, 107,  46, -12,   2,   0,   0,   0 },
    {  0,   0,   0,   5, -21,  99,  57, -15,   3,   0,   0,   0 },
    {  0,   0,   0,   5, -20,  89,  68, -18,   4,   0,   0,   0 },
    {  0,   0,   0,   4, -19,  79,  79, -19,   4,   0,   0,   0 },
    {  0,   0,   0,   4, -18,  68,  89, -20,   5,   0,   0,   0 },
    {  0,   0,   0,   3, -15,  57,  99, -21,   5,   0,   0,   0 },
    {  0,   0,   0,   2, -12,  46, 107, -20,   5,   0,   0,   0 },
    {  0,   0,   0,   1, -10,  36, 114, -18,   5,   0,   0,   0 },
    {  0,   0,   0,   1,  -7,  26, 120, -16,   4,   0,   0,   0 },
    {  0,   0,   0,   1,  -5,  16, 125, -12,   3,   0,   0,   0 },
    {  0,   0,   0,   0,  -2,   7, 127,  -6,   2,   0,   0,   0 }
  },
  {
    {  0,   2,   0, -14,  33,  86,  33, -14,   0,   2,   0,   0 },
    {  0,   1,   1, -14,  29,  85,  38, -13,  -1,   2,   0,   0 },
    {  0,   1,   2, -14,  24,  84,  43, -12,  -2,   2,   0,   0 },
    {  0,   1,   2, -13,  19,  83,  48, -11,  -3,   2,   0,   0 },
    {  0,   0,   3, -13,  15,  81,  53, -10,  -4,   3,   0,   0 },
    {  0,   0,   3, -12,  11,  79,  57,  -8,  -5,   3,   0,   0 },
    {  0,   0,   3, -11,   7,  76,  62,  -5,  -7,   3,   0,   0 },
    {  0,   0,   3, -10,   3,  73,  65,  -2,  -7,   3,   0,   0 },
    {  0,   0,   3,  -9,   0,  70,  70,   0,  -9,   3,   0,   0 },
    {  0,   0,   3,  -7,  -2,  65,  73,   3, -10,   3,   0,   0 },
    {  0,   0,   3,  -7,  -5,  62,  76,   7, -11,   3,   0,   0 },
    {  0,   0,   3,  -5,  -8,  57,  79,  11, -12,   3,   0,   0 },
    {  0,   0,   3,  -4, -10,  53,  81,  15, -13,   3,   0,   0 },
    {  0,   0,   2,  -3, -11,  48,  83,  19, -13,   2,   1,   0 },
    {  0,   0,   2,  -2, -12,  43,  84,  24, -14,   2,   1,   0 },
    {  0,   0,   2,  -1, -13,  38,  85,  29, -14,   1,   1,   0 }
  },
  {
    {  0,   5,  -6, -10,  37,  76,  37, -10,  -6,   5,   0,   0 },
    {  0,   5,  -4, -11,  33,  76,  40,  -9,  -7,   5,   0,   0 },
    { -1,   5,  -3, -12,  29,  75,  45,  -7,  -8,   5,   0,   0 },
    { -1,   4,  -2, -13,  25,  75,  48,  -5,  -9,   5,   1,   0 },
    { -1,   4,  -1, -13,  22,  73,  52,  -3, -10,   4,   1,   0 },
    { -1,   4,   0, -13,  18,  72,  55,  -1, -11,   4,   2,  -1 },
    { -1,   4,   1, -13,  14,  70,  59,   2, -12,   3,   2,  -1 },
    { -1,   3,   1, -13,  11,  68,  62,   5, -12,   3,   2,  -1 },
    { -1,   3,   2, -13,   8,  65,  65,   8, -13,   2,   3,  -1 },
    { -1,   2,   3, -12,   5,  62,  68,  11, -13,   1,   3,  -1 },
    { -1,   2,   3, -12,   2,  59,  70,  14, -13,   1,   4,  -1 },
    { -1,   2,   4, -11,  -1,  55,  72,  18, -13,   0,   4,  -1 },
    {  0,   1,   4, -10,  -3,  52,  73,  22, -13,  -1,   4,  -1 },
    {  0,   1,   5,  -9,  -5,  48,  75,  25, -13,  -2,   4,  -1 },
    {  0,   0,   5,  -8,  -7,  45,  75,  29, -12,  -3,   5,  -1 },
    {  0,   0,   5,  -7,  -9,  40,  76,  33, -11,  -4,   5,   0 },
  },
  {
    {  2,  -3,  -9,   6,  39,  58,  39,  6,   -9,  -3,   2,   0 },
    {  2,  -3,  -9,   4,  38,  58,  43,  7,   -9,  -4,   1,   0 },
    {  2,  -2,  -9,   2,  35,  58,  44,  9,   -8,  -4,   1,   0 },
    {  1,  -2,  -9,   1,  34,  58,  46,  11,  -8,  -5,   1,   0 },
    {  1,  -1,  -8,  -1,  31,  57,  47,  13,  -7,  -5,   1,   0 },
    {  1,  -1,  -8,  -2,  29,  56,  49,  15,  -7,  -6,   1,   1 },
    {  1,   0,  -8,  -3,  26,  55,  51,  17,  -7,  -6,   1,   1 },
    {  1,   0,  -7,  -4,  24,  54,  52,  19,  -6,  -7,   1,   1 },
    {  1,   0,  -7,  -5,  22,  53,  53,  22,  -5,  -7,   0,   1 },
    {  1,   1,  -7,  -6,  19,  52,  54,  24,  -4,  -7,   0,   1 },
    {  1,   1,  -6,  -7,  17,  51,  55,  26,  -3,  -8,   0,   1 },
    {  1,   1,  -6,  -7,  15,  49,  56,  29,  -2,  -8,  -1,   1 },
    {  0,   1,  -5,  -7,  13,  47,  57,  31,  -1,  -8,  -1,   1 },
    {  0,   1,  -5,  -8,  11,  46,  58,  34,   1,  -9,  -2,   1 },
    {  0,   1,  -4,  -8,   9,  44,  58,  35,   2,  -9,  -2,   2 },
    {  0,   1,  -4,  -9,   7,  43,  58,  38,   4,  -9,  -3,   2 },
  },
  {
    { -2,  -7,   0,  17,  35,  43,  35,  17,   0,  -7,  -5,   2 },
    { -2,  -7,  -1,  16,  34,  43,  36,  18,   1,  -7,  -5,   2 },
    { -1,  -7,  -1,  14,  33,  43,  36,  19,   1,  -6,  -5,   2 },
    { -1,  -7,  -2,  13,  32,  42,  37,  20,   3,  -6,  -5,   2 },
    {  0,  -7,  -3,  12,  31,  42,  38,  21,   3,  -6,  -5,   2 },
    {  0,  -7,  -3,  11,  30,  42,  39,  23,   4,  -6,  -6,   1 },
    {  0,  -7,  -4,  10,  29,  42,  40,  24,   5,  -6,  -6,   1 },
    {  1,  -7,  -4,   9,  27,  41,  40,  25,   6,  -5,  -6,   1 },
    {  1,  -6,  -5,   7,  26,  41,  41,  26,   7,  -5,  -6,   1 },
    {  1,  -6,  -5,   6,  25,  40,  41,  27,   9,  -4,  -7,   1 },
    {  1,  -6,  -6,   5,  24,  40,  42,  29,  10,  -4,  -7,   0 },
    {  1,  -6,  -6,   4,  23,  39,  42,  30,  11,  -3,  -7,   0 },
    {  2,  -5,  -6,   3,  21,  38,  42,  31,  12,  -3,  -7,   0 },
    {  2,  -5,  -6,   3,  20,  37,  42,  32,  13,  -2,  -7,  -1 },
    {  2,  -5,  -6,   1,  19,  36,  43,  33,  14,  -1,  -7,  -1 },
    {  2,  -5,  -7,   1,  18,  36,  43,  34,  16,  -1,  -7,  -2 }
  },
  {
    { -6,  -3,   5,  19,  31,  36,  31,  19,   5,  -3,  -6,   0 },
    { -6,  -4,   4,  18,  31,  37,  32,  20,   6,  -3,  -6,  -1 },
    { -6,  -4,   4,  17,  30,  36,  33,  21,   7,  -3,  -6,  -1 },
    { -5,  -5,   3,  16,  30,  36,  33,  22,   8,  -2,  -6,  -2 },
    { -5,  -5,   2,  15,  29,  36,  34,  23,   9,  -2,  -6,  -2 },
    { -5,  -5,   2,  15,  28,  36,  34,  24,  10,  -2,  -6,  -3 },
    { -4,  -5,   1,  14,  27,  36,  35,  24,  10,  -1,  -6,  -3 },
    { -4,  -5,   0,  13,  26,  35,  35,  25,  11,   0,  -5,  -3 },
    { -4,  -6,   0,  12,  26,  36,  36,  26,  12,   0,  -6,  -4 },
    { -3,  -5,   0,  11,  25,  35,  35,  26,  13,   0,  -5,  -4 },
    { -3,  -6,  -1,  10,  24,  35,  36,  27,  14,   1,  -5,  -4 },
    { -3,  -6,  -2,  10,  24,  34,  36,  28,  15,   2,  -5,  -5 },
    { -2,  -6,  -2,   9,  23,  34,  36,  29,  15,   2,  -5,  -5 },
    { -2,  -6,  -2,   8,  22,  33,  36,  30,  16,   3,  -5,  -5 },
    { -1,  -6,  -3,   7,  21,  33,  36,  30,  17,   4,  -4,  -6 },
    { -1,  -6,  -3,   6,  20,  32,  37,  31,  18,   4,  -4,  -6 }
  },
  {
    { -9,   0,   9,  20,  28,  32,  28,  20,   9,   0,  -9,   0 },
    { -9,   0,   8,  19,  28,  32,  29,  20,  10,   0,  -4,  -5 },
    { -9,  -1,   8,  18,  28,  32,  29,  21,  10,   1,  -4,  -5 },
    { -9,  -1,   7,  18,  27,  32,  30,  22,  11,   1,  -4,  -6 },
    { -8,  -2,   6,  17,  27,  32,  30,  22,  12,   2,  -4,  -6 },
    { -8,  -2,   6,  16,  26,  32,  31,  23,  12,   2,  -4,  -6 },
    { -8,  -2,   5,  16,  26,  31,  31,  23,  13,   3,  -3,  -7 },
    { -8,  -3,   5,  15,  25,  31,  31,  24,  14,   4,  -3,  -7 },
    { -7,  -3,   4,  14,  25,  31,  31,  25,  14,   4,  -3,  -7 },
    { -7,  -3,   4,  14,  24,  31,  31,  25,  15,   5,  -3,  -8 },
    { -7,  -3,   3,  13,  23,  31,  31,  26,  16,   5,  -2,  -8 },
    { -6,  -4,   2,  12,  23,  31,  32,  26,  16,   6,  -2,  -8 },
    { -6,  -4,   2,  12,  22,  30,  32,  27,  17,   6,  -2,  -8 },
    { -6,  -4,   1,  11,  22,  30,  32,  27,  18,   7,  -1,  -9 },
    { -5,  -4,   1,  10,  21,  29,  32,  28,  18,   8,  -1,  -9 },
    { -5,  -4,   0,  10,  20,  29,  32,  28,  19,   8,   0,  -9 }
  },
  {
    { -8,   7,  13,  18,  22,  24,  22,  18,  13,   7,   2, -10 },
    { -8,   7,  13,  18,  22,  23,  22,  19,  13,   7,   2, -10 },
    { -8,   6,  12,  18,  22,  23,  22,  19,  14,   8,   2, -10 },
    { -9,   6,  12,  17,  22,  23,  23,  19,  14,   8,   3, -10 },
    { -9,   6,  12,  17,  21,  23,  23,  19,  14,   9,   3, -10 },
    { -9,   5,  11,  17,  21,  23,  23,  20,  15,   9,   3, -10 },
    { -9,   5,  11,  16,  21,  23,  23,  20,  15,   9,   4, -10 },
    { -9,   5,  10,  16,  21,  23,  23,  20,  15,  10,   4, -10 },
    {-10,   5,  10,  16,  20,  23,  23,  20,  16,  10,   5, -10 },
    {-10,   4,  10,  15,  20,  23,  23,  21,  16,  10,   5,  -9 },
    {-10,   4,   9,  15,  20,  23,  23,  21,  16,  11,   5,  -9 },
    {-10,   3,   9,  15,  20,  23,  23,  21,  17,  11,   5,  -9 },
    {-10,   3,   9,  14,  19,  23,  23,  21,  17,  12,   6,  -9 },
    {-10,   3,   8,  14,  19,  23,  23,  22,  17,  12,   6,  -9 },
    {-10,   2,   8,  14,  19,  22,  23,  22,  18,  12,   6,  -8 },
    {-10,   2,   7,  13,  19,  22,  23,  22,  18,  13,   7,  -8 }
  }
};

static int GetFilterFromScale(int scale) {
  int filter = 0;
  if (scale > 245760) {
    filter = 7;
  } else if (scale > 187245) {
    filter = 6;
  } else if (scale > 163840) {
    filter = 5;
  } else if (scale > 131072) {
    filter = 4;
  } else if (scale > 109226) {
    filter = 3;
  } else if (scale > 81920) {
    filter = 2;
  } else if (scale > 68985) {
    filter = 1;
  }
  return filter;
}

template <typename T>
uint16_t FilterHor(const T* src, int sub_pel, int shift, int scale_factor) {
  int sum = 0;
  if (scale_factor < 65536) {
    // Upsampling.
    for (int i = 0; i < 8; i++) {
      sum += src[i - 3] * kUpsampleFilter[sub_pel][i];
    }
  } else if (scale_factor == 65536) {
    // No resampling.
    sum += src[0] << 6;
  } else {
    // Downsampling.
    int filter = GetFilterFromScale(scale_factor);
    for (int i = 0; i < 12; i++) {
      sum += src[i - 5] * kDownsampleFilters[filter][sub_pel][i];
    }
    sum >>= 1;
  }
  uint16_t max_value = std::numeric_limits<uint16_t>::max();
  return util::Clip3<uint16_t>(sum >> shift, 0, max_value);
}

template <typename T>
T FilterVer(const uint16_t* src, int sub_pel, int shift, int stride, T max,
            int scale_factor) {
  int sum = 0;
  if (scale_factor < 65536) {
    // Upsampling.
    for (int i = 0; i < 8; i++) {
      sum += src[(i - 3) * stride] * kUpsampleFilter[sub_pel][i];
    }
  } else if (scale_factor == 65536) {
    // No resampling.
    sum += src[0] << 6;
  } else {
    // Downsampling.
    int filter = GetFilterFromScale(scale_factor);
    for (int i = 0; i < 12; i++) {
      sum += src[(i - 5) * stride] * kDownsampleFilters[filter][sub_pel][i];
    }
    sum >>= 1;
  }
  return util::Clip3<T>(sum >> shift, 0, max);
}

template <typename T, typename U>
void Resample(uint8_t *dst_start, int dst_width, int dst_height,
              ptrdiff_t dst_stride, int dst_bitdepth,
              const uint8_t *src_start, int src_width, int src_height,
              ptrdiff_t src_stride, int src_bitdepth) {
  const T* src = reinterpret_cast<const T*>(src_start);
  U* dst = reinterpret_cast<U *>(dst_start);

  int tmp_pad = 8;
  int tmp_width = dst_width;
  int tmp_height = src_height;
  std::vector<uint16_t> tmp_bytes;
  tmp_bytes.resize((tmp_height + 2 * tmp_pad) * tmp_width);
  uint16_t* tmp = &tmp_bytes[0];

  int scale_x =
    ((src_width << kPositionPrecision) + (dst_width >> 1)) / dst_width;
  int shift_hor =
    std::max(src_bitdepth - (kInternalPrecision - kFilterPrecision), 0);

  // Horizontal filtering from src to tmp.
  for (int i = -tmp_pad; i < tmp_height + tmp_pad; i++) {
    for (int j = 0; j < tmp_width; j++) {
      int pos_x = (j * scale_x) >> (kPositionPrecision - 4);
      int sub_pel = pos_x & 15;
      int full_pel = pos_x >> 4;
      *tmp++ = FilterHor<T>(&src[i * src_stride + full_pel], sub_pel, shift_hor,
                            scale_x);
    }
  }

  int scale_y =
    ((src_height << kPositionPrecision) + (dst_height >> 1)) / dst_height;
  int shift_ver =
    2 * kFilterPrecision - shift_hor + src_bitdepth - dst_bitdepth;
  U max = (1 << dst_bitdepth) - 1;

  // Vertical filtering from tmp to dst.
  for (int i = 0; i < dst_height; i++) {
    int pos_y = (i * scale_y) >> (kPositionPrecision - 4);
    int sub_pel = pos_y & 15;
    int full_pel = pos_y >> 4;
    tmp = &tmp_bytes[(tmp_pad + full_pel) * tmp_width];
    for (int j = 0; j < dst_width; j++) {
      dst[j] = FilterVer<U>(tmp++, sub_pel, shift_ver, tmp_width, max, scale_y);
    }
    dst += dst_stride;
  }
}

template void Resample<uint8_t, uint8_t>(uint8_t *dst_start, int dst_width,
                                         int dst_height, ptrdiff_t dst_stride,
                                         int dst_bitdepth,
                                         const uint8_t *src_start,
                                         int src_width, int src_height,
                                         ptrdiff_t src_stride,
                                         int src_bitdepth);

template void Resample<uint16_t, uint8_t>(uint8_t *dst_start, int dst_width,
                                          int dst_height, ptrdiff_t dst_stride,
                                          int dst_bitdepth,
                                          const uint8_t *src_start,
                                          int src_width, int src_height,
                                          ptrdiff_t src_stride,
                                          int src_bitdepth);

template void Resample<uint8_t, uint16_t>(uint8_t *dst_start, int dst_width,
                                          int dst_height, ptrdiff_t dst_stride,
                                          int dst_bitdepth,
                                          const uint8_t *src_start,
                                          int src_width, int src_height,
                                          ptrdiff_t src_stride,
                                          int src_bitdepth);

template void Resample<uint16_t, uint16_t>(uint8_t *dst_start, int dst_width,
                                           int dst_height, ptrdiff_t dst_stride,
                                           int dst_bitdepth,
                                           const uint8_t *src_start,
                                           int src_width, int src_height,
                                           ptrdiff_t src_stride,
                                           int src_bitdepth);

template <typename T, typename U>
void BilinearResample(uint8_t *dst_start, int dst_width, int dst_height,
                      ptrdiff_t dst_stride, int dst_bitdepth,
                      const uint8_t *src_start, int src_width, int src_height,
                      ptrdiff_t src_stride, int src_bitdepth) {
  const T* src = reinterpret_cast<const T*>(src_start);
  U* dst = reinterpret_cast<U *>(dst_start);

  int shift = dst_bitdepth - src_bitdepth;
  if (shift > 0) {
    for (int i = 0; i < src_height; i++) {
      for (int j = 0; j < src_width; j++) {
        dst[2 * j] = static_cast<U>(src[j] << shift);
        dst[2 * j + 1] = static_cast<U>((src[j] + src[j + 1]) << (shift - 1));
        dst[2 * j + dst_stride] =
          static_cast<U>((src[j] + src[j + src_stride]) << (shift - 1));
        dst[2 * j + dst_stride + 1] =
          static_cast<U>((src[j] + src[j + 1] + src[j + src_stride] + src[j +
                          src_stride + 1] + 2) << (shift - 2));
      }
      dst += 2 * dst_stride;
      src += src_stride;
    }
  } else {
    shift = -shift;
    for (int i = 0; i < src_height; i++) {
      for (int j = 0; j < src_width; j++) {
        dst[2 * j] = static_cast<U>(src[j] >> shift);
        dst[2 * j + 1] = static_cast<U>((src[j] + src[j + 1]) >> (shift + 1));
        dst[2 * j + dst_stride] =
          static_cast<U>((src[j] + src[j + src_stride]) >> (shift + 1));
        dst[2 * j + dst_stride + 1] =
          static_cast<U>((src[j] + src[j + 1] + src[j + src_stride] + src[j +
                          src_stride + 1] + 2) >> (shift + 2));
      }
      dst += 2 * dst_stride;
      src += src_stride;
    }
  }
}

template void BilinearResample<Sample, uint8_t>(uint8_t *dst_start,
                                                int dst_width,
                                                int dst_height,
                                                ptrdiff_t dst_stride,
                                                int dst_bitdepth,
                                                const uint8_t *src_start,
                                                int src_width, int src_height,
                                                ptrdiff_t src_stride,
                                                int src_bitdepth);

template void BilinearResample<Sample, uint16_t>(uint8_t *dst_start,
                                                 int dst_width,
                                                 int dst_height,
                                                 ptrdiff_t dst_stride,
                                                 int dst_bitdepth,
                                                 const uint8_t *src_start,
                                                 int src_width, int src_height,
                                                 ptrdiff_t src_stride,
                                                 int src_bitdepth);

}   // namespace resample

}   // namespace xvc
