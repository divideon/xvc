/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/sample_metric.h"

#include <cassert>
#include <cmath>
#include <cstdlib>
#include <limits>

#if _MSC_VER
// Disable warning C6201: buffer overrun detection, which causes a lot of false
// positives due to bad static bounds checking
#pragma warning(disable:6201)
#endif

namespace xvc {

Distortion
SampleMetric::CompareSample(const CodingUnit &cu, YuvComponent comp,
                            const YuvPicture &src1, const YuvPicture &src2) {
  const Sample *src2_ptr =
    src2.GetSamplePtr(comp, cu.GetPosX(comp), cu.GetPosY(comp));
  ptrdiff_t stride2 = src2.GetStride(comp);
  return CompareSample(cu, comp, src1, src2_ptr, stride2);
}

Distortion
SampleMetric::CompareSample(const CodingUnit &cu, YuvComponent comp,
                            const YuvPicture &src1, const SampleBuffer &src2) {
  return CompareSample(cu, comp, src1, src2.GetDataPtr(), src2.GetStride());
}

Distortion
SampleMetric::CompareSample(const CodingUnit &cu, YuvComponent comp,
                            const SampleBuffer &src1,
                            const SampleBuffer &src2) {
  return CompareSample(comp, cu.GetWidth(comp), cu.GetHeight(comp),
                       src1.GetDataPtr(), src1.GetStride(),
                       src2.GetDataPtr(), src2.GetStride());
}

Distortion
SampleMetric::CompareSample(YuvComponent comp, int width, int height,
                            const Sample *src1, ptrdiff_t stride1,
                            const Sample *src2, ptrdiff_t stride2) {
  return Compare(comp, width, height, src1, stride1, src2, stride2);
}

Distortion
SampleMetric::CompareSample(const CodingUnit &cu, YuvComponent comp,
                            const YuvPicture &src1,
                            const Sample *src2, ptrdiff_t stride2) {
  int posx = cu.GetPosX(comp);
  int posy = cu.GetPosY(comp);
  int width = cu.GetWidth(comp);
  int height = cu.GetHeight(comp);
  const Sample *src1_ptr = src1.GetSamplePtr(comp, posx, posy);
  ptrdiff_t stride1 = src1.GetStride(comp);
  return Compare(comp, width, height, src1_ptr, stride1, src2, stride2);
}

Distortion
SampleMetric::CompareSample(YuvComponent comp, int width, int height,
                            const Residual *src1, ptrdiff_t stride1,
                            const Sample *src2, ptrdiff_t stride2) {
  return Compare(comp, width, height, src1, stride1, src2, stride2);
}

Distortion SampleMetric::CompareShort(YuvComponent comp, int width, int height,
                                      const DataBuffer<Residual> &src1,
                                      const DataBuffer<Residual> &src2) {
  return Compare(comp, width, height, src1.GetDataPtr(), src1.GetStride(),
                 src2.GetDataPtr(), src2.GetStride());
}

Distortion SampleMetric::CompareShort(YuvComponent comp, int width, int height,
                                      const Residual *src1, ptrdiff_t stride1,
                                      const Residual *src2, ptrdiff_t stride2) {
  return Compare(comp, width, height, src1, stride1, src2, stride2);
}

template<typename SampleT1, typename SampleT2>
Distortion
SampleMetric::Compare(YuvComponent comp, int width, int height,
                      const SampleT1 *src1, ptrdiff_t stride1,
                      const SampleT2 *src2, ptrdiff_t stride2) {
  double weight = qp_.GetDistortionWeight(comp);
  uint64_t dist;
  switch (type_) {
    case MetricType::kSsd:
      dist = ComputeSsd(width, height, src1, stride1, src2, stride2);
      break;
    case MetricType::kSatd:
      dist = ComputeSatd(width, height, src1, stride1, src2, stride2);
      break;
    case MetricType::kSad:
      dist = ComputeSad(width, height, src1, stride1, src2, stride2);
      break;
    case MetricType::kSadFast:
      dist = ComputeSadFast(width, height, src1, stride1, src2, stride2);
      break;
    case MetricType::kStructuralSsd:
      dist = ComputeStructuralSsd(width, height, src1, stride1, src2, stride2);
      break;
    default:
      assert(0);
      return std::numeric_limits<Distortion>::max();
      break;
  }
  return static_cast<Distortion>(dist * weight);
}

template<typename SampleT1, typename SampleT2>
uint64_t SampleMetric::ComputeSsd(int width, int height,
                                  const SampleT1 *sample1, ptrdiff_t stride1,
                                  const SampleT2 *sample2, ptrdiff_t stride2) {
  int shift = (2 * (bitdepth_ - 8));
  uint64_t ssd = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int diff = sample1[x] - sample2[x];
      ssd += diff * diff;
    }
    sample1 += stride1;
    sample2 += stride2;
  }
  ssd >>= shift;
  return ssd;
}

template<typename SampleT1, typename SampleT2>
uint64_t SampleMetric::ComputeSatd(int width, int height,
                                   const SampleT1 *sample1, ptrdiff_t stride1,
                                   const SampleT2 *sample2, ptrdiff_t stride2) {
  static_assert(constants::kMinBlockSize >= 4, "SATD only implmented for 4x4");
  uint64_t sad = 0;
  if (width == 4 && height == 4) {
    for (int y = 0; y < height; y += 4) {
      for (int x = 0; x < width; x += 4) {
        sad += ComputeSatdNxM<4, 4>(sample1 + x, stride1, sample2 + x, stride2);
      }
      sample1 += stride1 * 4;
      sample2 += stride2 * 4;
    }
  } else if (height == 4 && width > height) {
    for (int y = 0; y < height; y += 4) {
      for (int x = 0; x < width; x += 8) {
        sad += ComputeSatdNxM<8, 4>(sample1 + x, stride1, sample2 + x, stride2);
      }
      sample1 += stride1 * 4;
      sample2 += stride2 * 4;
    }
  } else if (width == 4 && height > width) {
    for (int y = 0; y < height; y += 8) {
      for (int x = 0; x < width; x += 4) {
        sad += ComputeSatdNxM<4, 8>(sample1 + x, stride1, sample2 + x, stride2);
      }
      sample1 += stride1 * 8;
      sample2 += stride2 * 8;
    }
  } else if (width > height) {
    for (int y = 0; y < height; y += 8) {
      for (int x = 0; x < width; x += 16) {
        sad +=
          ComputeSatdNxM<16, 8>(sample1 + x, stride1, sample2 + x, stride2);
      }
      sample1 += stride1 * 8;
      sample2 += stride2 * 8;
    }
  } else if (width < height) {
    for (int y = 0; y < height; y += 16) {
      for (int x = 0; x < width; x += 8) {
        sad +=
          ComputeSatdNxM<8, 16>(sample1 + x, stride1, sample2 + x, stride2);
      }
      sample1 += stride1 * 16;
      sample2 += stride2 * 16;
    }
  } else {
    for (int y = 0; y < height; y += 8) {
      for (int x = 0; x < width; x += 8) {
        sad += ComputeSatdNxM<8, 8>(sample1 + x, stride1, sample2 + x, stride2);
      }
      sample1 += stride1 * 8;
      sample2 += stride2 * 8;
    }
  }
  return sad >> (bitdepth_ - 8);
}

template<int W, int H, typename SampleT1, typename SampleT2>
int SampleMetric::ComputeSatdNxM(const SampleT1 *sample1, ptrdiff_t stride1,
                                 const SampleT2 *sample2, ptrdiff_t stride2) {
  int diff[W*H], m1[H][W], m2[H][W];
  static_assert(W == 4 || W == 8 || W == 16, "Only W = 8 or 16 supported");
  static_assert(H == 4 || H == 8 || H == 16, "Only H = 8 or 16 supported");

  for (int k = 0; k < W*H; k += W) {
    diff[k + 0] = sample1[0] - sample2[0];
    diff[k + 1] = sample1[1] - sample2[1];
    diff[k + 2] = sample1[2] - sample2[2];
    diff[k + 3] = sample1[3] - sample2[3];
    if (W > 4) {
      diff[k + 4] = sample1[4] - sample2[4];
      diff[k + 5] = sample1[5] - sample2[5];
      diff[k + 6] = sample1[6] - sample2[6];
      diff[k + 7] = sample1[7] - sample2[7];
    }
    if (W > 8) {
      diff[k + 8] = sample1[8] - sample2[8];
      diff[k + 9] = sample1[9] - sample2[9];
      diff[k + 10] = sample1[10] - sample2[10];
      diff[k + 11] = sample1[11] - sample2[11];
      diff[k + 12] = sample1[12] - sample2[12];
      diff[k + 13] = sample1[13] - sample2[13];
      diff[k + 14] = sample1[14] - sample2[14];
      diff[k + 15] = sample1[15] - sample2[15];
    }
    sample1 += stride1;
    sample2 += stride2;
  }

  // Horizontal
  for (int j = 0; j < H; j++) {
    int jj = j * W;

    if (W > 8) {
      m1[j][0] = diff[jj] + diff[jj + 8];
      m1[j][1] = diff[jj + 1] + diff[jj + 9];
      m1[j][2] = diff[jj + 2] + diff[jj + 10];
      m1[j][3] = diff[jj + 3] + diff[jj + 11];
      m1[j][4] = diff[jj + 4] + diff[jj + 12];
      m1[j][5] = diff[jj + 5] + diff[jj + 13];
      m1[j][6] = diff[jj + 6] + diff[jj + 14];
      m1[j][7] = diff[jj + 7] + diff[jj + 15];
      m1[j][8] = diff[jj] - diff[jj + 8];
      m1[j][9] = diff[jj + 1] - diff[jj + 9];
      m1[j][10] = diff[jj + 2] - diff[jj + 10];
      m1[j][11] = diff[jj + 3] - diff[jj + 11];
      m1[j][12] = diff[jj + 4] - diff[jj + 12];
      m1[j][13] = diff[jj + 5] - diff[jj + 13];
      m1[j][14] = diff[jj + 6] - diff[jj + 14];
      m1[j][15] = diff[jj + 7] - diff[jj + 15];

      m2[j][0] = m1[j][0] + m1[j][4];
      m2[j][1] = m1[j][1] + m1[j][5];
      m2[j][2] = m1[j][2] + m1[j][6];
      m2[j][3] = m1[j][3] + m1[j][7];
      m2[j][4] = m1[j][0] - m1[j][4];
      m2[j][5] = m1[j][1] - m1[j][5];
      m2[j][6] = m1[j][2] - m1[j][6];
      m2[j][7] = m1[j][3] - m1[j][7];
      m2[j][8] = m1[j][8] + m1[j][12];
      m2[j][9] = m1[j][9] + m1[j][13];
      m2[j][10] = m1[j][10] + m1[j][14];
      m2[j][11] = m1[j][11] + m1[j][15];
      m2[j][12] = m1[j][8] - m1[j][12];
      m2[j][13] = m1[j][9] - m1[j][13];
      m2[j][14] = m1[j][10] - m1[j][14];
      m2[j][15] = m1[j][11] - m1[j][15];
    } else if (W > 4) {
      m2[j][0] = diff[jj + 0] + diff[jj + 4];
      m2[j][1] = diff[jj + 1] + diff[jj + 5];
      m2[j][2] = diff[jj + 2] + diff[jj + 6];
      m2[j][3] = diff[jj + 3] + diff[jj + 7];
      m2[j][4] = diff[jj + 0] - diff[jj + 4];
      m2[j][5] = diff[jj + 1] - diff[jj + 5];
      m2[j][6] = diff[jj + 2] - diff[jj + 6];
      m2[j][7] = diff[jj + 3] - diff[jj + 7];
    } else {
      m1[j][0] = diff[jj + 0] + diff[jj + 2];
      m1[j][1] = diff[jj + 1] + diff[jj + 3];
      m1[j][2] = diff[jj + 0] - diff[jj + 2];
      m1[j][3] = diff[jj + 1] - diff[jj + 3];
    }

    if (W > 4) {
      m1[j][0] = m2[j][0] + m2[j][2];
      m1[j][1] = m2[j][1] + m2[j][3];
      m1[j][2] = m2[j][0] - m2[j][2];
      m1[j][3] = m2[j][1] - m2[j][3];
      m1[j][4] = m2[j][4] + m2[j][6];
      m1[j][5] = m2[j][5] + m2[j][7];
      m1[j][6] = m2[j][4] - m2[j][6];
      m1[j][7] = m2[j][5] - m2[j][7];
    }
    if (W > 8) {
      m1[j][8] = m2[j][8] + m2[j][10];
      m1[j][9] = m2[j][9] + m2[j][11];
      m1[j][10] = m2[j][8] - m2[j][10];
      m1[j][11] = m2[j][9] - m2[j][11];
      m1[j][12] = m2[j][12] + m2[j][14];
      m1[j][13] = m2[j][13] + m2[j][15];
      m1[j][14] = m2[j][12] - m2[j][14];
      m1[j][15] = m2[j][13] - m2[j][15];
    }

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    if (W > 4) {
      m2[j][4] = m1[j][4] + m1[j][5];
      m2[j][5] = m1[j][4] - m1[j][5];
      m2[j][6] = m1[j][6] + m1[j][7];
      m2[j][7] = m1[j][6] - m1[j][7];
    }
    if (W > 8) {
      m2[j][8] = m1[j][8] + m1[j][9];
      m2[j][9] = m1[j][8] - m1[j][9];
      m2[j][10] = m1[j][10] + m1[j][11];
      m2[j][11] = m1[j][10] - m1[j][11];
      m2[j][12] = m1[j][12] + m1[j][13];
      m2[j][13] = m1[j][12] - m1[j][13];
      m2[j][14] = m1[j][14] + m1[j][15];
      m2[j][15] = m1[j][14] - m1[j][15];
    }
  }

  // Vertical
  for (int i = 0; i < W; i++) {
    if (H > 8) {
      m1[0][i] = m2[0][i] + m2[8][i];
      m1[1][i] = m2[1][i] + m2[9][i];
      m1[2][i] = m2[2][i] + m2[10][i];
      m1[3][i] = m2[3][i] + m2[11][i];
      m1[4][i] = m2[4][i] + m2[12][i];
      m1[5][i] = m2[5][i] + m2[13][i];
      m1[6][i] = m2[6][i] + m2[14][i];
      m1[7][i] = m2[7][i] + m2[15][i];
      m1[8][i] = m2[0][i] - m2[8][i];
      m1[9][i] = m2[1][i] - m2[9][i];
      m1[10][i] = m2[2][i] - m2[10][i];
      m1[11][i] = m2[3][i] - m2[11][i];
      m1[12][i] = m2[4][i] - m2[12][i];
      m1[13][i] = m2[5][i] - m2[13][i];
      m1[14][i] = m2[6][i] - m2[14][i];
      m1[15][i] = m2[7][i] - m2[15][i];
    } else if (H > 4) {
      m1[0][i] = m2[0][i];
      m1[1][i] = m2[1][i];
      m1[2][i] = m2[2][i];
      m1[3][i] = m2[3][i];
      m1[4][i] = m2[4][i];
      m1[5][i] = m2[5][i];
      m1[6][i] = m2[6][i];
      m1[7][i] = m2[7][i];
    }

    if (H > 4) {
      m2[0][i] = m1[0][i] + m1[4][i];
      m2[1][i] = m1[1][i] + m1[5][i];
      m2[2][i] = m1[2][i] + m1[6][i];
      m2[3][i] = m1[3][i] + m1[7][i];
      m2[4][i] = m1[0][i] - m1[4][i];
      m2[5][i] = m1[1][i] - m1[5][i];
      m2[6][i] = m1[2][i] - m1[6][i];
      m2[7][i] = m1[3][i] - m1[7][i];
    }
    if (H > 8) {
      m2[8][i] = m1[8][i] + m1[12][i];
      m2[9][i] = m1[9][i] + m1[13][i];
      m2[10][i] = m1[10][i] + m1[14][i];
      m2[11][i] = m1[11][i] + m1[15][i];
      m2[12][i] = m1[8][i] - m1[12][i];
      m2[13][i] = m1[9][i] - m1[13][i];
      m2[14][i] = m1[10][i] - m1[14][i];
      m2[15][i] = m1[11][i] - m1[15][i];
    }

    m1[0][i] = m2[0][i] + m2[2][i];
    m1[1][i] = m2[1][i] + m2[3][i];
    m1[2][i] = m2[0][i] - m2[2][i];
    m1[3][i] = m2[1][i] - m2[3][i];
    if (H > 4) {
      m1[4][i] = m2[4][i] + m2[6][i];
      m1[5][i] = m2[5][i] + m2[7][i];
      m1[6][i] = m2[4][i] - m2[6][i];
      m1[7][i] = m2[5][i] - m2[7][i];
    }
    if (H > 8) {
      m1[8][i] = m2[8][i] + m2[10][i];
      m1[9][i] = m2[9][i] + m2[11][i];
      m1[10][i] = m2[8][i] - m2[10][i];
      m1[11][i] = m2[9][i] - m2[11][i];
      m1[12][i] = m2[12][i] + m2[14][i];
      m1[13][i] = m2[13][i] + m2[15][i];
      m1[14][i] = m2[12][i] - m2[14][i];
      m1[15][i] = m2[13][i] - m2[15][i];
    }

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    if (H > 4) {
      m2[4][i] = m1[4][i] + m1[5][i];
      m2[5][i] = m1[4][i] - m1[5][i];
      m2[6][i] = m1[6][i] + m1[7][i];
      m2[7][i] = m1[6][i] - m1[7][i];
    }
    if (H > 8) {
      m2[8][i] = m1[8][i] + m1[9][i];
      m2[9][i] = m1[8][i] - m1[9][i];
      m2[10][i] = m1[10][i] + m1[11][i];
      m2[11][i] = m1[10][i] - m1[11][i];
      m2[12][i] = m1[12][i] + m1[13][i];
      m2[13][i] = m1[12][i] - m1[13][i];
      m2[14][i] = m1[14][i] + m1[15][i];
      m2[15][i] = m1[14][i] - m1[15][i];
    }
  }

  int sum = 0;
  for (int i = 0; i < H; i++) {
    for (int j = 0; j < W; j++) {
      sum += std::abs(m2[i][j]);
    }
  }
  if (W == 4 && H == 4) {
    sum = (sum + 1) >> 1;
  } else if (W == H) {
    sum = (sum + 2) >> 2;
  } else {
    sum = static_cast<int>(2.0 * sum / std::sqrt(W*H));
  }
  return sum;
}

template<typename SampleT1, typename SampleT2>
uint64_t SampleMetric::ComputeSad(int width, int height,
                                  const SampleT1 *sample1, ptrdiff_t stride1,
                                  const SampleT2 *sample2, ptrdiff_t stride2) {
  uint64_t sum = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int diff = sample1[x] - sample2[x];
      sum += std::abs(diff);
    }
    sample1 += stride1;
    sample2 += stride2;
  }
  return sum >> (bitdepth_ - 8);
}

template<typename SampleT1, typename SampleT2>
uint64_t
SampleMetric::ComputeSadFast(int width, int height,
                             const SampleT1 *sample1, ptrdiff_t stride1,
                             const SampleT2 *sample2, ptrdiff_t stride2) {
  stride1 <<= 1;
  stride2 <<= 1;
  uint64_t sum = 0;
  for (int y = 0; y < height; y += 2) {
    for (int x = 0; x < width; x++) {
      int diff = sample1[x] - sample2[x];
      sum += std::abs(diff);
    }
    sample1 += stride1;
    sample2 += stride2;
  }
  sum <<= 1;
  return sum >> (bitdepth_ - 8);
}

template<typename SampleT1, typename SampleT2>
uint64_t SampleMetric::ComputeStructuralSsd8(const SampleT1 *sample1,
                                             ptrdiff_t stride1,
                                             const SampleT2 *sample2,
                                             ptrdiff_t stride2) {
  int64_t orig_sum = 0;
  int64_t reco_sum = 0;
  int64_t orig_orig_sum = 0;
  int64_t reco_reco_sum = 0;
  int64_t orig_reco_sum = 0;
  const int n = 64;
  const int shift = (2 * (bitdepth_ - 8));
  const int64_t c1 = (n * n * 26634ull >> 12) << shift;
  const int64_t c2 = (n * n * 239708ull >> 12) << shift;
  const int64_t c4 = ((1ull << bitdepth_) - 1) * ((1 << bitdepth_) - 1);
  int64_t ssd = 0;
  for (int y = 0; y < 8; y++) {
    for (int x = 0; x < 8; x++) {
      orig_sum += sample1[x];
      reco_sum += sample2[x];
      orig_orig_sum += sample1[x] * sample1[x];
      reco_reco_sum += sample2[x] * sample2[x];
      orig_reco_sum += sample1[x] * sample2[x];
      int diff = sample1[x] - sample2[x];
      ssd += diff * diff;
    }
    sample1 += stride1;
    sample2 += stride2;
  }
  double m = (1.0 * orig_sum - reco_sum) / n;
  double a = (c4 - m * m + c1) / (c4 + c1);
  double b = (2.0 * n * orig_reco_sum - 2 * orig_sum * reco_sum + c2) /
    (n * orig_orig_sum - orig_sum * orig_sum +
     n * reco_reco_sum - reco_sum * reco_sum + c2);

  ssd >>= shift;
  return static_cast<uint64_t>((ssd + c4 * (1 - a * b))) >> 1;
}

template<typename SampleT1, typename SampleT2>
uint64_t SampleMetric::ComputeStructuralSsd4(const SampleT1 *sample1,
                                             ptrdiff_t stride1,
                                             const SampleT2 *sample2,
                                             ptrdiff_t stride2) {
  int64_t orig_sum = 0;
  int64_t reco_sum = 0;
  int64_t orig_orig_sum = 0;
  int64_t reco_reco_sum = 0;
  int64_t orig_reco_sum = 0;
  const int n = 16;
  const int shift = (2 * (bitdepth_ - 8));
  const int64_t c1 = (n * n * 26634ull >> 12) << shift;
  const int64_t c2 = (n * n * 239708ull >> 12) << shift;
  const int64_t c4 = ((1ull << bitdepth_) - 1) * ((1 << bitdepth_) - 1);
  int64_t ssd = 0;
  for (int y = 0; y < 4; y++) {
    for (int x = 0; x < 4; x++) {
      orig_sum += sample1[x];
      reco_sum += sample2[x];
      orig_orig_sum += sample1[x] * sample1[x];
      reco_reco_sum += sample2[x] * sample2[x];
      orig_reco_sum += sample1[x] * sample2[x];
      int diff = sample1[x] - sample2[x];
      ssd += diff * diff;
    }
    sample1 += stride1;
    sample2 += stride2;
  }
  double m = (1.0 * orig_sum - reco_sum) / n;
  double a = (c4 - m * m + c1) / (c4 + c1);
  double b = (2.0 * n * orig_reco_sum - 2 * orig_sum * reco_sum + c2) /
    (n * orig_orig_sum - orig_sum * orig_sum +
     n * reco_reco_sum - reco_sum * reco_sum + c2);

  ssd >>= shift;
  return static_cast<uint64_t>(ssd + (c4 >> 2) * (1 - a * b)) >> 1;
}

template<typename SampleT1, typename SampleT2>
uint64_t SampleMetric::ComputeStructuralSsd(int width, int height,
                                            const SampleT1 *sample1,
                                            ptrdiff_t stride1,
                                            const SampleT2 *sample2,
                                            ptrdiff_t stride2) {
  if (height < 8 || width < 8) {
    uint64_t ssim = 0;
    for (int i = 0; i < height / 4; i++) {
      for (int j = 0; j < width / 4; j++) {
        ssim += ComputeStructuralSsd4(sample1 + 4 * j, stride1,
                                      sample2 + 4 * j, stride2);
      }
      sample1 += 4 * stride1;
      sample2 += 4 * stride2;
    }
    return ssim;
  }
  uint64_t ssim = 0;
  for (int i = 0; i < height / 8; i++) {
    for (int j = 0; j < width / 8; j++) {
      ssim += ComputeStructuralSsd8(sample1 + 8 * j, stride1,
                                    sample2 + 8 * j, stride2);
    }
    sample1 += 8 * stride1;
    sample2 += 8 * stride2;
  }
  return ssim;
}

}   // namespace xvc
