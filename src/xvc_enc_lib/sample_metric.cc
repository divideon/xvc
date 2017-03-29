/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/sample_metric.h"

#include <cassert>
#include <cstdlib>
#include <limits>

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
    case MetricType::kSSE:
      dist = ComputeSSE(width, height, src1, stride1, src2, stride2);
      break;
    case MetricType::kSATD:
      dist = ComputeSATD(width, height, src1, stride1, src2, stride2);
      break;
    case MetricType::kSAD:
      dist = ComputeSAD(width, height, src1, stride1, src2, stride2);
      break;
    case MetricType::kSADFast:
      dist = ComputeSADFast(width, height, src1, stride1, src2, stride2);
      break;
    default:
      assert(0);
      return std::numeric_limits<Distortion>::max();
      break;
  }
  return static_cast<Distortion>(dist * weight);
}

template<typename SampleT1, typename SampleT2>
uint64_t SampleMetric::ComputeSSE(int width, int height,
                                  const SampleT1 *sample1, ptrdiff_t stride1,
                                  const SampleT2 *sample2, ptrdiff_t stride2) {
  int shift = (2 * (bitdepth_ - 8));
  uint64_t ssd = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int diff = sample1[x] - sample2[x];
#if !HM_STRICT
      ssd += (diff*diff) >> shift;
#else
      ssd += diff * diff;
#endif
    }
    sample1 += stride1;
    sample2 += stride2;
  }
#if HM_STRICT
  ssd >>= shift;
#endif
  return ssd;
}

template<typename SampleT1, typename SampleT2>
uint64_t SampleMetric::ComputeSATD(int width, int height,
                                   const SampleT1 *sample1, ptrdiff_t stride1,
                                   const SampleT2 *sample2, ptrdiff_t stride2) {
  uint64_t sad = 0;
  for (int y = 0; y < height; y += 8) {
    for (int x = 0; x < width; x += 8) {
      sad += ComputeSATD8x8(sample1 + x, stride1, sample2 + x, stride2);
    }
    sample1 += stride1 * 8;
    sample2 += stride2 * 8;
  }
  return sad >> (bitdepth_ - 8);
}

template<typename SampleT1, typename SampleT2>
int SampleMetric::ComputeSATD8x8(const SampleT1 *sample1, ptrdiff_t stride1,
                                 const SampleT2 *sample2, ptrdiff_t stride2) {
  int diff[64], m1[8][8], m2[8][8], m3[8][8];

  for (int k = 0; k < 64; k += 8) {
    diff[k + 0] = sample1[0] - sample2[0];
    diff[k + 1] = sample1[1] - sample2[1];
    diff[k + 2] = sample1[2] - sample2[2];
    diff[k + 3] = sample1[3] - sample2[3];
    diff[k + 4] = sample1[4] - sample2[4];
    diff[k + 5] = sample1[5] - sample2[5];
    diff[k + 6] = sample1[6] - sample2[6];
    diff[k + 7] = sample1[7] - sample2[7];
    sample1 += stride1;
    sample2 += stride2;
  }

  // Horizontal
  for (int j = 0; j < 8; j++) {
    int jj = j << 3;
    m2[j][0] = diff[jj + 0] + diff[jj + 4];
    m2[j][1] = diff[jj + 1] + diff[jj + 5];
    m2[j][2] = diff[jj + 2] + diff[jj + 6];
    m2[j][3] = diff[jj + 3] + diff[jj + 7];
    m2[j][4] = diff[jj + 0] - diff[jj + 4];
    m2[j][5] = diff[jj + 1] - diff[jj + 5];
    m2[j][6] = diff[jj + 2] - diff[jj + 6];
    m2[j][7] = diff[jj + 3] - diff[jj + 7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  // Vertical
  for (int i = 0; i < 8; i++) {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  int sum = 0;
  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++) {
      sum += std::abs(m2[i][j]);
    }
  }
  sum = ((sum + 2) >> 2);
  return sum;
}

template<typename SampleT1, typename SampleT2>
uint64_t SampleMetric::ComputeSAD(int width, int height,
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
SampleMetric::ComputeSADFast(int width, int height,
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

}   // namespace xvc
