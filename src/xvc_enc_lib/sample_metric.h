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

#ifndef XVC_ENC_LIB_SAMPLE_METRIC_H_
#define XVC_ENC_LIB_SAMPLE_METRIC_H_

#include <vector>

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/common.h"
#include "xvc_common_lib/sample_buffer.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/yuv_pic.h"

namespace xvc {

enum class MetricType {
  kSsd,
  kSatd,
  kSatdAcOnly,
  kSad,
  kSadFast,
  kSadAcOnly,
  kSadAcOnlyFast,
  kStructuralSsd,
};

class SampleMetric {
public:
  struct SimdFunc;
  SampleMetric(const SimdFunc &simd_func, int bitdepth, MetricType type,
               int structural_strength = 1)
    : simd_func_(simd_func), bitdepth_(bitdepth), type_(type),
    structural_strength_(structural_strength) {
  }
  SampleMetric(const SampleMetric&) = delete;
  // Sample vs Sample
  Distortion CompareSample(const CodingUnit &cu, YuvComponent comp,
                           const YuvPicture &src1,
                           const YuvPicture &src2) const;
  Distortion CompareSample(const CodingUnit &cu, YuvComponent comp,
                           const YuvPicture &src1,
                           const Sample *src2, ptrdiff_t stride2) const;
  Distortion CompareSample(const CodingUnit &cu, YuvComponent comp,
                           const YuvPicture &src1,
                           const SampleBufferConst &src2) const {
    return CompareSample(cu, comp, src1, src2.GetDataPtr(), src2.GetStride());
  }
  Distortion CompareSample(const Qp &qp, YuvComponent comp,
                           int width, int height,
                           const SampleBufferConst &src1,
                           const SampleBufferConst &src2) const {
    return CompareSample(qp, comp, width, height,
                         src1.GetDataPtr(), src1.GetStride(),
                         src2.GetDataPtr(), src2.GetStride());
  }
  Distortion CompareSample(const Qp &qp, YuvComponent comp,
                           int width, int height,
                           const Sample *src1, ptrdiff_t stride1,
                           const Sample *src2, ptrdiff_t stride2) const {
    return Compare(qp, comp, width, height, src1, stride1, src2, stride2);
  }
  // Sample vs Residual
  Distortion CompareSample(const Qp &qp, YuvComponent comp,
                           int width, int height,
                           const ResidualBufferConst &src1,
                           const SampleBufferConst &src2) const {
    return CompareSample(qp, comp, width, height,
                         src1.GetDataPtr(), src1.GetStride(),
                         src2.GetDataPtr(), src2.GetStride());
  }
  Distortion CompareSample(const Qp &qp, YuvComponent comp,
                           int width, int height,
                           const Residual *src1, ptrdiff_t stride1,
                           const Sample *src2, ptrdiff_t stride2) const {
    return Compare(qp, comp, width, height, src1, stride1, src2, stride2);
  }
  // Residual vs Residual
  Distortion CompareShort(const Qp &qp, YuvComponent comp,
                          int width, int height,
                          const ResidualBufferConst &src1,
                          const ResidualBufferConst &src2) const {
    return Compare(qp, comp, width, height, src1.GetDataPtr(), src1.GetStride(),
                   src2.GetDataPtr(), src2.GetStride());
  }
  Distortion CompareShort(const Qp &qp, YuvComponent comp,
                          int width, int height,
                          const Residual *src1, ptrdiff_t stride1,
                          const Residual *src2, ptrdiff_t stride2) const {
    return Compare(qp, comp, width, height, src1, stride1, src2, stride2);
  }

private:
  Distortion Compare(const Qp &qp, YuvComponent comp, int width, int height,
                     const Sample *src1, ptrdiff_t stride1,
                     const Sample *src2, ptrdiff_t stride2) const;
  Distortion Compare(const Qp &qp, YuvComponent comp, int width, int height,
                     const Residual *src1, ptrdiff_t stride1,
                     const Sample *src2, ptrdiff_t stride2) const;
  Distortion Compare(const Qp &qp, YuvComponent comp, int width, int height,
                     const Residual *src1, ptrdiff_t stride1,
                     const Residual *src2, ptrdiff_t stride2) const;
  template<bool RemoveAvg, typename SampleT1, typename SampleT2>
  uint64_t ComputeSatd(int width, int height, int offset,
                       const SampleT1 *sample1, ptrdiff_t stride1,
                       const SampleT2 *sample2, ptrdiff_t stride2) const;
  template<typename SampleT1, typename SampleT2>
  uint64_t ComputeSatdAcOnly(int width, int height,
                             const SampleT1 *sample1, ptrdiff_t stride1,
                             const SampleT2 *sample2, ptrdiff_t stride2) const;
  template<bool RemoveAvg, int W, int H, typename SampleT1, typename SampleT2>
  int ComputeSatdNxM(const SampleT1 *sample1, ptrdiff_t stride1,
                     const SampleT2 *sample2, ptrdiff_t stride2,
                     int offset) const;
  template<bool RemoveAvg, typename SampleT1, typename SampleT2>
  int ComputeSatd2x2(const SampleT1 *sample1, ptrdiff_t stride1,
                     const SampleT2 *sample2, ptrdiff_t stride2,
                     int offset) const;
  template<int SkipLines, typename SampleT1, typename SampleT2>
  uint64_t ComputeSadAcOnly(int width, int height,
                            const SampleT1 *sample1, ptrdiff_t stride1,
                            const SampleT2 *sample2, ptrdiff_t stride2) const;
  template<typename SampleT1, typename SampleT2>
  uint64_t
    ComputeStructuralSsdBlock(const Qp &qp, int N,
                              const SampleT1 *sample1, ptrdiff_t stride1,
                              const SampleT2 *sample2, ptrdiff_t stride2) const;
  template<typename SampleT1, typename SampleT2>
  uint64_t
    ComputeStructuralSsd(const Qp &qp, int width, int height,
                         const SampleT1 *sample1, ptrdiff_t stride1,
                         const SampleT2 *sample2, ptrdiff_t stride2) const;
  template<int SkipLines, typename SampleT1, typename SampleT2>
  int CalcMeanDiff(int width, int height,
                   const SampleT1 *sample1, ptrdiff_t stride1,
                   const SampleT2 *sample2, ptrdiff_t stride2) const;

  const SimdFunc &simd_func_;
  const int bitdepth_;
  const MetricType type_;
  const int structural_strength_;
};

struct SampleMetric::SimdFunc {
  static const int kMaxSize = constants::kCtuSizeLog2 + 1;
  SimdFunc();

  int(*sad_sample_sample[kMaxSize])(int width, int height,
                                    const Sample *sample1, ptrdiff_t stride1,
                                    const Sample *sample2, ptrdiff_t stride2);
  int(*sad_short_sample[kMaxSize])(int width, int height,
                                   const int16_t *sample1, ptrdiff_t stride1,
                                   const Sample *sample2, ptrdiff_t stride2);
  uint64_t(*ssd_sample_sample[kMaxSize])(int width, int height,
                                         const Sample *sample1,
                                         ptrdiff_t stride1,
                                         const Sample *sample2,
                                         ptrdiff_t stride2);
  uint64_t(*ssd_short_sample[kMaxSize])(int width, int height,
                                        const int16_t *sample1,
                                        ptrdiff_t stride1,
                                        const Sample *sample2,
                                        ptrdiff_t stride2);
  uint64_t(*ssd_short_short[kMaxSize])(int width, int height,
                                       const int16_t *sample1,
                                       ptrdiff_t stride1,
                                       const int16_t *sample2,
                                       ptrdiff_t stride2);
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_SAMPLE_METRIC_H_
