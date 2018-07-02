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

#ifndef XVC_COMMON_LIB_QUANTIZE_H_
#define XVC_COMMON_LIB_QUANTIZE_H_

#include <array>
#include <vector>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/picture_types.h"

namespace xvc {

class CodingUnit;

class Qp {
public:
  Qp(int qp, ChromaFormat chroma_format, int bitdepth, double lambda,
     int chroma_offset_table = 0, int chroma_offset_u = 0,
     int chroma_offset_v = 0);
  bool operator<(const Qp &qp) const {
    return qp_raw_[0] < qp.qp_raw_[0];
  }
  bool operator<=(const Qp &qp) const {
    return qp_raw_[0] <= qp.qp_raw_[0];
  }
  int GetQpRaw(YuvComponent comp) const {
    return qp_raw_[comp];
  }
  int GetQpPer(YuvComponent comp) const {
    return qp_bitdepth_[comp] / kNumScalingListRem_;
  }
  int GetFwdScale(YuvComponent comp) const {
    return kFwdQuantScales_[qp_bitdepth_[comp] % kNumScalingListRem_];
  }
  int GetInvScale(YuvComponent comp) const {
    return kInvQuantScales_[qp_bitdepth_[comp] % kNumScalingListRem_]
      << (qp_bitdepth_[comp] / kNumScalingListRem_);
  }
  double GetDistortionWeight(YuvComponent comp) const {
    return distortion_weight_[comp];
  }
  double GetLambda() const { return lambda_[0]; }
  double GetLambdaSqrt() const { return lambda_sqrt_; }
  double GetLambdaScaled(YuvComponent comp) const {
    return lambda_[static_cast<int>(comp)];
  }

private:
  static const int kChromaQpMax_ = 57;
  static const uint8_t kChromaScale_[kChromaQpMax_ + 1];
  static const int kNumScalingListRem_ = 6;
  static const int kFwdQuantScales_[kNumScalingListRem_];
  static const int kInvQuantScales_[kNumScalingListRem_];
  static int ScaleChromaQp(int qp, ChromaFormat chroma_format, int bitdepth,
                           int chroma_scaling_table, int offset);
  static double GetChromaDistWeight(int qp, ChromaFormat chroma_format,
                                    int chroma_scaling_table, int offset);

  std::array<int, constants::kMaxYuvComponents> qp_raw_;
  std::array<int, constants::kMaxYuvComponents> qp_bitdepth_;
  std::array<double, constants::kMaxYuvComponents> distortion_weight_;
  std::array<double, constants::kMaxYuvComponents> lambda_;
  double lambda_sqrt_;
};

class Quantize {
public:
  static const int kQuantShift = 14;
  static const int kIQuantShift = 6;

  void Inverse(YuvComponent comp, const Qp &qp, int width, int height,
               int bitdepth, const Coeff *in, ptrdiff_t in_stride, Coeff *out,
               ptrdiff_t out_stride);
  static int GetTransformShift(int width, int height, int bitdepth);
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_QUANTIZE_H_
