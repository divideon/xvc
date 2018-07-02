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
