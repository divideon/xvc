/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_QUANTIZE_H_
#define XVC_COMMON_LIB_QUANTIZE_H_

#include <array>
#include <vector>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/picture_types.h"

namespace xvc {

class CodingUnit;

class QP {
public:
  QP(int qp, ChromaFormat chroma_format, int bitdepth, double lambda,
     int chroma_offset = 0);
  bool operator<(const QP &qp) const {
    return qp_raw_[0] < qp.qp_raw_[0];
  }
  bool operator<=(const QP &qp) const {
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
  double GetLambda() const { return lambda_; }
  double GetLambdaSqrt() const { return lambda_sqrt_; }

  static double CalculateLambda(int qp, PicturePredictionType pic_type,
                                int sub_gop_length, int temporal_id,
                                int max_temporal_id);
  static int GetQpFromLambda(int bitdepth, double lambda);

private:
  static const int kChromaQpMax_ = 57;
  static const uint8_t kChromaScale_[kChromaQpMax_ + 1];
  static const int kNumScalingListRem_ = 6;
  static const int kFwdQuantScales_[kNumScalingListRem_];
  static const int kInvQuantScales_[kNumScalingListRem_];
  static int ScaleChromaQP(int qp, ChromaFormat chroma_format, int bitdepth);
  static double GetChromaDistWeight(int qp, ChromaFormat chroma_format);

  std::array<int, constants::kMaxYuvComponents> qp_raw_;
  std::array<int, constants::kMaxYuvComponents> qp_bitdepth_;
  std::array<double, constants::kMaxYuvComponents> distortion_weight_;
  double lambda_;
  double lambda_sqrt_;
};

class Quantize {
public:
  int Forward(const CodingUnit &cu, YuvComponent comp, const QP &qp, int width,
              int height,
              int bitdepth, PicturePredictionType pic_type,
              const Coeff *in, ptrdiff_t in_stride,
              Coeff *out, ptrdiff_t out_stride);
  void Inverse(YuvComponent comp, const QP &qp, int width, int height,
               int bitdepth, const Coeff *in, ptrdiff_t in_stride, Coeff *out,
               ptrdiff_t out_stride);

private:
  static int GetTransformShift(int width, int height, int bitdepth);
  void AdjustCoeffsForSignHiding(const CodingUnit &cu, YuvComponent comp,
                                 int width, int height,
                                 const Coeff *in, ptrdiff_t in_stride,
                                 const Coeff *delta, ptrdiff_t delta_stride,
                                 Coeff *out, ptrdiff_t out_stride);
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_QUANTIZE_H_
