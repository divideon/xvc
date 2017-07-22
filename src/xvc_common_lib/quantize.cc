/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/quantize.h"

#include <algorithm>
#include <cstdlib>
#include <cmath>

#include "xvc_common_lib/utils.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/transform.h"

namespace xvc {

const uint8_t Qp::kChromaScale_[Qp::kChromaQpMax_ + 1] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
  22, 23, 24, 25, 26, 27, 28, 29, 29, 30, 31, 32, 33, 33, 34, 34, 35, 35, 36,
  36, 37, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51
};

const int Qp::kFwdQuantScales_[kNumScalingListRem_] = {
  26214, 23302, 20560, 18396, 16384, 14564
};

const int Qp::kInvQuantScales_[kNumScalingListRem_] = {
  40, 45, 51, 57, 64, 72
};

Qp::Qp(int qp, ChromaFormat chroma_format, int bitdepth, double lambda,
       int chroma_offset_table, int chroma_offset_u, int chroma_offset_v)
  : lambda_({ lambda, lambda, lambda }),
  lambda_sqrt_(std::sqrt(lambda)) {
  qp_raw_[0] = qp;
  qp_raw_[1] = ScaleChromaQp(qp, chroma_format, bitdepth,
                             chroma_offset_table, chroma_offset_u);
  qp_raw_[2] = ScaleChromaQp(qp, chroma_format, bitdepth,
                             chroma_offset_table, chroma_offset_v);
  qp_bitdepth_[0] =
    std::max(0, qp_raw_[0] + kNumScalingListRem_ * (bitdepth - 8));
  qp_bitdepth_[1] =
    std::max(0, qp_raw_[1] + kNumScalingListRem_ * (bitdepth - 8));
  qp_bitdepth_[2] =
    std::max(0, qp_raw_[2] + kNumScalingListRem_ * (bitdepth - 8));
  distortion_weight_[0] = 1.0;
  distortion_weight_[1] =
    GetChromaDistWeight(qp, chroma_format,
                        chroma_offset_table, chroma_offset_u);
  distortion_weight_[2] =
    GetChromaDistWeight(qp, chroma_format,
                        chroma_offset_table, chroma_offset_v);
  lambda_[1] = lambda / distortion_weight_[1];
  lambda_[2] = lambda / distortion_weight_[2];
}

int Qp::GetQpFromLambda(int bitdepth, double lambda) {
  int qp = static_cast<int>(
    std::floor((3.0 * (log(lambda / 0.57) / log(2.0))) + 0.5));
  return util::Clip3(12 + qp, constants::kMinAllowedQp,
                     constants::kMaxAllowedQp);
}

int Qp::ScaleChromaQp(int qp, ChromaFormat chroma_format, int bitdepth,
                      int chroma_scaling_table, int offset) {
  int chroma_qp = util::Clip3(qp + offset, 0, kChromaQpMax_);
  if (chroma_format == ChromaFormat::k420 && chroma_scaling_table == 1) {
    chroma_qp = kChromaScale_[chroma_qp];
  }
  return chroma_qp;
}

double Qp::GetChromaDistWeight(int qp, ChromaFormat chroma_format,
                               int chroma_scaling_table, int offset) {
  int chroma_qp = util::Clip3(qp, 0, kChromaQpMax_);
  int chroma_qp_with_offset = util::Clip3(qp + offset, 0, kChromaQpMax_);
  int comp_qp_offset = 0;
  if (chroma_format == ChromaFormat::k420 && chroma_scaling_table == 1) {
    comp_qp_offset = kChromaScale_[chroma_qp_with_offset] - chroma_qp;
  }
  return pow(2.0, -comp_qp_offset / 3.0);
}

double Qp::CalculateLambda(int qp, PicturePredictionType pic_type,
                           int sub_gop_length, int temporal_id,
                           int max_temporal_id, int scaling_type) {
  int qp_temp = qp - 12;
  double lambda = pow(2.0, qp_temp / 3.0);
  double pic_type_factor =
    pic_type == PicturePredictionType::kIntra ? 0.57 : 0.68;
  double subgop_factor =
    1.0 - util::Clip3(0.05 * (sub_gop_length - 1), 0.0, 0.5);
  double hierarchical_factor = 1;
  if (temporal_id > 0 && temporal_id == max_temporal_id) {
    subgop_factor = 1.0;
    hierarchical_factor = util::Clip3(qp_temp / 6.0, 2.0, 4.0);
  } else if (temporal_id > 0) {
    hierarchical_factor = util::Clip3(qp_temp / 6.0, 2.0, 4.0);
    hierarchical_factor *= 0.8;
  }
  if (sub_gop_length == 16 && pic_type != PicturePredictionType::kIntra) {
    if (scaling_type == 0) {
      static const std::array<double, 5> temporal_factor = { {
          0.6, 0.2, 0.33, 0.33, 0.4
        } };
      hierarchical_factor =
        temporal_id == 0 ? 1 : util::Clip3(qp_temp / 6.0, 2.0, 4.0);
      return temporal_factor[temporal_id] * hierarchical_factor * lambda;
    } else {
      static const std::array<double, 5> temporal_factor = { {
          0.14, 0.2, 0.33, 0.33, 0.4
        } };
      hierarchical_factor = util::Clip3(qp_temp / 6.0, 2.0, 4.0);
    return temporal_factor[temporal_id] * hierarchical_factor * lambda;
    }
  }
  return pic_type_factor * subgop_factor * hierarchical_factor * lambda;
}

void Quantize::Inverse(YuvComponent comp, const Qp &qp, int width, int height,
                       int bitdepth, const Coeff *in, ptrdiff_t in_stride,
                       Coeff *out, ptrdiff_t out_stride) {
  const bool size_rounding_bias =
    (util::SizeToLog2(width) + util::SizeToLog2(height)) % 2 != 0;
  const int transform_shift = GetTransformShift(width, height, bitdepth);
  const int shift = kIQuantShift - transform_shift +
    (size_rounding_bias ? 8 : 0);
  const int scale = qp.GetInvScale(comp) * (size_rounding_bias ? 181 : 1);

  if (shift > 0) {
    int offset = (1 << (shift - 1));
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int coeff = ((in[x] * scale) + offset) >> shift;
        out[x] = util::Clip3(coeff, constants::kInt16Min, constants::kInt16Max);
      }
      in += in_stride;
      out += out_stride;
    }
  } else {
    int inv_shift = -shift;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int coeff = ((in[x] * scale)) << inv_shift;
        out[x] = util::Clip3(coeff, constants::kInt16Min, constants::kInt16Max);
      }
      in += in_stride;
      out += out_stride;
    }
  }
}

int Quantize::GetTransformShift(int width, int height, int bitdepth) {
  const int tr_size_log2 =
    (util::SizeToLog2(width) + util::SizeToLog2(height)) >> 1;
  return constants::kMaxTrDynamicRange - bitdepth - tr_size_log2;
}

}   // namespace xvc
