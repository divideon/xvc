/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/quantize.h"

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <limits>

#include "xvc_common_lib/utils.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/transform.h"

namespace xvc {

const uint8_t QP::kChromaScale_[QP::kChromaQpMax_ + 1] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21,
  22, 23, 24, 25, 26, 27, 28, 29, 29, 30, 31, 32, 33, 33, 34, 34, 35, 35, 36,
  36, 37, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51
};

const int QP::kFwdQuantScales_[kNumScalingListRem_] = {
  26214, 23302, 20560, 18396, 16384, 14564
};

const int QP::kInvQuantScales_[kNumScalingListRem_] = {
  40, 45, 51, 57, 64, 72
};

QP::QP(int qp_unscaled, ChromaFormat chroma_format, int bitdepth, double lambda,
       int chroma_offset)
  : lambda_(lambda),
  lambda_sqrt_(std::sqrt(lambda)) {
  int qp = ScaleLumaQP(qp_unscaled, bitdepth, lambda);
  qp_raw_[0] = qp;
  qp_raw_[1] = ScaleChromaQP(qp + chroma_offset, chroma_format, bitdepth);
  qp_raw_[2] = ScaleChromaQP(qp + chroma_offset, chroma_format, bitdepth);
  qp_bitdepth_[0] =
    std::max(0, qp_raw_[0] + kNumScalingListRem_ * (bitdepth - 8));
  qp_bitdepth_[1] =
    std::max(0, qp_raw_[1] + kNumScalingListRem_ * (bitdepth - 8));
  qp_bitdepth_[2] =
    std::max(0, qp_raw_[2] + kNumScalingListRem_ * (bitdepth - 8));
  distortion_weight_[0] = 1.0;
  distortion_weight_[1] =
    GetChromaDistWeight(qp + chroma_offset, chroma_format);
  distortion_weight_[2] =
    GetChromaDistWeight(qp + chroma_offset, chroma_format);
}

int QP::ScaleLumaQP(int qp, int bitdepth, double lambda) {
  if (lambda == 0) {
    return qp;
  }
  int qp_temp = qp - 12;
  double lambda_ref = 0.57 * std::pow(2.0, qp_temp / 3.0);
  int qp_offset = static_cast<int>(
    std::floor((3.0 * log(lambda / lambda_ref) / log(2.0)) + 0.5));
  return util::Clip3(qp + qp_offset, constants::kMinAllowedQp,
                     constants::kMaxAllowedQp);
}

int QP::ScaleChromaQP(int qp, ChromaFormat chroma_format, int bitdepth) {
  int chroma_qp = util::Clip3(qp, 0, kChromaQpMax_);
  if (chroma_format == ChromaFormat::k420) {
    chroma_qp = kChromaScale_[chroma_qp];
  }
  return chroma_qp;
}

double QP::GetChromaDistWeight(int qp, ChromaFormat chroma_format) {
  int chroma_qp = util::Clip3(qp, 0, kChromaQpMax_);
  int comp_qp_offset = 0;
  if (chroma_format == ChromaFormat::k420) {
    comp_qp_offset = kChromaScale_[chroma_qp] - chroma_qp;
  }
  return pow(2.0, -comp_qp_offset / 3.0);
}

double QP::CalculateLambda(int qp, PicturePredictionType pic_type,
                           int sub_gop_length, int temporal_id,
                           int max_temporal_id) {
  int qp_temp = qp - 12;
  double lambda = std::pow(2.0, qp_temp / 3.0);
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
    static const std::array<double, 5> temporal_factor = { {
        0.6, 0.2, 0.33, 0.33, 0.4
    } };
    hierarchical_factor =
      temporal_id == 0 ? 1 : util::Clip3(qp_temp / 6.0, 2.0, 4.0);
    return temporal_factor[temporal_id] * hierarchical_factor * lambda;
  }
  return pic_type_factor * subgop_factor * hierarchical_factor * lambda;
}

int Quantize::Forward(const CodingUnit &cu, YuvComponent comp, const QP &qp,
                      int width, int height,
                      int bitdepth, PicturePredictionType pic_type,
                      const Coeff *in, ptrdiff_t in_stride,
                      Coeff *out, ptrdiff_t out_stride) {
  const int transform_shift = GetTransformShift(width, height, bitdepth);
  const int shift = constants::kQuantShift + qp.GetQpPer(comp) +
    transform_shift + (width != height ? 7 : 0);
  const int scale = qp.GetFwdScale(comp) * (width != height ? 181 : 1);
  const int64_t offset =
    (pic_type == PicturePredictionType::kIntra ? 171ull : 85ull) << (shift - 9);
  Coeff delta[constants::kMaxBlockSize * constants::kMaxBlockSize];
  ptrdiff_t delta_stride = constants::kMaxBlockSize;

  const Coeff *in_ptr = in;
  Coeff *delta_ptr = delta;
  Coeff *out_ptr = out;

  int num_non_zero = 0;
  int abs_sum = 0;
  int delta_shift = shift - 8;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int sign = in_ptr[x] < 0 ? -1 : 1;
      int64_t level = std::abs(in_ptr[x]);
      int coeff = static_cast<int>(((level * scale) + offset) >> shift);
      abs_sum += coeff;
      delta_ptr[x] = static_cast<Coeff>(((level * scale) -
        (static_cast<int64_t>(coeff) << shift)) >> delta_shift);
      coeff *= sign;
      coeff = util::Clip3(coeff, constants::kInt16Min, constants::kInt16Max);
      out_ptr[x] = static_cast<Coeff>(coeff);
      if (coeff) {
        num_non_zero++;
      }
    }
    in_ptr += in_stride;
    delta_ptr += delta_stride;
    out_ptr += out_stride;
  }
  if (!Restrictions::Get().disable_transform_sign_hiding &&
      abs_sum > 1 && width >= 4 && height >= 4) {
    AdjustCoeffsForSignHiding(cu, comp, width, height, in, in_stride,
                              delta, delta_stride, out, out_stride);
  }
  return num_non_zero;
}

void Quantize::AdjustCoeffsForSignHiding(const CodingUnit &cu,
                                         YuvComponent comp, int width,
                                         int height, const Coeff *in,
                                         ptrdiff_t in_stride,
                                         const Coeff *delta,
                                         ptrdiff_t delta_stride,
                                         Coeff *out, ptrdiff_t out_stride) {
  const int subblock_shift = constants::kSubblockShift;
  const int subblock_width = width >> subblock_shift;
  const int subblock_height = height >> subblock_shift;
  const int subblock_size = 1 << (subblock_shift * 2);
  const int nbr_subblocks = subblock_width * subblock_height;

  std::vector<uint16_t> scan_subblock_table(nbr_subblocks);
  ScanOrder scan_order = TransformHelper::DetermineScanOrder(cu, comp);
  TransformHelper::DeriveSubblockScan(scan_order, subblock_width,
                                      subblock_height, &scan_subblock_table[0]);
  const uint8_t *scan_table = TransformHelper::GetCoeffScanTable4x4(scan_order);

  int subblock_last_index = subblock_width * subblock_height - 1;
  int last_subblock = -1;


  for (int subblock_index = subblock_last_index; subblock_index >= 0;
       subblock_index--) {
    int subblock_scan = scan_subblock_table[subblock_index];
    int subblock_scan_y = subblock_scan / subblock_width;
    int subblock_scan_x = subblock_scan - (subblock_scan_y * subblock_width);
    int subblock_pos_x = subblock_scan_x << subblock_shift;
    int subblock_pos_y = subblock_scan_y << subblock_shift;

    // Lambda help function for getting the value of a specific coefficient.
    auto get_coeff = [&scan_table, &subblock_pos_x, &subblock_pos_y]
    (const Coeff *coeff, ptrdiff_t stride, int idx) {
      constexpr int scan_shift = constants::kSubblockShift;
      constexpr int scan_mask = (1 << scan_shift) - 1;
      int scan_offset = scan_table[idx];
      int coeff_scan_x = subblock_pos_x + (scan_offset & scan_mask);
      int coeff_scan_y = subblock_pos_y + (scan_offset >> scan_shift);
      return coeff[coeff_scan_y * stride + coeff_scan_x];
    };

    // Lambda help function for adding value to a specific coefficient.
    auto change_coeff = [&scan_table, &subblock_pos_x, &subblock_pos_y]
    (Coeff *coeff, ptrdiff_t stride, int idx, Coeff value) {
      constexpr int scan_shift = constants::kSubblockShift;
      constexpr int scan_mask = (1 << scan_shift) - 1;
      int scan_offset = scan_table[idx];
      int coeff_scan_x = subblock_pos_x + (scan_offset & scan_mask);
      int coeff_scan_y = subblock_pos_y + (scan_offset >> scan_shift);
      coeff[coeff_scan_y * stride + coeff_scan_x] += value;
    };

    int last_nonzero_pos = -1;
    int first_nonzero_pos = subblock_size;
    int abs_sum = 0;
    for (int coeff_idx = 0; coeff_idx < subblock_size; coeff_idx++) {
      Coeff coeff = get_coeff(out, out_stride, coeff_idx);
      if (coeff) {
        first_nonzero_pos = std::min(first_nonzero_pos, coeff_idx);
        last_nonzero_pos = std::max(last_nonzero_pos, coeff_idx);
        abs_sum += coeff;
      }
    }

    if (last_nonzero_pos >= 0 && last_subblock == -1) {
      last_subblock = 1;
    }

    if (last_nonzero_pos - first_nonzero_pos > constants::SignHidingThreshold) {
      Coeff coeff = get_coeff(out, out_stride, first_nonzero_pos);
      int sign = (coeff > 0) ? 0 : 1;

      // If last bit in sum leads to wrong sign of first nonzero ceoff,
      // then one of the coefficients must be rounded differently.
      if (sign != (abs_sum & 0x1)) {
        Coeff curr_cost = std::numeric_limits<Coeff>::max();
        Coeff curr_change = 0;
        Coeff min_cost = std::numeric_limits<Coeff>::max();
        Coeff min_change = 0;
        int min_index = -1;

        int start = (last_subblock == 1) ? last_nonzero_pos : subblock_size - 1;
        for (int coeff_index = start; coeff_index >= 0; --coeff_index) {
          if (get_coeff(out, out_stride, coeff_index) != 0) {
            if (get_coeff(delta, delta_stride, coeff_index) > 0) {
              curr_cost = -get_coeff(delta, delta_stride, coeff_index);
              curr_change = 1;
            } else {
              if (coeff_index == first_nonzero_pos
                  && abs(get_coeff(out, out_stride, coeff_index)) == 1) {
                curr_cost = std::numeric_limits<Coeff>::max();
              } else {
                curr_cost = get_coeff(delta, delta_stride, coeff_index);
                curr_change = -1;
              }
            }
          } else {
            if (coeff_index < first_nonzero_pos) {
              int this_sign =
                get_coeff(in, in_stride, coeff_index) >= 0 ? 0 : 1;
              if (this_sign != sign) {
                curr_cost = std::numeric_limits<Coeff>::max();
              } else {
                curr_cost = -get_coeff(delta, delta_stride, coeff_index);
                curr_change = 1;
              }
            } else {
              curr_cost = -get_coeff(delta, delta_stride, coeff_index);
              curr_change = 1;
            }
          }

          if (curr_cost < min_cost) {
            min_cost = curr_cost;
            min_change = curr_change;
            min_index = coeff_index;
          }
        }

        if (get_coeff(out, out_stride, min_index) == constants::kInt16Min ||
            get_coeff(out, out_stride, min_index) == constants::kInt16Max) {
          min_change = -1;
        }

        if (get_coeff(in, in_stride, min_index) >= 0) {
          change_coeff(out, out_stride, min_index, min_change);
        } else {
          change_coeff(out, out_stride, min_index, -min_change);
        }
      }
    }
    if (last_subblock == 1) {
      last_subblock = 0;
    }
  }
}

void Quantize::Inverse(YuvComponent comp, const QP &qp, int width, int height,
                       int bitdepth, const Coeff *in, ptrdiff_t in_stride,
                       Coeff *out, ptrdiff_t out_stride) {
  const int transform_shift = GetTransformShift(width, height, bitdepth);
  const int shift = constants::kIQuantShift - transform_shift +
    (width != height ? 8 : 0);
  const int scale = qp.GetInvScale(comp) *  (width != height ? 181 : 1);

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
