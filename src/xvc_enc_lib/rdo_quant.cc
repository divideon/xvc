/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/rdo_quant.h"

#include <algorithm>
#include <cassert>
#include <limits>
#include <vector>

#include "xvc_common_lib/transform.h"

namespace xvc {

int RdoQuant::QuantFast(const CodingUnit &cu, YuvComponent comp, const Qp &qp,
                        int width, int height, PicturePredictionType pic_type,
                        const Coeff *in, ptrdiff_t in_stride,
                        Coeff *out, ptrdiff_t out_stride) {
  const bool size_rounding_bias =
    (util::SizeToLog2(width) + util::SizeToLog2(height)) % 2 != 0;
  const int transform_shift =
    Quantize::GetTransformShift(width, height, bitdepth_);
  const int shift = Quantize::kQuantShift + qp.GetQpPer(comp) +
    transform_shift + (size_rounding_bias ? 7 : 0);
  const int scale = qp.GetFwdScale(comp) * (size_rounding_bias ? 181 : 1);
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

void RdoQuant::AdjustCoeffsForSignHiding(const CodingUnit &cu,
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

      // If last bit in sum leads to wrong sign of first nonzero coeff,
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

}   // namespace xvc
