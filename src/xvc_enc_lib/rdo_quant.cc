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

#include "xvc_enc_lib/rdo_quant.h"

#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

#include "xvc_common_lib/transform.h"
#include "xvc_enc_lib/encoder_settings.h"

namespace xvc {

struct RdoQuant::CoeffCodingState {
  int ctx_set;
  int c1 = 1;
  int c2 = 0;
  int c1_idx = 0;
  int c2_idx = 0;
  uint32_t golomb_rice_k = 0;
};

template<int SubBlockShift>
class SubblockScan {
public:
  struct SubblockIterator {
  public:
    struct CoeffIterator {
      CoeffIterator(const SubblockIterator &subblock, int i)
        : offset(i),
        subblock_(subblock) {
        Update();
      }
      bool operator!=(const CoeffIterator &other) {
        return offset != other.offset;
      }
      CoeffIterator& operator++() {
        offset--;
        Update();
        return *this;
      }
      CoeffIterator& operator*() {
        return *this;
      }
      int offset;
      int index;
      int scan_x;
      int scan_y;

    private:
      void Update() {
        if (offset >= 0) {
          index = subblock_.index + offset;
          int scan_pos = subblock_.subblock_scan_.coeff_scan_table_[offset];
          scan_x = subblock_.pos_x + (scan_pos & ((1 << SubBlockShift) - 1));
          scan_y = subblock_.pos_y + (scan_pos >> SubBlockShift);
        }
      }
      const SubblockIterator &subblock_;
    };

    SubblockIterator(const SubblockScan &subblock, int i)
      : i_(i),
      subblock_scan_(subblock) {
      Update();
    }
    bool operator!=(const SubblockIterator &other) {
      return i_ != other.i_;
    }
    SubblockIterator& operator++() {
      i_--;
      Update();
      return *this;
    }
    SubblockIterator& operator*() {
      return *this;
    }
    CoeffIterator at(int i) {
      return CoeffIterator(*this, i);
    }
    CoeffIterator begin() {
      constexpr int subblock_size = 1 << (SubBlockShift * 2);
      return CoeffIterator(*this, subblock_size - 1);
    }
    CoeffIterator end() {
      return CoeffIterator(*this, -1);
    }
    int scan = 0;
    int scan_y = 0;
    int scan_x = 0;
    int index = 0;
    int pos_x = 0;
    int pos_y = 0;

  private:
    void Update() {
      if (i_ >= 0) {
        index = i_ << (2 * SubBlockShift);
        scan = subblock_scan_.scan_table_[i_];
        scan_y = scan / subblock_scan_.width_;
        scan_x = scan - (scan_y * subblock_scan_.width_);
        pos_x = scan_x << SubBlockShift;
        pos_y = scan_y << SubBlockShift;
      }
    }
    int i_;
    const SubblockScan &subblock_scan_;
  };
  SubblockScan(ScanOrder scan_order, int width, int height)
    : width_(width >> SubBlockShift),
    height_(height >> SubBlockShift),
    coeff_scan_table_(SubBlockShift == 1 ?
                      TransformHelper::GetCoeffScanTable2x2(scan_order) :
                      TransformHelper::GetCoeffScanTable4x4(scan_order)),
    scan_table_(width_ * height_) {
    TransformHelper::DeriveSubblockScan(scan_order, width_, height_,
                                        &scan_table_[0]);
  }
  SubblockIterator begin() const {
    return SubblockIterator(*this, width_ * height_ - 1);
  }
  SubblockIterator end() const {
    return SubblockIterator(*this, -1);
  }

private:
  const int width_;
  const int height_;
  const uint8_t *coeff_scan_table_;
  std::vector<uint16_t> scan_table_;
};

int RdoQuant::QuantFast(const CodingUnit &cu, YuvComponent comp, const Qp &qp,
                        PicturePredictionType pic_type,
                        const Coeff *in, ptrdiff_t in_stride,
                        Coeff *out, ptrdiff_t out_stride) {
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
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
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int sign = in_ptr[x] < 0 ? -1 : 1;
      int64_t abs_coeff = std::abs(in_ptr[x]);
      int level = static_cast<int>(((abs_coeff * scale) + offset) >> shift);
      num_non_zero += level != 0;
      int coeff =
        util::Clip3(level * sign, constants::kInt16Min, constants::kInt16Max);
      out_ptr[x] = static_cast<Coeff>(coeff);
      delta_ptr[x] = static_cast<Coeff>(((abs_coeff * scale) -
        (static_cast<int64_t>(level) << shift)) >> (shift - 8));
    }
    in_ptr += in_stride;
    delta_ptr += delta_stride;
    out_ptr += out_stride;
  }
  if (!Restrictions::Get().disable_transform_sign_hiding &&
      num_non_zero > 1 && width >= 4 && height >= 4) {
    num_non_zero = CoeffSignHideFast(cu, comp, width, height, in, in_stride,
                                     delta, delta_stride, out, out_stride);
  }
  return num_non_zero;
}

int RdoQuant::QuantRdo(const CodingUnit &cu, YuvComponent comp,
                       const Qp &qp, PicturePredictionType pic_type,
                       const SyntaxWriter &writer,
                       const Coeff *src, ptrdiff_t src_stride,
                       Coeff *out, ptrdiff_t out_stride) {
  if (cu.GetWidth(comp) == 2 || cu.GetHeight(comp) == 2) {
    if (EncoderSettings::rdo_quant_size_2) {
      return QuantRdo<1>(cu, comp, qp, pic_type, writer, src, src_stride,
                         out, out_stride);
    } else {
      return QuantFast(cu, comp, qp, pic_type, src, src_stride,
                       out, out_stride);
    }
  } else {
    return
      QuantRdo<constants::kSubblockShift>(cu, comp, qp, pic_type, writer,
                                          src, src_stride, out, out_stride);
  }
}

template<int SubBlockShift>
int RdoQuant::QuantRdo(const CodingUnit &cu, YuvComponent comp,
                       const Qp &qp, PicturePredictionType pic_type,
                       const SyntaxWriter &writer,
                       const Coeff *src, ptrdiff_t src_stride,
                       Coeff *out, ptrdiff_t out_stride) {
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const int width_log2 = util::SizeToLog2(width);
  const int height_log2 = util::SizeToLog2(height);
  const int subblock_shift = SubBlockShift;
  const int subblock_width = width >> subblock_shift;
  const int subblock_height = height >> subblock_shift;
  constexpr int subblock_size = 1 << (subblock_shift * 2);
  const int transform_shift =
    Quantize::GetTransformShift(width, height, bitdepth_);
  const int size_rounding_bias =
    (util::SizeToLog2(width) + util::SizeToLog2(height)) % 2 != 0 ? 1 : 0;
  const int shift = Quantize::kQuantShift + qp.GetQpPer(comp) +
    transform_shift;
  const int size_bias_shift = size_rounding_bias ? 7 : 0;
  const int size_bias_offset = size_rounding_bias ?
    (1 << (size_bias_shift - 1)) : 0;
  const int scale = qp.GetFwdScale(comp) * (size_rounding_bias ? 181 : 1);
  // Scale coeff distortion to a magnitude that is compatible with lambda
  const int cost_scale =
    ContextModel::kFracBitsPrecision - 2 * transform_shift -
    2 * (bitdepth_ - 8) + 2 * size_rounding_bias;
  const int64_t lambda = static_cast<int64_t>(qp.GetLambdaScaled(comp) *
    (1 << kLambdaPrecision) + 0.5);
  // TODO(PH) Fix cabac context get methods to be const!
  Contexts *contexts = const_cast<Contexts*>(&writer.GetContexts());

  const ScanOrder scan_order = TransformHelper::DetermineScanOrder(cu, comp);
  const auto fwd_quant = GetFwdQuantFunc(comp, qp, width, height);
  const auto inv_quant = GetInvQuantFunc(comp, qp, width, height);

  constexpr int kMaxSubblockSize = constants::kMaxBlockSize >> SubBlockShift;
  uint8_t subblock_csbf[kMaxSubblockSize * kMaxSubblockSize];
  Bits csbf_bits_to_zero[kMaxSubblockSize * kMaxSubblockSize];
  ::memset(&subblock_csbf[0], 0, sizeof(subblock_csbf));
  ::memset(&err_dist_[0], 0, sizeof(err_dist_[0]) * width * height);
  ::memset(&sig_rate_[0], 0, sizeof(sig_rate_[0]) * width * height);
  ::memset(&rate_up_[0], 0, sizeof(rate_up_[0]) * width * height);
  ::memset(&rate_down_[0], 0, sizeof(rate_down_[0]) * width * height);

  CoeffCodingState code_state;
  int last_pos_index = -1;
  int64_t comp_zero_dist = 0;
  int64_t comp_code_cost = 0;

  SubblockScan<SubBlockShift> subblock_scan(scan_order, width, height);
  for (auto &subblock : subblock_scan) {
    int last_c1 = code_state.c1;
    code_state = CoeffCodingState();
    code_state.ctx_set = (subblock.index > 0 && util::IsLuma(comp)) ? 2 : 0;
    if (last_c1 == 0) {
      code_state.ctx_set++;
    }

    int64_t subblock_zero_dist = 0;
    int64_t subblock_code_cost = 0;

    int pattern_sig_ctx = 0;
    ContextModel &csbf_ctx =
      contexts->GetSubblockCsbfCtx(comp, &subblock_csbf[0], subblock.scan_x,
                                   subblock.scan_y, subblock_width,
                                   subblock_height, &pattern_sig_ctx);
    int num_non_zero = 0;

    for (auto &coeff : subblock) {
      Coeff abs_coeff = static_cast<Coeff>(
        std::abs(src[coeff.scan_y * src_stride + coeff.scan_x]));
      int64_t coeff_zero_cost =
        static_cast<int64_t>(abs_coeff * abs_coeff) << cost_scale;
      subblock_zero_dist += coeff_zero_cost;

      Coeff quant_coeff = fwd_quant(abs_coeff);

      if (quant_coeff && last_pos_index == -1) {
        last_pos_index = coeff.index;
      } else if (last_pos_index == -1) {
        out[coeff.scan_y * out_stride + coeff.scan_x] = 0;
        subblock_code_cost += coeff_zero_cost;
        continue;
      }

      const ContextModel &sig_ctx =
        contexts->GetCoeffSigCtx(comp, pattern_sig_ctx, scan_order,
                                 coeff.scan_x, coeff.scan_y, out, out_stride,
                                 width_log2, height_log2);
      const ContextModel &c1_ctx =
        contexts->GetCoeffGreater1Ctx(comp, code_state.ctx_set, code_state.c1,
                                      coeff.scan_x, coeff.scan_y,
                                      coeff.index == last_pos_index,
                                      out, out_stride, width, height);
      const ContextModel &c2_ctx =
        contexts->GetCoeffGreater2Ctx(comp, code_state.ctx_set,
                                      coeff.scan_x, coeff.scan_y,
                                      coeff.index == last_pos_index,
                                      out, out_stride, width, height);
      if (!Restrictions::Get().disable_ext_cabac_alt_residual_ctx) {
        code_state.golomb_rice_k =
          contexts->GetCoeffGolombRiceK(coeff.scan_x, coeff.scan_y,
                                        width, height, out, out_stride);
      }
      const Bits sig0_bits = sig_ctx.GetEntropyBits(0);
      Bits sig1_bits = sig_ctx.GetEntropyBits(1);
      if (last_pos_index == coeff.index ||
        (subblock.index > 0 && coeff.offset == 0 && num_non_zero == 0)) {
        // Implicitly signaled coeff sig flag
        sig1_bits = 0;
      }

      int64_t best_cost = std::numeric_limits<int64_t>::max();
      Bits best_cost_sig = 0;
      Coeff best_level = quant_coeff;
      if (quant_coeff > 0) {
        best_cost_sig = sig1_bits;
        best_level = QuantCoeffRdo(comp, abs_coeff, quant_coeff, code_state,
                                   sig1_bits, lambda, cost_scale,
                                   c1_ctx, c2_ctx, inv_quant, &best_cost);
      }
      if (last_pos_index != coeff.index && quant_coeff < 3) {
        // Eval setting coefficient to zero
        int64_t cost = coeff_zero_cost + BitCost(sig0_bits, lambda);
        if (cost <= best_cost) {
          best_cost = cost;
          best_cost_sig = sig0_bits;
          best_level = 0;
        }
      }
      out[coeff.scan_y * out_stride + coeff.scan_x] = best_level;
      coeff_cost_to_zero_[coeff.index] = coeff_zero_cost - best_cost;
      coeff_sig_bits_[coeff.index] = best_cost_sig;
      subblock_code_cost += best_cost;
      int64_t orig_scaled = ((static_cast<int64_t>(abs_coeff) * scale +
                              size_bias_offset) >> size_bias_shift);
      int64_t quant_err =
        orig_scaled - (static_cast<int64_t>(best_level) << shift);
      err_dist_[coeff.index] = static_cast<Coeff>(quant_err >> (shift - 8));
      sig_rate_[coeff.index] =
        last_pos_index != coeff.index ? sig1_bits - sig0_bits : 0;
      if (best_level) {
        subblock_csbf[subblock.scan] = 1;
        num_non_zero++;
        int lvl_rate =
          GetAbsLevelBits(comp, best_level, c1_ctx, c2_ctx, code_state);
        rate_up_[coeff.index] = -lvl_rate +
          GetAbsLevelBits(comp, best_level + 1, c1_ctx, c2_ctx, code_state);
        rate_down_[coeff.index] = -lvl_rate +
          GetAbsLevelBits(comp, best_level - 1, c1_ctx, c2_ctx, code_state);
      } else {
        rate_up_[coeff.index] = c1_ctx.GetEntropyBits(0);
      }
      UpdateCodeState(comp, best_level, &code_state);
    }

    // Try to zero out whole subblock
    if (EvalZeroSubblock(subblock.index, subblock_size,
                         subblock_csbf[subblock.scan] != 0, csbf_ctx,
                         last_pos_index, subblock_zero_dist, lambda,
                         &csbf_bits_to_zero[subblock.scan],
                         &subblock_code_cost)) {
      subblock_csbf[subblock.scan] = 0;
      for (auto &coeff : subblock) {
        out[coeff.scan_y * out_stride + coeff.scan_x] = 0;
        coeff_cost_to_zero_[coeff.index] = 0;
      }
    }

    comp_code_cost += subblock_code_cost;
    comp_zero_dist += subblock_zero_dist;
  }
  if (last_pos_index < 0) {
    // No significant coefficients found
    return 0;
  }

  // Last position evaluation
  last_pos_index = EvalLastPos<SubBlockShift>(
    cu, comp, scan_order, contexts, last_pos_index, lambda, comp_code_cost,
    comp_zero_dist, out, out_stride, subblock_csbf, csbf_bits_to_zero);
  if (last_pos_index < 0) {
    // Zero out whole component
    return 0;
  }

  // Zero out unused coefficients
  int last_subblock_index =
    last_pos_index - (last_pos_index & (subblock_size - 1));
  for (auto &subblock : subblock_scan) {
    if (subblock.index < last_subblock_index) {
      break;
    }
    int last_pos_index_end = 0;
    if (subblock.index == last_subblock_index) {
      last_pos_index_end = last_pos_index % subblock_size;
    }
    for (auto coeff = subblock.begin();
         coeff != subblock.at(last_pos_index_end - 1); ++coeff) {
      out[coeff.scan_y * out_stride + coeff.scan_x] = 0;
    }
  }

  // Re-apply sign
  int num_non_zero = 0;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      Coeff level = out[y * out_stride + x];
      num_non_zero += level != 0;
      out[y * out_stride + x] = (src[y * src_stride + x] < 0) ? -level : level;
    }
  }

  // Sign hiding
  if (!Restrictions::Get().disable_transform_sign_hiding &&
      num_non_zero > 1 && SubBlockShift > 1) {
    num_non_zero =
      CoeffSignHideRdo(cu, comp, qp, src, src_stride, out, out_stride);
  }

  return num_non_zero;
}

int RdoQuant::CoeffSignHideFast(const CodingUnit &cu, YuvComponent comp,
                                int width, int height,
                                const Coeff *in, ptrdiff_t in_stride,
                                const Coeff *delta, ptrdiff_t delta_stride,
                                Coeff *out, ptrdiff_t out_stride) const {
  constexpr int subblock_shift = constants::kSubblockShift;
  const int subblock_size = 1 << (subblock_shift * 2);
  const ScanOrder scan_order = TransformHelper::DetermineScanOrder(cu, comp);
  const uint8_t *scan_table = TransformHelper::GetCoeffScanTable4x4(scan_order);
  int num_non_zero = 0;
  int last_subblock = -1;

  SubblockScan<subblock_shift> subblock_scan_(scan_order, width, height);
  for (auto &subblock : subblock_scan_) {
    auto get_coeff = [&scan_table, &subblock]
    (const Coeff *coeff, ptrdiff_t stride, int idx) {
      constexpr int scan_shift = constants::kSubblockShift;
      constexpr int scan_mask = (1 << scan_shift) - 1;
      int scan_offset = scan_table[idx];
      int coeff_scan_x = subblock.pos_x + (scan_offset & scan_mask);
      int coeff_scan_y = subblock.pos_y + (scan_offset >> scan_shift);
      return coeff[coeff_scan_y * stride + coeff_scan_x];
    };
    auto change_coeff = [&scan_table, &subblock]
    (Coeff *coeff, ptrdiff_t stride, int idx, Coeff value) {
      constexpr int scan_shift = constants::kSubblockShift;
      constexpr int scan_mask = (1 << scan_shift) - 1;
      int scan_offset = scan_table[idx];
      int coeff_scan_x = subblock.pos_x + (scan_offset & scan_mask);
      int coeff_scan_y = subblock.pos_y + (scan_offset >> scan_shift);
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
        num_non_zero++;
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

        if (!get_coeff(out, out_stride, min_index)) {
          num_non_zero++;
        }
        if (get_coeff(in, in_stride, min_index) >= 0) {
          change_coeff(out, out_stride, min_index, min_change);
        } else {
          change_coeff(out, out_stride, min_index, -min_change);
        }
        if (!get_coeff(out, out_stride, min_index)) {
          num_non_zero--;
        }
      }
    }
    if (last_subblock == 1) {
      last_subblock = 0;
    }
  }
  return num_non_zero;
}

int
RdoQuant::CoeffSignHideRdo(const CodingUnit &cu, YuvComponent comp,
                           const Qp &qp,
                           const Coeff *src, ptrdiff_t src_stride,
                           Coeff *out, ptrdiff_t out_stride) const {
  constexpr int SubBlockShift = constants::kSubblockShift;
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const int subblock_shift = SubBlockShift;
  constexpr int subblock_mask = (1 << subblock_shift) - 1;
  constexpr int subblock_size = 1 << (subblock_shift * 2);
  const ScanOrder scan_order = TransformHelper::DetermineScanOrder(cu, comp);
  const uint8_t *scan_table = SubBlockShift == 1 ?
    TransformHelper::GetCoeffScanTable2x2(scan_order) :
    TransformHelper::GetCoeffScanTable4x4(scan_order);
  const double lambda = qp.GetLambdaScaled(comp);
  const double inv_scale = qp.GetInvScale(comp);
  const int64_t rd_factor = static_cast<int64_t>(
    inv_scale * inv_scale / lambda / subblock_size /
    (1ull << (2 * (bitdepth_ - 8))) + 0.5);
  int num_non_zero = 0;

  int is_last_subblock = -1;
  SubblockScan<SubBlockShift> subblock_scan_(scan_order, width, height);
  for (auto &subblock : subblock_scan_) {
    int first_in_subblock = subblock_size;
    int last_in_subblock = -1;
    int subblock_sum = 0;
    for (auto &coeff : subblock) {
      if (out[coeff.scan_y * out_stride + coeff.scan_x]) {
        first_in_subblock = std::min(first_in_subblock, coeff.offset);
        last_in_subblock = std::max(last_in_subblock, coeff.offset);
        subblock_sum += out[coeff.scan_y * out_stride + coeff.scan_x];
        num_non_zero++;
      }
    }
    if (last_in_subblock >= 0 && is_last_subblock == -1) {
      is_last_subblock = 1;
    }
    if (last_in_subblock - first_in_subblock < 4) {
      if (is_last_subblock == 1) {
        is_last_subblock = 0;
      }
      continue;
    }
    const int first_scan_offset = scan_table[first_in_subblock];
    const int first_coeff_scan_x =
      subblock.pos_x + (first_scan_offset & subblock_mask);
    const int first_coeff_scan_y =
      subblock.pos_y + (first_scan_offset >> subblock_shift);
    int first_sign =
      (out[first_coeff_scan_y * out_stride + first_coeff_scan_x] > 0 ? 0 : 1);
    if (first_sign == (subblock_sum & 0x1)) {
      if (is_last_subblock == 1) {
        is_last_subblock = 0;
      }
      continue;
    }
    const int start_coeff_index =
      is_last_subblock == 1 ? last_in_subblock : subblock_size - 1;
    int64_t best_cost = std::numeric_limits<int64_t>::max();
    Coeff best_level_delta = 0;
    int best_scan_x = -1;
    int best_scan_y = -1;
    for (auto coeff = subblock.at(start_coeff_index);
         coeff != subblock.end(); ++coeff) {
      const Coeff coeff_lvl = out[coeff.scan_y * out_stride + coeff.scan_x];
      int64_t cost;
      Coeff level_delta = 0;
      if (coeff_lvl != 0) {
        int64_t cost_inc =
          rd_factor * (-err_dist_[coeff.index]) + rate_up_[coeff.index];
        int64_t cost_dec =
          rd_factor * (err_dist_[coeff.index]) + rate_down_[coeff.index]
          - (std::abs(coeff_lvl) == 1 ? sig_rate_[coeff.index] : 0);
        if (is_last_subblock == 1 && coeff.offset == last_in_subblock &&
            std::abs(coeff_lvl) == 1) {
          cost_dec -= 4 * ContextModel::kEntropyBypassBits;
        }
        if (cost_inc < cost_dec) {
          cost = cost_inc;
          level_delta = 1;
        } else {
          level_delta = -1;
          if (coeff.offset == first_in_subblock && std::abs(coeff_lvl) == 1) {
            cost = std::numeric_limits<int>::max();
          } else {
            cost = cost_dec;
          }
        }
      } else {
        cost = rd_factor * -(std::abs(err_dist_[coeff.index])) +
          rate_up_[coeff.index] + sig_rate_[coeff.index] +
          ContextModel::kEntropyBypassBits;
        level_delta = 1;
        if (coeff.offset < first_in_subblock) {
          int sign =
            (src[coeff.scan_y * src_stride + coeff.scan_x] >= 0 ? 0 : 1);
          if (sign != first_sign) {
            cost = std::numeric_limits<int>::max();
          }
        }
      }
      if (cost < best_cost) {
        best_cost = cost;
        best_level_delta = level_delta;
        best_scan_x = coeff.scan_x;
        best_scan_y = coeff.scan_y;
      }
    }
    if (out[best_scan_y * out_stride + best_scan_x] == 32767 ||
        out[best_scan_y * out_stride + best_scan_x] == -32768) {
      best_level_delta = -1;
    }
    if (!out[best_scan_y * out_stride + best_scan_x]) {
      num_non_zero++;
    }
    if (src[best_scan_y * src_stride + best_scan_x] >= 0) {
      out[best_scan_y * out_stride + best_scan_x] += best_level_delta;
    } else {
      out[best_scan_y * out_stride + best_scan_x] -= best_level_delta;
    }
    if (!out[best_scan_y * out_stride + best_scan_x]) {
      num_non_zero--;
    }
    if (is_last_subblock == 1) {
      is_last_subblock = 0;
    }
  }
  return num_non_zero;
}

Coeff
RdoQuant::QuantCoeffRdo(YuvComponent comp, Coeff orig_coeff, Coeff max_level,
                        const CoeffCodingState &code_state, Bits sig1_bits,
                        int64_t lambda, int cost_scale,
                        const ContextModel &c1_ctx, const ContextModel &c2_ctx,
                        const std::function<Coeff(Coeff)> &inv_quant,
                        int64_t *out_cost) const {
  int64_t best_cost = std::numeric_limits<int64_t>::max();
  Coeff best_level = max_level;
  auto get_cost = [&](Coeff level) {
    Bits bits =
      sig1_bits + GetAbsLevelBits(comp, level, c1_ctx, c2_ctx, code_state);
    Coeff dequant = inv_quant(level);
    int64_t err = static_cast<int>(orig_coeff) - dequant;
    int64_t dist = (err * err) << cost_scale;
    return dist + BitCost(bits, lambda);
  };
  if (max_level > 1) {
    int64_t cost = get_cost(max_level - 1);
    best_cost = cost;
    best_level = max_level - 1;
  }
  int64_t cost = get_cost(max_level);
  if (cost <= best_cost) {
    best_cost = cost;
    best_level = max_level;
  }
  *out_cost = best_cost;
  return best_level;
}

bool
RdoQuant::EvalZeroSubblock(int subblock_index, int size, bool subblock_csbf,
                           const ContextModel &csbf_ctx, int last_pos_index,
                           int64_t subblock_zero_dist, int64_t lambda,
                           Bits *csbf_bits_to_zero,
                           int64_t *subblock_code_cost) const {
  if (last_pos_index < 0) {
    // No last coeff found yet
    *csbf_bits_to_zero = 0;
    return false;
  } else if (subblock_index == 0 ||
             subblock_index + size > last_pos_index) {
    // Implicitly signaled csbf
    *csbf_bits_to_zero = 0;
    return false;
  }
  const Bits csbf_zero_cost = csbf_ctx.GetEntropyBits(0);
  const Bits csbf_code_bits = csbf_ctx.GetEntropyBits(1);
  const int64_t subblock_zero_cost = subblock_zero_dist +
    BitCost(csbf_zero_cost, lambda);
  if (subblock_csbf) {
    int64_t cost_cost = *subblock_code_cost + BitCost(csbf_code_bits, lambda);
    if (subblock_zero_cost < cost_cost) {
      *subblock_code_cost = subblock_zero_cost;
      *csbf_bits_to_zero = csbf_zero_cost;
      return true;
    } else {
      *subblock_code_cost = cost_cost;
      *csbf_bits_to_zero = csbf_code_bits;
    }
  } else {
    *subblock_code_cost = subblock_zero_cost;
    *csbf_bits_to_zero = csbf_zero_cost;
  }
  return false;
}

template<int SubBlockShift>
int
RdoQuant::EvalLastPos(const CodingUnit &cu, YuvComponent comp,
                      ScanOrder scan_order, Contexts *contexts,
                      int last_pos_index, int64_t lambda,
                      int64_t comp_code_cost, int64_t comp_zero_dist,
                      const Coeff *out, ptrdiff_t out_stride,
                      const uint8_t *subblock_csbf,
                      const Bits *csbf_bits_to_zero) const {
  constexpr int subblock_size = 1 << (SubBlockShift * 2);
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  SubblockScan<SubBlockShift> subblock_scan(scan_order, width, height);

  static_assert(1 == Contexts::kNumCuCbfCtxLuma, "only 1 cbf ctx");
  static_assert(1 == Contexts::kNumCuCbfCtxChroma, "only 1 cbf ctx");
  static_assert(1 == Contexts::kNumCuRootCbfCtx, "only 1 root cbf ctx");
  ContextModel &cbf_ctx = !util::IsLuma(comp) ? contexts->cu_cbf_chroma[0] :
    (cu.IsIntra() ? contexts->cu_cbf_luma[0] : contexts->cu_root_cbf[0]);
  comp_code_cost += BitCost(cbf_ctx.GetEntropyBits(1), lambda);

  int start_last_index = last_pos_index % subblock_size;
  int64_t best_cost = std::numeric_limits<int64_t>::max();
  int best_last_pos_plus1 = 0;
  bool stop_search = false;
  for (auto &subblock : subblock_scan) {
    if (subblock.index > last_pos_index) {
      continue;
    }
    comp_code_cost -= BitCost(csbf_bits_to_zero[subblock.scan], lambda);
    if (!subblock_csbf[subblock.scan]) {
      continue;
    }
    for (auto coeff = subblock.at(start_last_index);
         coeff != subblock.end(); ++coeff) {
      Coeff coeff_val = out[coeff.scan_y * out_stride + coeff.scan_x];
      if (!coeff_val) {
        comp_code_cost += coeff_cost_to_zero_[coeff.index];
        continue;
      }
      Bits last_pos_bits = GetLastPosBits(width, height, comp, scan_order,
                                          contexts, coeff.scan_x, coeff.scan_y);
      Bits implicit_sig = coeff_sig_bits_[coeff.index];
      int64_t cost = comp_code_cost + BitCost(last_pos_bits, lambda) -
        BitCost(implicit_sig, lambda);
      if (cost < best_cost) {
        best_cost = cost;
        best_last_pos_plus1 = coeff.index + 1;
      }
      if (coeff_val > 1) {
        stop_search = true;
        break;
      }
      comp_code_cost += coeff_cost_to_zero_[coeff.index];
    }
    if (stop_search) {
      break;
    }
    start_last_index = subblock_size - 1;
  }
  // Compare against zero out whole component
  int64_t comp_zero_cost = comp_zero_dist +
    BitCost(cbf_ctx.GetEntropyBits(0), lambda);
  if (comp_zero_cost < best_cost) {
    return -1;
  }
  return best_last_pos_plus1;
}

Bits RdoQuant::GetAbsLevelBits(YuvComponent comp, Coeff quant_level,
                               const ContextModel &c1_ctx,
                               const ContextModel &c2_ctx,
                               const CoeffCodingState &state) const {
  const Coeff base_level = (state.c1_idx < constants::kMaxNumC1Flags) ?
    (2 + (state.c2_idx < constants::kMaxNumC2Flags)) : 1;
  const uint32_t threshold =
    !Restrictions::Get().disable_ext_cabac_alt_residual_ctx ?
    TransformHelper::kGolombRiceRangeExt[state.golomb_rice_k] :
    constants::kCoeffRemainBinReduction;
  Bits bits_sum = 0;
  bits_sum += ContextModel::kEntropyBypassBits;   // sign bypass coded

  if (quant_level >= base_level) {
    uint32_t code_number = quant_level - base_level;
    if (code_number < (threshold << state.golomb_rice_k)) {
      int length = code_number >> state.golomb_rice_k;
      bits_sum +=
        (length + 1 + state.golomb_rice_k) * ContextModel::kEntropyBypassBits;
    } else {
      int length = state.golomb_rice_k;
      code_number = code_number - (threshold << state.golomb_rice_k);
      while (code_number >= (1u << length)) {
        code_number -= (1 << (length++));
      }
      int num_bins = length + threshold + length + 1 - state.golomb_rice_k;
      bits_sum += num_bins * ContextModel::kEntropyBypassBits;
    }
    if (state.c1_idx < constants::kMaxNumC1Flags) {
      bits_sum += c1_ctx.GetEntropyBits(1);
      if (state.c2_idx < constants::kMaxNumC2Flags) {
        bits_sum += c2_ctx.GetEntropyBits(1);
      }
    }
  } else if (quant_level == 1) {
    bits_sum += c1_ctx.GetEntropyBits(0);
  } else if (quant_level == 2) {
    bits_sum += c1_ctx.GetEntropyBits(1);
    bits_sum += c2_ctx.GetEntropyBits(0);
  } else {
    return 0;
  }
  return bits_sum;
}

void RdoQuant::UpdateCodeState(YuvComponent comp, Coeff quant_level,
                               CoeffCodingState *state) const {
  const Coeff base_level = (state->c1_idx < constants::kMaxNumC1Flags) ?
    (2 + (state->c2_idx < constants::kMaxNumC2Flags)) : 1;
  if (quant_level >= 1) {
    state->c1_idx++;
  }
  if (quant_level >= 2) {
    state->c2_idx++;
    state->c1 = 0;
  } else if (quant_level >= 1 && (state->c1 < 3) && (state->c1 > 0)) {
    state->c1++;
  }
  if (quant_level >= base_level) {
    if (quant_level > 3 * (1 << state->golomb_rice_k)) {
      state->golomb_rice_k = std::min<int>(state->golomb_rice_k + 1, 4);
    }
  }
}

Bits RdoQuant::GetLastPosBits(int width, int height, YuvComponent comp,
                              ScanOrder scan_order, Contexts *contexts,
                              int last_pos_x, int last_pos_y) const {
  // TODO(PH) Replace with call to SyntaxWriter::WriteCoeffLastPos?
  Bits bits = 0;
  if (scan_order == ScanOrder::kVertical) {
    std::swap(last_pos_x, last_pos_y);
    std::swap(width, height);
  }
  const int group_idx_x = TransformHelper::kLastPosGroupIdx[last_pos_x];
  const int group_idx_y = TransformHelper::kLastPosGroupIdx[last_pos_y];
  // pos X
  int ctx_last_x;
  for (ctx_last_x = 0; ctx_last_x < group_idx_x; ctx_last_x++) {
    ContextModel &ctx =
      contexts->GetCoeffLastPosCtx(comp, width, height, ctx_last_x, true);
    bits += ctx.GetEntropyBits(1);
  }
  if (group_idx_x < TransformHelper::kLastPosGroupIdx[width - 1]) {
    ContextModel &ctx =
      contexts->GetCoeffLastPosCtx(comp, width, height, ctx_last_x, true);
    bits += ctx.GetEntropyBits(0);
  }
  // pos Y
  int ctx_last_y;
  for (ctx_last_y = 0; ctx_last_y < group_idx_y; ctx_last_y++) {
    ContextModel &ctx =
      contexts->GetCoeffLastPosCtx(comp, width, height, ctx_last_y, false);
    bits += ctx.GetEntropyBits(1);
  }
  if (group_idx_y < TransformHelper::kLastPosGroupIdx[height - 1]) {
    ContextModel &ctx =
      contexts->GetCoeffLastPosCtx(comp, width, height, ctx_last_y, false);
    bits += ctx.GetEntropyBits(0);
  }
  if (group_idx_x > 3) {
    int length = (group_idx_x - 2) >> 1;
    bits += length * ContextModel::kEntropyBypassBits;
  }
  if (group_idx_y > 3) {
    int length = (group_idx_y - 2) >> 1;
    bits += length * ContextModel::kEntropyBypassBits;
  }
  return bits;
}

std::function<Coeff(Coeff)>
RdoQuant::GetFwdQuantFunc(YuvComponent comp, const Qp &qp, int width,
                          int height) {
  const bool size_rounding_bias =
    (util::SizeToLog2(width) + util::SizeToLog2(height)) % 2 != 0;
  const int transform_shift =
    Quantize::GetTransformShift(width, height, bitdepth_);
  const int shift = Quantize::kQuantShift + qp.GetQpPer(comp) +
    transform_shift + (size_rounding_bias ? 7 : 0);
  const int scale = qp.GetFwdScale(comp) * (size_rounding_bias ? 181 : 1);
  const int64_t offset = 1ull << (shift - 1);
  return [scale, offset, shift](const Coeff abs_coeff) {
    int level = static_cast<int>(
      ((static_cast<int64_t>(abs_coeff) * scale) + offset) >> shift);
    return static_cast<Coeff>(level);
  };
}

std::function<Coeff(Coeff)>
RdoQuant::GetInvQuantFunc(YuvComponent comp, const Qp &qp, int width,
                          int height) {
  const bool size_rounding_bias =
    (util::SizeToLog2(width) + util::SizeToLog2(height)) % 2 != 0;
  const int transform_shift =
    Quantize::GetTransformShift(width, height, bitdepth_);
  const int shift = Quantize::kIQuantShift - transform_shift +
    (size_rounding_bias ? 8 : 0);
  const int scale = qp.GetInvScale(comp) * (size_rounding_bias ? 181 : 1);
  const int offset = (1 << (shift - 1));
  return [scale, shift, offset](Coeff in) {
    if (shift > 0) {
      int coeff = ((in * scale) + offset) >> shift;
      return util::Clip3(coeff, constants::kInt16Min, constants::kInt16Max);
    } else {
      int coeff = ((in * scale)) << -shift;
      return util::Clip3(coeff, constants::kInt16Min, constants::kInt16Max);
    }
  };
}

}   // namespace xvc
