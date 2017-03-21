/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/syntax_writer.h"

#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>

#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

SyntaxWriter::SyntaxWriter(const QP &qp, PicturePredictionType pic_type,
                           EntropyEncoder *entropyenc)
  : entropyenc_(entropyenc) {
  ctx_.ResetStates(qp, pic_type);
}

SyntaxWriter::SyntaxWriter(const CabacContexts &contexts,
                           EntropyEncoder *entropyenc)
  : ctx_(contexts), entropyenc_(entropyenc) {
}

void SyntaxWriter::WriteCbf(const CodingUnit &cu, YuvComponent comp, bool cbf) {
  if (util::IsLuma(comp)) {
    entropyenc_->EncodeBin(cbf ? 1 : 0, &ctx_.cu_cbf_luma[0]);
  } else {
    entropyenc_->EncodeBin(cbf ? 1 : 0, &ctx_.cu_cbf_chroma[0]);
  }
}

void SyntaxWriter::WriteCoefficients(const CodingUnit &cu, YuvComponent comp,
                                     const Coeff *src_coeff,
                                     ptrdiff_t src_coeff_stride) {
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const int log2size = util::SizeToLog2(width);
  const int total_coeff = width*height;
  const int subblock_shift = constants::kSubblockShift;
  const int subblock_size = 1 << (subblock_shift * 2);
  ScanOrder scan_order = TransformHelper::DetermineScanOrder(cu, comp);
  const uint16_t *scan_table = TransformHelper::GetScanTable(width, height,
                                                             scan_order);
  const uint16_t *scan_subblock_table =
    TransformHelper::GetScanTableSubblock(width, height, scan_order);

  int subblock_width = width >> subblock_shift;
  int subblock_height = height >> subblock_shift;
  std::vector<uint8_t> subblock_csbf;
  subblock_csbf.resize(subblock_width * subblock_height);
  if (!Restrictions::Get().disable_transform_cbf) {
    subblock_csbf[0] = 1;
  }

  // Find all significant subblocks and last coeff pos x/y
  int subblock_last_index = subblock_width * subblock_height - 1;
  int subblock_last_coeff_offset = 1;
  uint32_t coeff_signs = 0;
  int coeff_num_non_zero = 0;
  std::array<Coeff, subblock_size> subblock_coeff;

  int pos_last_index = 0;
  for (int i = 0; i < total_coeff; i++) {
    int pos = scan_table[i];
    int pos_y = pos >> log2size;
    int pos_x = pos - (pos_y << log2size);
    if (src_coeff[pos_y * src_coeff_stride + pos_x]) {
      pos_last_index = i;
      int subblock_index = i >> (subblock_shift + subblock_shift);
      int subblock_scan = scan_subblock_table[subblock_index];
      subblock_csbf[subblock_scan] = 1;
    }
  }

  if (!Restrictions::Get().disable_transform_last_position) {
    int pos_last = scan_table[pos_last_index];
    int pos_last_y = pos_last >> log2size;
    int pos_last_x = pos_last - (pos_last_y << log2size);
    WriteCoeffLastPos(width, height, comp, scan_order, pos_last_x, pos_last_y);

    subblock_last_index =
      pos_last_index >> (subblock_shift + subblock_shift);

    // Special handling of last sig coeff(implicitly signaled)
    Coeff last_coeff = src_coeff[pos_last_y * src_coeff_stride + pos_last_x];
    subblock_last_coeff_offset =
      ((subblock_last_index + 1) << (subblock_shift + subblock_shift)) -
      pos_last_index + 1;
    if (Restrictions::Get().disable_transform_cbf &&
        Restrictions::Get().disable_transform_subblock_csbf &&
        pos_last_x == 0 && pos_last_y == 0) {
      subblock_last_coeff_offset--;
    } else {
      coeff_num_non_zero = 1;
      coeff_signs = last_coeff < 0;
    }
    subblock_coeff[0] = static_cast<Coeff>(std::abs(last_coeff));
  }

  int c1 = 1;

  for (int subblock_index = subblock_last_index; subblock_index >= 0;
       subblock_index--) {
    int subblock_scan = scan_subblock_table[subblock_index];
    int subblock_scan_y = subblock_scan / subblock_width;
    int subblock_scan_x = subblock_scan - (subblock_scan_y * subblock_width);
    int subblock_offset = subblock_index << (subblock_shift * 2);

    // Code sig sublock flag
    if (Restrictions::Get().disable_transform_subblock_csbf) {
      subblock_csbf[subblock_scan] = 1;
    }
    bool sig = subblock_csbf[subblock_scan] != 0;
    int pattern_sig_ctx = 0;
    bool is_last_subblock = subblock_index == subblock_last_index &&
      !Restrictions::Get().disable_transform_last_position &&
      !Restrictions::Get().disable_transform_cbf;
    bool is_first_subblock = subblock_index == 0 &&
      !Restrictions::Get().disable_transform_cbf;
    if (is_last_subblock || is_first_subblock ||
        Restrictions::Get().disable_transform_subblock_csbf) {
      // implicitly signaled
      assert(sig || Restrictions::Get().disable_transform_subblock_csbf);
      // derive pattern_sig_ctx
      ctx_.GetSubblockCsbfCtx(comp, &subblock_csbf[0], subblock_scan_x,
                              subblock_scan_y, subblock_width, subblock_height,
                              &pattern_sig_ctx);

    } else {
      ContextModel &ctx = ctx_.GetSubblockCsbfCtx(comp, &subblock_csbf[0],
                                                  subblock_scan_x,
                                                  subblock_scan_y,
                                                  subblock_width,
                                                  subblock_height,
                                                  &pattern_sig_ctx);
      entropyenc_->EncodeBin(sig ? 1 : 0, &ctx);
    }
    if (!sig) {
      continue;
    }

    // sig flag
    for (int coeff_index = subblock_size - subblock_last_coeff_offset;
         coeff_index >= 0; coeff_index--) {
      int coeff_scan = scan_table[subblock_offset + coeff_index];
      int coeff_scan_y = coeff_scan >> log2size;
      int coeff_scan_x = coeff_scan - (coeff_scan_y << log2size);
      Coeff coeff = src_coeff[coeff_scan_y * src_coeff_stride + coeff_scan_x];
      bool not_first_subblock = subblock_index > 0 &&
        !Restrictions::Get().disable_transform_subblock_csbf;
      if (coeff_index == 0 && not_first_subblock && coeff_num_non_zero == 0) {
        // implicitly signaled 1
        assert(coeff != 0);
      } else {
        ContextModel &ctx = ctx_.GetCoeffSigCtx(comp, pattern_sig_ctx,
                                                scan_order, coeff_scan_x,
                                                coeff_scan_y, log2size);
        entropyenc_->EncodeBin(coeff != 0, &ctx);
      }
      if (coeff != 0) {
        subblock_coeff[coeff_num_non_zero++] =
          static_cast<Coeff>(std::abs(coeff));
        coeff_signs = (coeff_signs << 1) + (coeff < 0);
      }
    }
    subblock_last_coeff_offset = 1;
    if (!coeff_num_non_zero) {
      continue;
    }

    // greater than 1 flag
    int max_num_c1_flags = constants::kMaxNumC1Flags;
    if (Restrictions::Get().disable_transform_residual_greater_than_flags) {
      max_num_c1_flags = 0;
    }
    int ctx_set = (subblock_index > 0 && util::IsLuma(comp)) ? 2 : 0;
    if (c1 == 0) {
      ctx_set++;
    }
    c1 = 1;
    int first_c2_idx = -1;
    for (int i = 0; i < coeff_num_non_zero; i++) {
      if (i == max_num_c1_flags) {
        break;
      }
      uint32_t greater_than_1 = subblock_coeff[i] > 1;
      ContextModel &ctx = ctx_.GetCoeffGreaterThan1Ctx(comp, ctx_set, c1);
      entropyenc_->EncodeBin(greater_than_1, &ctx);
      if (greater_than_1) {
        c1 = 0;
        if (first_c2_idx == -1) {
          first_c2_idx = i;
        }
      } else if (c1 < 3 && c1 > 0) {
        c1++;
      }
    }

    // greater than 2 flag (max 1)
    if (first_c2_idx >= 0) {
      uint32_t greater_than_2 = subblock_coeff[first_c2_idx] > 2;
      ContextModel &ctx = ctx_.GetCoeffGreaterThan2Ctx(comp, ctx_set);
      entropyenc_->EncodeBin(greater_than_2, &ctx);
    }

    // sign flags
    entropyenc_->EncodeBypassBins(coeff_signs, coeff_num_non_zero);
    coeff_signs = 0;

    // abs level remaining
    if (c1 == 0 || coeff_num_non_zero > max_num_c1_flags) {
      int first_coeff_greater2 = 1;
      uint32_t golomb_rice_k = 0;
      for (int i = 0; i < coeff_num_non_zero; i++) {
        Coeff base_level = static_cast<Coeff>(
          (i < max_num_c1_flags) ? (2 + first_coeff_greater2) : 1);
        if (subblock_coeff[i] >= base_level) {
          WriteCoeffRemainExpGolomb(subblock_coeff[i] - base_level,
                                    golomb_rice_k);
          if (subblock_coeff[i] > 3 * (1 << golomb_rice_k) &&
              !Restrictions::Get().disable_transform_adaptive_exp_golomb) {
            golomb_rice_k = std::min(golomb_rice_k + 1, (uint32_t)4);
          }
        }
        if (subblock_coeff[i] >= 2) {
          first_coeff_greater2 = 0;
        }
      }
    }
    coeff_num_non_zero = 0;
  }
}

void SyntaxWriter::WriteEndOfSlice(bool end_of_slice) {
  entropyenc_->EncodeBinTrm(end_of_slice ? 1 : 0);
}

void SyntaxWriter::WriteInterDir(const CodingUnit &cu, InterDir inter_dir) {
  assert(cu.GetPartitionType() == PartitionType::kSize2Nx2N);
  ContextModel &ctx = ctx_.inter_dir[cu.GetDepth()];
  entropyenc_->EncodeBin(inter_dir == InterDir::kBi ? 1 : 0, &ctx);
  if (inter_dir != InterDir::kBi) {
    uint32_t bin = inter_dir == InterDir::kL0 ? 0 : 1;
    entropyenc_->EncodeBin(bin, &ctx_.inter_dir[4]);
  }
}

void SyntaxWriter::WriteInterMvd(const MotionVector &mvd) {
  int abs_mvd_x = std::abs(mvd.x);
  int abs_mvd_y = std::abs(mvd.y);
  if (Restrictions::Get().disable_inter_mvd_greater_than_flags) {
    WriteExpGolomb(abs_mvd_x, 1);
    if (abs_mvd_x) {
      entropyenc_->EncodeBypass(mvd.x < 0 ? 1 : 0);
    }
    WriteExpGolomb(abs_mvd_y, 1);
    if (abs_mvd_y) {
      entropyenc_->EncodeBypass(mvd.y < 0 ? 1 : 0);
    }
    return;
  }
  entropyenc_->EncodeBin(mvd.x != 0, &ctx_.inter_mvd[0]);
  entropyenc_->EncodeBin(mvd.y != 0, &ctx_.inter_mvd[0]);
  if (abs_mvd_x) {
    entropyenc_->EncodeBin(abs_mvd_x > 1, &ctx_.inter_mvd[1]);
  }
  if (abs_mvd_y) {
    entropyenc_->EncodeBin(abs_mvd_y > 1, &ctx_.inter_mvd[1]);
  }
  if (abs_mvd_x) {
    if (abs_mvd_x > 1) {
      WriteExpGolomb(abs_mvd_x - 2, 1);
    }
    entropyenc_->EncodeBypass(mvd.x < 0 ? 1 : 0);
  }
  if (abs_mvd_y) {
    if (abs_mvd_y > 1) {
      WriteExpGolomb(abs_mvd_y - 2, 1);
    }
    entropyenc_->EncodeBypass(mvd.y < 0 ? 1 : 0);
  }
}

void SyntaxWriter::WriteInterMvpIdx(int mvp_idx) {
  if (Restrictions::Get().disable_inter_mvp) {
    return;
  }
  WriteUnaryMaxSymbol(mvp_idx, constants::kNumInterMvPredictors - 1,
                      &ctx_.inter_mvp_idx[0], &ctx_.inter_mvp_idx[0]);
}

void SyntaxWriter::WriteInterRefIdx(int ref_idx, int num_refs_available) {
  assert(ref_idx < num_refs_available);
  if (num_refs_available == 1) {
    return;
  }
  entropyenc_->EncodeBin(ref_idx != 0 ? 1 : 0, &ctx_.inter_ref_idx[0]);
  if (!ref_idx || num_refs_available == 2) {
    return;
  }
  ref_idx--;
  entropyenc_->EncodeBin(ref_idx != 0 ? 1 : 0, &ctx_.inter_ref_idx[1]);
  if (!ref_idx) {
    return;
  }
  for (int i = 1; i < num_refs_available - 2; i++) {
    uint32_t bin = (i == ref_idx) ? 0 : 1;
    entropyenc_->EncodeBypass(bin);
    if (!bin) {
      break;
    }
  }
}

void SyntaxWriter::WriteIntraMode(IntraMode intra_mode,
                                  const IntraPredictorLuma &mpm) {
  assert(intra_mode < IntraMode::kTotalNumber);
  // TODO(Dev) NxN support missing
  int mpm_index = -1;
  for (int i = 0; i < static_cast<int>(mpm.size()); i++) {
    if (intra_mode == mpm[i]) {
      mpm_index = i;
    }
  }
  ContextModel &ctx = ctx_.intra_pred_luma[0];
  if (mpm_index >= 0) {
    entropyenc_->EncodeBin(1, &ctx);
    static_assert(constants::kNumIntraMPM == 3, "non-branching invariant");
    int num_bits = 1 + (mpm_index > 0);
    entropyenc_->EncodeBypassBins(mpm_index + (mpm_index > 0), num_bits);
  } else {
    entropyenc_->EncodeBin(0, &ctx);
    std::array<IntraMode, constants::kNumIntraMPM> mpm2 = mpm;
    if (mpm2[0] > mpm2[1]) {
      std::swap(mpm2[0], mpm2[1]);
    }
    if (mpm2[0] > mpm2[2]) {
      std::swap(mpm2[0], mpm2[2]);
    }
    if (mpm2[1] > mpm2[2]) {
      std::swap(mpm2[1], mpm2[2]);
    }
    int mode_index = static_cast<int>(intra_mode);
    for (int i = static_cast<int>(mpm2.size()) - 1; i >= 0; i--) {
      mode_index -= mode_index >= mpm2[i] ? 1 : 0;
    }
    entropyenc_->EncodeBypassBins(mode_index, 5);
  }
}

void SyntaxWriter::WriteIntraChromaMode(IntraChromaMode chroma_mode,
                                        IntraPredictorChroma chroma_preds) {
  if (chroma_mode == IntraChromaMode::kDMChroma) {
    entropyenc_->EncodeBin(0, &ctx_.intra_pred_chroma[0]);
    return;
  }
  int chroma_index = 0;
  for (int i = 1; i < static_cast<int>(chroma_preds.size()) - 1; i++) {
    if (chroma_mode == chroma_preds[i]) {
      chroma_index = i;
    }
  }
  entropyenc_->EncodeBin(1, &ctx_.intra_pred_chroma[0]);
  entropyenc_->EncodeBypassBins(chroma_index, 2);
}

void SyntaxWriter::WriteMergeFlag(bool merge) {
  if (Restrictions::Get().disable_inter_merge_mode) {
    return;
  }
  entropyenc_->EncodeBin(merge ? 1 : 0, &ctx_.inter_merge_flag[0]);
}

void SyntaxWriter::WriteMergeIdx(int merge_idx) {
  if (Restrictions::Get().disable_inter_merge_candidates) {
    return;
  }
  const int max_merge_cand = constants::kNumInterMergeCandidates;
  uint32_t bin = merge_idx != 0;
  entropyenc_->EncodeBin(bin, &ctx_.inter_merge_idx[0]);
  if (merge_idx != 0) {
    uint32_t bins = (1 << merge_idx) - 2;
    bins >>= (merge_idx == max_merge_cand - 1) ? 1 : 0;
    int num_bins = merge_idx - ((merge_idx == max_merge_cand - 1) ? 1 : 0);
    entropyenc_->EncodeBypassBins(bins, num_bins);
  }
}

void SyntaxWriter::WritePartitionType(const CodingUnit &cu,
                                      PartitionType type) {
  if (cu.GetPredMode() == PredictionMode::kIntra) {
    if (cu.GetDepth() == constants::kMaxCuDepth) {
      uint32_t bin = type == PartitionType::kSize2Nx2N ? 1 : 0;
      entropyenc_->EncodeBin(bin, &ctx_.cu_part_size[0]);
    }
    return;
  }
  switch (type) {
    case PartitionType::kSize2Nx2N:
      entropyenc_->EncodeBin(1, &ctx_.cu_part_size[0]);
      break;
    default:
      entropyenc_->EncodeBin(0, &ctx_.cu_part_size[0]);
      // TODO(Dev) Non 2Nx2N part size not implemented
      assert(0);
      break;
  }
}

void SyntaxWriter::WritePredMode(PredictionMode pred_mode) {
  uint32_t is_intra = pred_mode == PredictionMode::kIntra ? 1 : 0;
  entropyenc_->EncodeBin(is_intra, &ctx_.cu_pred_mode[0]);
}

void SyntaxWriter::WriteRootCbf(bool root_cbf) {
  entropyenc_->EncodeBin(root_cbf != 0, &ctx_.cu_root_cbf[0]);
}

void SyntaxWriter::WriteSkipFlag(const CodingUnit &cu, bool flag) {
  if (Restrictions::Get().disable_inter_skip_mode) {
    return;
  }
  int offset = 0;
  if (!Restrictions::Get().disable_cabac_skip_flag_ctx) {
    const CodingUnit *tmp;
    if ((tmp = cu.GetCodingUnitLeft()) != nullptr && tmp->GetSkipFlag()) {
      offset++;
    }
    if ((tmp = cu.GetCodingUnitAbove()) != nullptr && tmp->GetSkipFlag()) {
      offset++;
    }
  }
  entropyenc_->EncodeBin(flag ? 1 : 0, &ctx_.cu_skip_flag[offset]);
}

void SyntaxWriter::WriteSplitFlag(int depth, const CodingUnit *left,
                                  const CodingUnit *above, bool split) {
  int offset = 0;
  if (!Restrictions::Get().disable_cabac_split_flag_ctx) {
    if (left) {
      offset += left->GetDepth() > depth;
    }
    if (above) {
      offset += above->GetDepth() > depth;
    }
  }
  ContextModel &ctx = ctx_.cu_split_flag[offset];
  entropyenc_->EncodeBin(split ? 1 : 0, &ctx);
}

void SyntaxWriter::WriteCoeffLastPos(int width, int height, YuvComponent comp,
                                     ScanOrder scan_order, int last_pos_x,
                                     int last_pos_y) {
  if (scan_order == ScanOrder::kVertical) {
    std::swap(last_pos_x, last_pos_y);
  }
  int group_idx_x = TransformHelper::kLastPosGroupIdx[last_pos_x];
  int group_idx_y = TransformHelper::kLastPosGroupIdx[last_pos_y];
  // pos X
  int ctx_last_x;
  for (ctx_last_x = 0; ctx_last_x < group_idx_x; ctx_last_x++) {
    ContextModel &ctx =
      ctx_.GetCoeffLastPosCtx(comp, width, height, ctx_last_x, true);
    entropyenc_->EncodeBin(1, &ctx);
  }
  if (group_idx_x < TransformHelper::kLastPosGroupIdx[width - 1]) {
    ContextModel &ctx =
      ctx_.GetCoeffLastPosCtx(comp, width, height, ctx_last_x, true);
    entropyenc_->EncodeBin(0, &ctx);
  }
  // pos Y
  int ctx_last_y;
  for (ctx_last_y = 0; ctx_last_y < group_idx_y; ctx_last_y++) {
    ContextModel &ctx =
      ctx_.GetCoeffLastPosCtx(comp, width, height, ctx_last_y, false);
    entropyenc_->EncodeBin(1, &ctx);
  }
  if (group_idx_y < TransformHelper::kLastPosGroupIdx[height - 1]) {
    ContextModel &ctx =
      ctx_.GetCoeffLastPosCtx(comp, width, height, ctx_last_y, false);
    entropyenc_->EncodeBin(0, &ctx);
  }

  if (group_idx_x > 3) {
    int length = (group_idx_x - 2) >> 1;
    uint32_t remain_x =
      last_pos_x - TransformHelper::kLastPosMinInGroup[group_idx_x];
    for (int i = length - 1; i >= 0; i--) {
      entropyenc_->EncodeBypass((remain_x >> i) & 1);
    }
  }
  if (group_idx_y > 3) {
    int length = (group_idx_y - 2) >> 1;
    uint32_t remain_y =
      last_pos_y - TransformHelper::kLastPosMinInGroup[group_idx_y];
    for (int i = length - 1; i >= 0; i--) {
      entropyenc_->EncodeBypass((remain_y >> i) & 1);
    }
  }
}

void SyntaxWriter::WriteCoeffRemainExpGolomb(Coeff code_number,
                                             uint32_t golomb_rice_k) {
  if (code_number < (constants::kCoeffRemainBinReduction << golomb_rice_k)) {
    uint32_t length = code_number >> golomb_rice_k;
    entropyenc_->EncodeBypassBins((1 << (length + 1)) - 2, length + 1);
    entropyenc_->EncodeBypassBins((code_number % (1 << golomb_rice_k)),
                                  golomb_rice_k);
  } else {
    uint32_t length = golomb_rice_k;
    code_number -= (constants::kCoeffRemainBinReduction << golomb_rice_k);
    while (code_number >= (1 << length)) {
      code_number -= (1 << (length++));
    }
    int num_bins =
      constants::kCoeffRemainBinReduction + length + 1 - golomb_rice_k;
    entropyenc_->EncodeBypassBins((1 << num_bins) - 2, num_bins);
    entropyenc_->EncodeBypassBins(code_number, length);
  }
}

void SyntaxWriter::WriteExpGolomb(uint32_t abs_level, uint32_t golomb_rice_k) {
  uint32_t bins = 0;
  int num_bins = 0;
  while (abs_level >= (1u << golomb_rice_k)) {
    bins = bins * 2 + 1;
    num_bins++;
    abs_level -= 1 << golomb_rice_k;
    golomb_rice_k++;
  }
  bins *= 2;
  num_bins++;
  bins = (bins << golomb_rice_k) | abs_level;
  num_bins += golomb_rice_k;
  entropyenc_->EncodeBypassBins(bins, num_bins);
}

void SyntaxWriter::WriteUnaryMaxSymbol(uint32_t symbol, uint32_t max_val,
                                       ContextModel *ctx_start,
                                       ContextModel *ctx_rest) {
  assert(max_val > 0);
  entropyenc_->EncodeBin(symbol > 0, ctx_start);
  if (!symbol || max_val == 1) {
    return;
  }
  bool not_max = symbol < max_val;
  while (--symbol) {
    entropyenc_->EncodeBin(1, ctx_rest);
  }
  if (not_max) {
    entropyenc_->EncodeBin(0, ctx_rest);
  }
}

RdoSyntaxWriter::RdoSyntaxWriter(const SyntaxWriter &writer)
  : SyntaxWriter(writer.GetContexts(), &entropy_instance_),
  entropy_instance_(nullptr, writer.GetNumWrittenBits(),
                    writer.GetFractionalBits()) {
}

RdoSyntaxWriter::RdoSyntaxWriter(const RdoSyntaxWriter &writer)
  : RdoSyntaxWriter(static_cast<const SyntaxWriter&>(writer)) {
}

RdoSyntaxWriter::RdoSyntaxWriter(const SyntaxWriter &writer,
                                 uint32_t bits_written)
  : SyntaxWriter(writer.GetContexts(), &entropy_instance_),
  entropy_instance_(nullptr, bits_written, writer.GetFractionalBits()) {
}

RdoSyntaxWriter& RdoSyntaxWriter::operator=(const RdoSyntaxWriter &writer) {
  ctx_ = writer.ctx_;
  // Assumes a null BitWriter is used
  entropy_instance_ = writer.entropy_instance_;
  return *this;
}

}   // namespace xvc
