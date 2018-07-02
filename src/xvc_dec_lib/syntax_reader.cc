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

#include "xvc_dec_lib/syntax_reader.h"

#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>

#include "xvc_common_lib/intra_prediction.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

template<typename Ctx>
SyntaxReaderCabac<Ctx>::SyntaxReaderCabac(const Qp &qp,
                                          PicturePredictionType pic_type,
                                          BitReader *bit_reader)
  : decoder_(bit_reader) {
  ctx_.ResetStates(qp, pic_type);
  decoder_.Start();
}

template<typename Ctx>
bool SyntaxReaderCabac<Ctx>::Finish() {
  if (!decoder_.DecodeBinTrm()) {
    return false;
  }
  decoder_.Finish();
  return true;
}

template<typename Ctx>
bool SyntaxReaderCabac<Ctx>::ReadAffineFlag(const CodingUnit &cu,
                                            bool is_merge) {
  if (Restrictions::Get().disable_ext2_inter_affine ||
    (is_merge && Restrictions::Get().disable_ext2_inter_affine_merge)) {
    return false;
  }
  Ctx &ctx = ctx_.GetAffineCtx(cu);
  return decoder_.DecodeBin(&ctx) != 0;
}

template<typename Ctx>
bool SyntaxReaderCabac<Ctx>::ReadCbf(const CodingUnit &cu, YuvComponent comp) {
  if (Restrictions::Get().disable_transform_cbf) {
    return true;
  }
  if (util::IsLuma(comp)) {
    return decoder_.DecodeBin(&ctx_.cu_cbf_luma[0]) != 0;
  } else {
    return decoder_.DecodeBin(&ctx_.cu_cbf_chroma[0]) != 0;
  }
}

template<typename Ctx>
int
SyntaxReaderCabac<Ctx>::ReadCoefficients(const CodingUnit &cu,
                                         YuvComponent comp, Coeff *dst_coeff,
                                         ptrdiff_t dst_stride) {
  if (cu.GetWidth(comp) == 2 || cu.GetHeight(comp) == 2) {
    return ReadCoeffSubblock<1>(cu, comp, dst_coeff, dst_stride);
  } else {
    return ReadCoeffSubblock<constants::kSubblockShift>(cu, comp,
                                                        dst_coeff, dst_stride);
  }
}

template<typename Ctx>
template<int SubBlockShift>
int
SyntaxReaderCabac<Ctx>::ReadCoeffSubblock(const CodingUnit &cu,
                                          YuvComponent comp, Coeff *dst_coeff,
                                          ptrdiff_t dst_stride) {
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const int width_log2 = util::SizeToLog2(width);
  const int height_log2 = util::SizeToLog2(height);
  const int log2size = util::SizeToLog2(width);
  constexpr int subblock_shift = SubBlockShift;
  constexpr int subblock_mask = (1 << subblock_shift) - 1;
  constexpr int subblock_size = 1 << (subblock_shift * 2);

  int subblock_width = width >> subblock_shift;
  int subblock_height = height >> subblock_shift;
  int nbr_subblocks = subblock_width * subblock_height;
  std::vector<uint8_t> subblock_csbf(nbr_subblocks);
  std::vector<uint16_t> scan_subblock_table(nbr_subblocks);
  ScanOrder scan_order = TransformHelper::DetermineScanOrder(cu, comp);
  TransformHelper::DeriveSubblockScan(scan_order, subblock_width,
                                      subblock_height, &scan_subblock_table[0]);
  const uint8_t *scan_table = SubBlockShift == 1 ?
    TransformHelper::GetCoeffScanTable2x2(scan_order) :
    TransformHelper::GetCoeffScanTable4x4(scan_order);

  // decode last pos x/y
  int subblock_last_index = subblock_width * subblock_height - 1;
  int subblock_last_coeff_offset = 1;
  int coeff_num_non_zero = 0;
  int total_num_sig_coeff = 0;
  std::array<Coeff, subblock_size> subblock_coeff;
  std::array<uint16_t, subblock_size> subblock_pos;
  subblock_pos[0] = static_cast<uint16_t>(-1);

  int last_nonzero_pos = -1;
  int first_nonzero_pos = subblock_size;
  if (!Restrictions::Get().disable_transform_last_position) {
    uint32_t pos_last_x, pos_last_y;
    ReadCoeffLastPos(width, height, comp, scan_order, &pos_last_x, &pos_last_y);
    const int pos_last_index =
      DetermineLastIndex<SubBlockShift>(subblock_width, subblock_height,
                                        pos_last_x, pos_last_y,
                                        &scan_subblock_table[0], scan_table);
    const int pos_last = (pos_last_y << log2size) + pos_last_x;

    subblock_last_index =
      pos_last_index >> (subblock_shift + subblock_shift);

    // Special handling of last sig coeff (implicitly signaled)
    subblock_last_coeff_offset =
      ((subblock_last_index + 1) << (subblock_shift + subblock_shift)) -
      pos_last_index + 1;
    if (Restrictions::Get().disable_transform_cbf &&
        Restrictions::Get().disable_transform_subblock_csbf &&
        pos_last_x == 0 && pos_last_y == 0) {
      subblock_last_coeff_offset--;
    } else {
      subblock_coeff[0] = 1;
      coeff_num_non_zero = 1;
      dst_coeff[pos_last_y * dst_stride + pos_last_x] = 1;
    }
    subblock_pos[0] = static_cast<uint16_t>(pos_last);
    int subblock_last_offset = subblock_last_index << (subblock_shift * 2);
    last_nonzero_pos = pos_last_index - subblock_last_offset;
    first_nonzero_pos = pos_last_index - subblock_last_offset;
  }

  int c1 = 1;

  // foreach subblock
  for (int subblock_index = subblock_last_index; subblock_index >= 0;
       subblock_index--) {
    int subblock_scan = scan_subblock_table[subblock_index];
    int subblock_scan_y = subblock_scan / subblock_width;
    int subblock_scan_x = subblock_scan - (subblock_scan_y * subblock_width);
    int subblock_pos_x = subblock_scan_x << subblock_shift;
    int subblock_pos_y = subblock_scan_y << subblock_shift;

    int pattern_sig_ctx = 0;
    bool is_last_subblock = subblock_index == subblock_last_index &&
      !Restrictions::Get().disable_transform_last_position &&
      !Restrictions::Get().disable_transform_cbf;
    bool is_first_subblock = subblock_index == 0 &&
      !Restrictions::Get().disable_transform_cbf;
    if (is_last_subblock || is_first_subblock ||
        Restrictions::Get().disable_transform_subblock_csbf) {
      subblock_csbf[subblock_scan] = 1;
      // derive pattern_sig_ctx
      ctx_.GetSubblockCsbfCtx(comp, &subblock_csbf[0], subblock_scan_x,
                              subblock_scan_y, subblock_width, subblock_height,
                              &pattern_sig_ctx);
    } else {
      Ctx &ctx =
        ctx_.GetSubblockCsbfCtx(comp, &subblock_csbf[0], subblock_scan_x,
                                subblock_scan_y, subblock_width,
                                subblock_height, &pattern_sig_ctx);
      subblock_csbf[subblock_scan] =
        static_cast<uint8_t>(decoder_.DecodeBin(&ctx));
    }
    if (!subblock_csbf[subblock_scan]) {
      continue;
    }

    // sig flags
    for (int coeff_index = subblock_size - subblock_last_coeff_offset;
         coeff_index >= 0; coeff_index--) {
      const int scan_offset = scan_table[coeff_index];
      const int coeff_scan_x = subblock_pos_x + (scan_offset & subblock_mask);
      const int coeff_scan_y = subblock_pos_y + (scan_offset >> subblock_shift);
      bool sig_coeff;
      bool not_first_subblock = subblock_index > 0 &&
        !Restrictions::Get().disable_transform_subblock_csbf;
      if (coeff_index == 0 && not_first_subblock && coeff_num_non_zero == 0) {
        sig_coeff = true;
      } else {
        Ctx &ctx =
          ctx_.GetCoeffSigCtx(comp, pattern_sig_ctx, scan_order,
                              coeff_scan_x, coeff_scan_y, dst_coeff, dst_stride,
                              width_log2, height_log2);
        sig_coeff = decoder_.DecodeBin(&ctx) != 0;
      }
      if (sig_coeff) {
        subblock_coeff[coeff_num_non_zero] = 1;
        subblock_pos[coeff_num_non_zero] =
          static_cast<uint16_t>((coeff_scan_y << log2size) + coeff_scan_x);
        coeff_num_non_zero++;
        dst_coeff[coeff_scan_y * dst_stride + coeff_scan_x] = 1;
        if (last_nonzero_pos == -1) {
          last_nonzero_pos = coeff_index;
        }
        first_nonzero_pos = coeff_index;
      } else {
        dst_coeff[coeff_scan_y * dst_stride + coeff_scan_x] = 0;
      }
    }
    subblock_last_coeff_offset = 1;
    if (!coeff_num_non_zero) {
      continue;
    }

    int ctx_set = (subblock_index > 0 && util::IsLuma(comp)) ? 2 : 0;
    if (c1 == 0) {
      ctx_set++;
    }
    c1 = 1;
    int first_c2_idx = -1;

    // greater than 1 flag
    int max_num_c1_flags = constants::kMaxNumC1Flags;
    if (Restrictions::Get().disable_transform_residual_greater_than_flags) {
      max_num_c1_flags = 0;
    }
    for (int i = 0; i < coeff_num_non_zero; i++) {
      if (i == max_num_c1_flags) {
        break;
      }
      const int coeff_scan_y = subblock_pos[i] >> log2size;
      const int coeff_scan_x = subblock_pos[i] - (coeff_scan_y << log2size);
      Ctx &ctx =
        ctx_.GetCoeffGreater1Ctx(comp, ctx_set, c1, coeff_scan_x, coeff_scan_y,
                                 i == 0 && is_last_subblock,
                                 dst_coeff, dst_stride, width, height);
      uint32_t greater_than_1 = decoder_.DecodeBin(&ctx);
      if (greater_than_1) {
        c1 = 0;
        if (first_c2_idx == -1 &&
            !Restrictions::Get().disable_transform_residual_greater2) {
          first_c2_idx = i;
        }
        subblock_coeff[i] = 2;
        dst_coeff[coeff_scan_y * dst_stride + coeff_scan_x] = 2;
      } else if (c1 < 3 && c1 > 0) {
        c1++;
      }
    }

    // greater than 2 flag (max 1)
    if (first_c2_idx >= 0) {
      const int coeff_scan_y = subblock_pos[first_c2_idx] >> log2size;
      const int coeff_scan_x = subblock_pos[first_c2_idx] -
        (coeff_scan_y << log2size);
      Ctx &ctx =
        ctx_.GetCoeffGreater2Ctx(comp, ctx_set, coeff_scan_x, coeff_scan_y,
                                 first_c2_idx == 0 && is_last_subblock,
                                 dst_coeff, dst_stride, width, height);
      Coeff abs_lvl = static_cast<Coeff>(decoder_.DecodeBin(&ctx));
      subblock_coeff[first_c2_idx] += abs_lvl;
      dst_coeff[coeff_scan_y * dst_stride + coeff_scan_x] += abs_lvl;
    }

    // sign hiding
    bool sign_hidden = false;
    if (!Restrictions::Get().disable_transform_sign_hiding &&
        last_nonzero_pos - first_nonzero_pos > constants::SignHidingThreshold) {
      sign_hidden = true;
    }
    last_nonzero_pos = -1;
    first_nonzero_pos = subblock_size;

    // sign flags
    uint32_t coeff_signs;
    if (sign_hidden) {
      coeff_signs = decoder_.DecodeBypassBins(coeff_num_non_zero - 1);
      coeff_signs <<= 32 - (coeff_num_non_zero - 1);
    } else {
      coeff_signs = decoder_.DecodeBypassBins(coeff_num_non_zero);
      coeff_signs <<= 32 - coeff_num_non_zero;
    }

    // abs level remaining
    if (c1 == 0 || coeff_num_non_zero > max_num_c1_flags) {
      int first_coeff_greater2 =
        Restrictions::Get().disable_transform_residual_greater2 ? 0 : 1;
      uint32_t golomb_rice_k = 0;
      for (int i = 0; i < coeff_num_non_zero; i++) {
        const int coeff_scan_y = subblock_pos[i] >> log2size;
        const int coeff_scan_x = subblock_pos[i] - (coeff_scan_y << log2size);
        Coeff base_level = static_cast<Coeff>(
          (i < max_num_c1_flags) ? (2 + first_coeff_greater2) : 1);
        if (subblock_coeff[i] == base_level) {
          if (!Restrictions::Get().disable_ext2_cabac_alt_residual_ctx) {
            golomb_rice_k =
              ctx_.GetCoeffGolombRiceK(coeff_scan_x, coeff_scan_y, width,
                                       height, dst_coeff, dst_stride);
          }
          Coeff abs_lvl =
            static_cast<Coeff>(ReadCoeffRemainExpGolomb(golomb_rice_k));
          subblock_coeff[i] += abs_lvl;
          dst_coeff[coeff_scan_y * dst_stride + coeff_scan_x] += abs_lvl;
          if (subblock_coeff[i] > 3 * (1 << golomb_rice_k) &&
              !Restrictions::Get().disable_transform_adaptive_exp_golomb) {
            golomb_rice_k = std::min(golomb_rice_k + 1, 4u);
          }
        }
        if (subblock_coeff[i] >= 2) {
          first_coeff_greater2 = 0;
        }
      }
    }

    int abs_sum = 0;
    // store calculated coefficients
    for (int i = 0; i < coeff_num_non_zero; i++) {
      const int coeff_scan_y = subblock_pos[i] >> log2size;
      const int coeff_scan_x = subblock_pos[i] - (coeff_scan_y << log2size);
      Coeff coeff = subblock_coeff[i];
      abs_sum += coeff;
      if (i == coeff_num_non_zero - 1 && sign_hidden) {
        int sign = (abs_sum & 0x1) ? -1 : 1;
        dst_coeff[coeff_scan_y * dst_stride + coeff_scan_x] =
          static_cast<Coeff>(sign * coeff);
      } else {
        int sign = static_cast<int32_t>(coeff_signs) >> 31;
        dst_coeff[coeff_scan_y * dst_stride + coeff_scan_x] =
          static_cast<Coeff>((coeff ^ sign) - sign);
        coeff_signs <<= 1;
      }
    }
    total_num_sig_coeff += coeff_num_non_zero;
    coeff_num_non_zero = 0;
  }
  if (!total_num_sig_coeff && subblock_pos[0] != static_cast<uint16_t>(-1)) {
    // Cleanup assumed implicit signaled last sig coeff
    const int coeff_scan_y = subblock_pos[0] >> log2size;
    const int coeff_scan_x = subblock_pos[0] - (coeff_scan_y << log2size);
    dst_coeff[coeff_scan_y * dst_stride + coeff_scan_x] = 0;
  }
  return total_num_sig_coeff;
}

template<typename Ctx>
bool SyntaxReaderCabac<Ctx>::ReadEndOfSlice() {
  uint32_t bin = decoder_.DecodeBinTrm();
  return bin != 0;
}

template<typename Ctx>
InterDir SyntaxReaderCabac<Ctx>::ReadInterDir(const CodingUnit &cu) {
  assert(cu.GetPartitionType() == PartitionType::kSize2Nx2N);
  Ctx &ctx = ctx_.GetInterDirBiCtx(cu);
  if (decoder_.DecodeBin(&ctx) != 0) {
    return InterDir::kBi;
  }
  uint32_t bin = decoder_.DecodeBin(&ctx_.inter_dir[4]);
  return bin == 0 ? InterDir::kL0 : InterDir::kL1;
}

template<typename Ctx>
bool SyntaxReaderCabac<Ctx>::ReadInterFullpelMvFlag(const CodingUnit &cu) {
  if (Restrictions::Get().disable_ext2_inter_adaptive_fullpel_mv) {
    return false;
  }
  Ctx &ctx = ctx_.GetInterFullpelMvCtx(cu);
  return decoder_.DecodeBin(&ctx) != 0;
}

template<typename Ctx>
MvDelta SyntaxReaderCabac<Ctx>::ReadInterMvd() {
  if (Restrictions::Get().disable_inter_mvd_greater_than_flags) {
    MvDelta mvd;
    mvd.x += ReadExpGolomb(1);
    if (mvd.x) {
      uint32_t sign = decoder_.DecodeBypass();
      mvd.x = sign ? -mvd.x : mvd.x;
    }
    mvd.y += ReadExpGolomb(1);
    if (mvd.y) {
      uint32_t sign = decoder_.DecodeBypass();
      mvd.y = sign ? -mvd.y : mvd.y;
    }
    return mvd;
  }
  uint32_t non_zero_x = decoder_.DecodeBin(&ctx_.inter_mvd[0]);
  uint32_t non_zero_y = decoder_.DecodeBin(&ctx_.inter_mvd[0]);
  MvDelta mvd;
  if (non_zero_x) {
    mvd.x = 1 + decoder_.DecodeBin(&ctx_.inter_mvd[1]);
  }
  if (non_zero_y) {
    mvd.y = 1 + decoder_.DecodeBin(&ctx_.inter_mvd[1]);
  }
  if (mvd.x) {
    if (mvd.x > 1) {
      mvd.x += ReadExpGolomb(1);
    }
    uint32_t sign = decoder_.DecodeBypass();
    mvd.x = sign ? -mvd.x : mvd.x;
  }
  if (mvd.y) {
    if (mvd.y > 1) {
      mvd.y += ReadExpGolomb(1);
    }
    uint32_t sign = decoder_.DecodeBypass();
    mvd.y = sign ? -mvd.y : mvd.y;
  }
  return mvd;
}

template<typename Ctx>
int SyntaxReaderCabac<Ctx>::ReadInterMvpIdx(const CodingUnit &cu) {
  if ((!cu.GetUseAffine() && Restrictions::Get().disable_inter_mvp) ||
    (cu.GetUseAffine() && Restrictions::Get().disable_ext2_inter_affine_mvp)) {
    return 0;
  }
  return ReadUnaryMaxSymbol(constants::kNumInterMvPredictors - 1,
                            &ctx_.inter_mvp_idx[0], &ctx_.inter_mvp_idx[0]);
}

template<typename Ctx>
int SyntaxReaderCabac<Ctx>::ReadInterRefIdx(int num_refs_available) {
  if (num_refs_available == 1) {
    return 0;
  }
  int ref_idx = decoder_.DecodeBin(&ctx_.inter_ref_idx[0]);
  if (!ref_idx || num_refs_available == 2) {
    return ref_idx;
  }
  ref_idx += decoder_.DecodeBin(&ctx_.inter_ref_idx[1]);
  if (ref_idx == 1) {
    return ref_idx;
  }
  for (ref_idx = 1; ref_idx < num_refs_available - 2; ref_idx++) {
    uint32_t bin = decoder_.DecodeBypass();
    if (!bin) {
      break;
    }
  }
  return ref_idx + 1;
}

template<typename Ctx>
IntraMode SyntaxReaderCabac<Ctx>::ReadIntraMode(const IntraPredictorLuma &mpm) {
  Ctx &ctx = ctx_.intra_pred_luma[0];
  uint32_t is_mpm_coded = decoder_.DecodeBin(&ctx);
  if (is_mpm_coded) {
    if (!Restrictions::Get().disable_ext2_intra_6_predictors) {
      int mpm_index =
        decoder_.DecodeBin(&ctx_.GetIntraPredictorCtx(mpm[0]));
      if (mpm_index > 0) {
        mpm_index +=
          decoder_.DecodeBin(&ctx_.GetIntraPredictorCtx(mpm[1]));
        if (mpm_index > 1) {
          mpm_index +=
            decoder_.DecodeBin(&ctx_.GetIntraPredictorCtx(mpm[2]));
          if (mpm_index > 2) {
            mpm_index += decoder_.DecodeBypass();
            if (mpm_index > 3) {
              mpm_index += decoder_.DecodeBypass();
            }
          }
        }
      }
      return mpm[mpm_index];
    } else {
      int mpm_index = decoder_.DecodeBypass();
      if (mpm_index) {
        mpm_index += decoder_.DecodeBypass();
      }
      return mpm[mpm_index];
    }
  } else {
    if (!Restrictions::Get().disable_ext2_intra_6_predictors) {
      int intra_mode;
      if (!Restrictions::Get().disable_ext2_intra_67_modes) {
        intra_mode = decoder_.DecodeBypassBins(4);
        intra_mode <<= 2;
        if (intra_mode <= kNbrIntraModesExt - 8) {
          intra_mode += decoder_.DecodeBypassBins(2);
        }
      } else {
        intra_mode = decoder_.DecodeBypassBins(5);
      }
      IntraPredictorLuma mpm_sorted = mpm;
      std::sort(mpm_sorted.begin(),
                mpm_sorted.begin() + constants::kNumIntraMpmExt);
      for (int i = 0; i < static_cast<int>(constants::kNumIntraMpmExt); i++) {
        intra_mode += intra_mode >= mpm_sorted[i] ? 1 : 0;
      }
      return static_cast<IntraMode>(intra_mode);
    } else {
      int intra_mode;
      if (!Restrictions::Get().disable_ext2_intra_67_modes) {
        intra_mode = decoder_.DecodeBypassBins(6);
      } else {
        intra_mode = decoder_.DecodeBypassBins(5);
      }
      IntraPredictorLuma mpm_sorted = mpm;
      if (mpm_sorted[0] > mpm_sorted[1]) {
        std::swap(mpm_sorted[0], mpm_sorted[1]);
      }
      if (mpm_sorted[0] > mpm_sorted[2]) {
        std::swap(mpm_sorted[0], mpm_sorted[2]);
      }
      if (mpm_sorted[1] > mpm_sorted[2]) {
        std::swap(mpm_sorted[1], mpm_sorted[2]);
      }
      for (int i = 0; i < static_cast<int>(constants::kNumIntraMpm); i++) {
        intra_mode += intra_mode >= mpm_sorted[i] ? 1 : 0;
      }
      return static_cast<IntraMode>(intra_mode);
    }
  }
}

template<typename Ctx>
IntraChromaMode
SyntaxReaderCabac<Ctx>::ReadIntraChromaMode(IntraPredictorChroma chroma_preds) {
  uint32_t not_dm_chroma = decoder_.DecodeBin(&ctx_.intra_pred_chroma[0]);
  if (!not_dm_chroma) {
    return IntraChromaMode::kDmChroma;
  }
  if (!Restrictions::Get().disable_ext2_intra_chroma_from_luma) {
    uint32_t not_lm_chroma = decoder_.DecodeBin(&ctx_.intra_pred_chroma[1]);
    if (!not_lm_chroma) {
      return IntraChromaMode::kLmChroma;
    }
  }
  uint32_t chroma_index = decoder_.DecodeBypassBins(2);
  return chroma_preds[chroma_index];
}

template<typename Ctx>
bool SyntaxReaderCabac<Ctx>::ReadLicFlag() {
  if (Restrictions::Get().disable_ext2_inter_local_illumination_comp) {
    return false;
  }
  return decoder_.DecodeBin(&ctx_.lic_flag[0]) != 0;
}

template<typename Ctx>
bool SyntaxReaderCabac<Ctx>::ReadMergeFlag() {
  if (Restrictions::Get().disable_inter_merge_mode) {
    return false;
  }
  uint32_t bin = decoder_.DecodeBin(&ctx_.inter_merge_flag[0]);
  return bin != 0;
}

template<typename Ctx>
int SyntaxReaderCabac<Ctx>::ReadMergeIdx() {
  if (Restrictions::Get().disable_inter_merge_candidates) {
    return 0;
  }
  const int max_merge_cand = constants::kNumInterMergeCandidates;
  uint32_t merge_idx = decoder_.DecodeBin(&ctx_.inter_merge_idx[0]);
  if (merge_idx) {
    while (merge_idx < max_merge_cand - 1 && decoder_.DecodeBypass()) {
      merge_idx++;
    }
  }
  return merge_idx;
}

template<typename Ctx>
PartitionType SyntaxReaderCabac<Ctx>::ReadPartitionType(const CodingUnit &cu) {
  if (cu.GetPredMode() == PredictionMode::kIntra) {
    PartitionType part_type = PartitionType::kSize2Nx2N;
    // Signaling partition type for lowest level assumes single CU tree
    assert(cu.GetCuTree() == CuTree::Primary);
    if (cu.GetDepth() == constants::kMaxCuDepth) {
      uint32_t bin = decoder_.DecodeBin(&ctx_.cu_part_size[0]);
      part_type =
        bin != 0 ? PartitionType::kSize2Nx2N : PartitionType::kSizeNxN;
    }
    return part_type;
  }
  uint32_t bin = decoder_.DecodeBin(&ctx_.cu_part_size[0]);
  if (bin) {
    return PartitionType::kSize2Nx2N;
  }
  // TODO(Dev) Non 2Nx2N part size not implemented
  assert(0);
  return PartitionType::kSizeNxN;
}

template<typename Ctx>
PredictionMode SyntaxReaderCabac<Ctx>::ReadPredMode() {
  uint32_t is_intra = decoder_.DecodeBin(&ctx_.cu_pred_mode[0]);
  return is_intra != 0 ? PredictionMode::kIntra : PredictionMode::kInter;
}

template<typename Ctx>
int SyntaxReaderCabac<Ctx>::ReadQp(int predicted_qp, int base_qp,
                                   int aqp_mode) {
  if (aqp_mode == 1) {
    return decoder_.DecodeBypassBins(7);
  }
  int val = decoder_.DecodeBin(&ctx_.delta_qp[0]);
  int tmp_qp = 0;
  if (val == 1) {
    return predicted_qp;
  }
  val = decoder_.DecodeBypassBins(1);
  if (val == 1) {
    val = decoder_.DecodeBypassBins(1);
    if (val == 0) {
      tmp_qp = predicted_qp + 10;
    } else {
      tmp_qp = predicted_qp + 1;
    }
  } else {
    val = decoder_.DecodeBypassBins(3);
    tmp_qp = predicted_qp + 2 + val;
  }
  if (tmp_qp > base_qp + 7) {
    tmp_qp -= 11;
  } else if (tmp_qp < base_qp - 3) {
    tmp_qp += 11;
  }
  return tmp_qp;
}

template<typename Ctx>
bool SyntaxReaderCabac<Ctx>::ReadRootCbf() {
  if (Restrictions::Get().disable_transform_root_cbf) {
    return true;
  }
  return decoder_.DecodeBin(&ctx_.cu_root_cbf[0]) != 0;
}

template<typename Ctx>
bool SyntaxReaderCabac<Ctx>::ReadSkipFlag(const CodingUnit &cu) {
  if (Restrictions::Get().disable_inter_skip_mode ||
      Restrictions::Get().disable_inter_merge_mode) {
    return false;
  }
  Ctx &ctx = ctx_.GetSkipFlagCtx(cu);
  return decoder_.DecodeBin(&ctx) != 0;
}

template<typename Ctx>
SplitType
SyntaxReaderCabac<Ctx>::ReadSplitBinary(const CodingUnit &cu,
                                        SplitRestriction split_restriction) {
  Ctx &ctx = ctx_.GetSplitBinaryCtx(cu);
  uint32_t bin = decoder_.DecodeBin(&ctx);
  if (!bin) {
    return SplitType::kNone;
  }
  if (cu.GetWidth(YuvComponent::kY) == constants::kMinBinarySplitSize ||
      split_restriction == SplitRestriction::kNoVertical) {
    return SplitType::kHorizontal;
  }
  if (cu.GetHeight(YuvComponent::kY) == constants::kMinBinarySplitSize ||
      split_restriction == SplitRestriction::kNoHorizontal) {
    return SplitType::kVertical;
  }
  int offset =
    cu.GetWidth(YuvComponent::kY) == cu.GetHeight(YuvComponent::kY) ? 0 :
    (cu.GetWidth(YuvComponent::kY) > cu.GetHeight(YuvComponent::kY) ? 1 : 2);
  Ctx &ctx2 = ctx_.cu_split_binary[3 + offset];
  uint32_t bin2 = decoder_.DecodeBin(&ctx2);
  return bin2 != 0 ? SplitType::kVertical : SplitType::kHorizontal;
}

template<typename Ctx>
SplitType SyntaxReaderCabac<Ctx>::ReadSplitQuad(const CodingUnit &cu,
                                                int max_depth) {
  Ctx &ctx = ctx_.GetSplitFlagCtx(cu, max_depth);
  uint32_t bin = decoder_.DecodeBin(&ctx);
  return bin != 0 ? SplitType::kQuad : SplitType::kNone;
}

template<typename Ctx>
bool SyntaxReaderCabac<Ctx>::ReadTransformSkip(const CodingUnit &cu,
                                               YuvComponent comp) {
  if (Restrictions::Get().disable_ext2_transform_skip ||
      !cu.CanTransformSkip(comp)) {
    return false;
  }
  Ctx &ctx = ctx_.transform_skip_flag[util::IsLuma(comp) ? 0 : 1];
  return decoder_.DecodeBin(&ctx) != 0;
}

template<typename Ctx>
bool SyntaxReaderCabac<Ctx>::ReadTransformSelectEnable(const CodingUnit &cu) {
  if (Restrictions::Get().disable_ext2_transform_select) {
    return false;
  }
  Ctx &ctx = ctx_.transform_select_flag[cu.GetDepth()];
  return decoder_.DecodeBin(&ctx) != 0;
}

template<typename Ctx>
int SyntaxReaderCabac<Ctx>::ReadTransformSelectIdx(const CodingUnit &cu) {
  if (Restrictions::Get().disable_ext2_transform_select) {
    return 0;
  }
  static_assert(constants::kMaxTransformSelectIdx == 4, "2 bits signaling");
  Ctx &ctx1 = cu.IsIntra() ?
    ctx_.transform_select_idx[0] : ctx_.transform_select_idx[2];
  Ctx &ctx2 = cu.IsIntra() ?
    ctx_.transform_select_idx[1] : ctx_.transform_select_idx[3];
  int type_idx = 0;
  type_idx += decoder_.DecodeBin(&ctx1) ? 1 : 0;
  type_idx += decoder_.DecodeBin(&ctx2) ? 2 : 0;
  return type_idx;
}

template<typename Ctx>
void SyntaxReaderCabac<Ctx>::ReadCoeffLastPos(int width, int height,
                                              YuvComponent comp,
                                              ScanOrder scan_order,
                                              uint32_t *out_pos_last_x,
                                              uint32_t *out_pos_last_y) {
  if (scan_order == ScanOrder::kVertical) {
    std::swap(width, height);
  }
  uint32_t group_idx_x = TransformHelper::kLastPosGroupIdx[width - 1];
  uint32_t group_idx_y = TransformHelper::kLastPosGroupIdx[height - 1];
  uint32_t pos_last_x = 0;
  for (; pos_last_x < group_idx_x; pos_last_x++) {
    Ctx &ctx = ctx_.GetCoeffLastPosCtx(comp, width, height,
                                       pos_last_x, true);
    uint32_t has_more = decoder_.DecodeBin(&ctx);
    if (!has_more) {
      break;
    }
  }
  uint32_t pos_last_y = 0;
  for (; pos_last_y < group_idx_y; pos_last_y++) {
    Ctx &ctx = ctx_.GetCoeffLastPosCtx(comp, width, height,
                                       pos_last_y, false);
    uint32_t has_more = decoder_.DecodeBin(&ctx);
    if (!has_more) {
      break;
    }
  }
  if (pos_last_x > 3) {
    uint32_t offset = 0;
    uint32_t count = (pos_last_x - 2) >> 1;
    for (int i = count - 1; i >= 0; i--) {
      uint32_t bin = decoder_.DecodeBypass();
      offset += bin << i;
    }
    pos_last_x = TransformHelper::kLastPosMinInGroup[pos_last_x] + offset;
  }
  if (pos_last_y > 3) {
    uint32_t offset = 0;
    uint32_t count = (pos_last_y - 2) >> 1;
    for (int i = count - 1; i >= 0; i--) {
      uint32_t bin = decoder_.DecodeBypass();
      offset += bin << i;
    }
    pos_last_y = TransformHelper::kLastPosMinInGroup[pos_last_y] + offset;
  }
  if (scan_order == ScanOrder::kVertical) {
    std::swap(pos_last_x, pos_last_y);
  }
  *out_pos_last_x = pos_last_x;
  *out_pos_last_y = pos_last_y;
}

template<typename Ctx>
template<int SubBlockShift>
int
SyntaxReaderCabac<Ctx>::DetermineLastIndex(int subblock_width,
                                           int subblock_height,
                                           int pos_last_x, int pos_last_y,
                                           const uint16_t *subblock_scan_table,
                                           const uint8_t *coeff_scan_table) {
  constexpr int subblock_shift = SubBlockShift;
  constexpr int subblock_mask = (1 << subblock_shift) - 1;
  constexpr int subblock_size = 1 << (subblock_shift * 2);
  const int nbr_subblocks = subblock_width * subblock_height;
  for (int subblock_i = 0; subblock_i < nbr_subblocks; subblock_i++) {
    int subblock_scan = subblock_scan_table[subblock_i];
    int subblock_scan_y = subblock_scan / subblock_width;
    int subblock_scan_x = subblock_scan - (subblock_scan_y * subblock_width);
    int subblock_pos_x = subblock_scan_x << subblock_shift;
    int subblock_pos_y = subblock_scan_y << subblock_shift;
    for (int coeff_index = 0; coeff_index < subblock_size; coeff_index++) {
      int scan_offset = coeff_scan_table[coeff_index];
      int coeff_scan_x = subblock_pos_x + (scan_offset & subblock_mask);
      int coeff_scan_y = subblock_pos_y + (scan_offset >> subblock_shift);
      if (coeff_scan_x == pos_last_x && coeff_scan_y == pos_last_y) {
        return (subblock_i << (subblock_shift * 2)) + coeff_index;
      }
    }
  }
  assert(0);
  return 0;
}

template<typename Ctx>
uint32_t
SyntaxReaderCabac<Ctx>::ReadCoeffRemainExpGolomb(uint32_t golomb_rice_k) {
  const uint32_t threshold =
    !Restrictions::Get().disable_ext2_cabac_alt_residual_ctx ?
    TransformHelper::kGolombRiceRangeExt[golomb_rice_k] :
    constants::kCoeffRemainBinReduction;
  uint32_t prefix = 0;
  uint32_t code_word;
  while ((code_word = decoder_.DecodeBypass()) != 0) {
    prefix++;
  }
  if (prefix < threshold) {
    code_word = decoder_.DecodeBypassBins(golomb_rice_k);
    return (prefix << golomb_rice_k) + code_word;
  } else {
    code_word =
      decoder_.DecodeBypassBins(prefix - threshold + golomb_rice_k);
    return code_word +
      (((1 << (prefix - threshold)) + threshold - 1) << golomb_rice_k);
  }
}

template<typename Ctx>
uint32_t SyntaxReaderCabac<Ctx>::ReadExpGolomb(uint32_t golomb_rice_k) {
  uint32_t abs_level = 0;
  uint32_t bin = 1;
  while (bin) {
    bin = decoder_.DecodeBypass();
    abs_level += bin << golomb_rice_k;
    golomb_rice_k++;
  }
  if (--golomb_rice_k) {
    abs_level += decoder_.DecodeBypassBins(golomb_rice_k);
  }
  return abs_level;
}

template<typename Ctx>
uint32_t
SyntaxReaderCabac<Ctx>::ReadUnaryMaxSymbol(uint32_t max_val,
                                           Ctx *ctx_start,
                                           Ctx *ctx_rest) {
  assert(max_val > 0);
  uint32_t symbol = decoder_.DecodeBin(ctx_start);
  if (!symbol || max_val == 1) {
    return symbol;
  }
  symbol = 0;
  uint32_t bin;
  do {
    bin = decoder_.DecodeBin(ctx_rest);
    symbol++;
  } while (bin && symbol < (max_val - 1));
  if (bin && symbol == (max_val - 1)) {
    symbol++;
  }
  return symbol;
}

std::unique_ptr<SyntaxReader>
SyntaxReader::Create(const Qp &qp, PicturePredictionType pic_type,
                     BitReader *bit_reader) {
  if (Restrictions::Get().disable_cabac_ctx_update) {
    return std::unique_ptr<SyntaxReader>(
      new SyntaxReaderCabac<ContextModelStatic>(qp, pic_type, bit_reader));
  }
  return std::unique_ptr<SyntaxReader>(
    new SyntaxReaderCabac<ContextModelDynamic>(qp, pic_type, bit_reader));
}

}   // namespace xvc
