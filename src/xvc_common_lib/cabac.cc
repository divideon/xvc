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

#include "xvc_common_lib/cabac.h"

#include <algorithm>
#include <cstdlib>

#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

static const uint8_t kNotUsed = 0;
static const uint8_t kDef = 154;  // TODO(PH) Used but value not determined

static const uint8_t
kInitCuTransquantBypassFlag[3][CabacCommon::kNumTquantBypassFlagCtx] = {
  { 154 },
  { 154 },
  { 154 },
};

static const uint8_t
kInitSplitQuadFlag[3][CabacCommon::kNumSplitQuadFlagCtx] = {
  { 107,  139,  126, 255, 0, },
  { 107,  139,  126, 255, 0, },
  { 139,  141,  157, 255, 0, },
};

static const uint8_t
kInitSplitBinary[3][CabacCommon::kNumSplitBinaryCtx] = {
  { 107,  139,  126, 154, 154, 154 },
  { 107,  139,  126, 154, 154, 154 },
  { 139,  141,  157, 154, 154, 154 },
};

static const uint8_t
kInitSkipFlag[3][CabacCommon::kNumSkipFlagCtx] = {
  { 197,  185,  201, },
  { 197,  185,  201, },
  { kNotUsed,  kNotUsed,  kNotUsed, },
};

static const uint8_t
kInitMergeFlag[3][CabacCommon::kNumMergeFlagCtx] = {
  { 154, },
  { 110, },
  { kNotUsed, },
};

static const uint8_t
kInitMergeIdx[3][CabacCommon::kNumMergeIdxCtx] = {
  { 137, },
  { 122, },
  { kNotUsed, },
};

static const uint8_t
kInitPartSize[3][CabacCommon::kNumPartSizeCtx] = {
  { 154,  139,  154, 154 },
  { 154,  139,  154, 154 },
  { 184,  kNotUsed,  kNotUsed, kNotUsed },
};

static const uint8_t
kInitPredMode[3][CabacCommon::kNumPredModeCtx] = {
  { 134, },
  { 149, },
  { kNotUsed, },
};

static const uint8_t
kInitIntraLumaPredMode[3][CabacCommon::kNumIntraPredCtxLuma] = {
  { 183, kDef, kDef, kDef, kDef, kDef, kDef, kDef, kDef },
  { 154, kDef, kDef, kDef, kDef, kDef, kDef, kDef, kDef },
  { 184, kDef, kDef, kDef, kDef, kDef, kDef, kDef, kDef },
};

static const uint8_t
kInitIntraChromaPredMode[3][CabacCommon::kNumIntraPredCtxChroma] = {
  { 152, 139 },
  { 152, 139 },
  { 63, 139 },
};

static const uint8_t
kInitInterDir[3][CabacCommon::kNumInterDirCtx] = {
  { 95,   79,   63,   31,  31, },
  { 95,   79,   63,   31,  31, },
  { kNotUsed,  kNotUsed,  kNotUsed,  kNotUsed, kNotUsed, },
};

static const uint8_t
kInitInterFullpelMv[3][CabacCommon::kNumInterFullpelMvCtx] = {
  { 197, 185, 201, },
  { 197, 185, 201, },
  { kNotUsed,  kNotUsed,  kNotUsed, },
};

static const uint8_t
kInitAffineFlag[3][CabacCommon::kNumAffineCtx] = {
  { 197, 185, 201, },
  { 197, 185, 201, },
  { kNotUsed, kNotUsed, kNotUsed, },
};

static const uint8_t
kInitLicFlag[3][CabacCommon::kNumLicFlagCtx] = {
  { 154 },
  { 154 },
  { kNotUsed },
};

static const uint8_t
kInitMvd[3][CabacCommon::kNumMvdCtx] = {
  { 169,  198, },
  { 140,  198, },
  { kNotUsed,  kNotUsed, },
};

static const uint8_t
kInitRefIdx[3][CabacCommon::kNumRefIdxCtx] = {
  { 153,  153 },
  { 153,  153 },
  { kNotUsed,  kNotUsed },
};

static const uint8_t
kInitDqp[3][CabacCommon::kNumDeltaQpCtx] = {
  { 154,  154,  154, },
  { 154,  154,  154, },
  { 154,  154,  154, },
};

static const uint8_t
kInitCuCbf[3][CabacCommon::kNumCuCbfCtx] = {
  { 111,  149 },
  { 111,  149 },
  { 141,   94 },
};

static const uint8_t
kInitCuRootCbf[3][CabacCommon::kNumCuRootCbfCtx] = {
  { 79, },
  { 79, },
  { kNotUsed, },
};

static const uint8_t
kInitLastPos[3][CabacCommon::kNumCoeffLastPosCtx] = {
  { 125, 110, 124, 110, 95, 94, 125, 111, 111, 79, 125, 126, 111, 111, 79,
  126, 111, 111, 79, kDef, kDef, kDef, kDef, kDef, kDef,
  108, 123, 93 },
  { 125, 110, 94, 110, 95, 79, 125, 111, 110, 78, 110, 111, 111, 95, 94,
  111, 111, 95, 94, kDef, kDef, kDef, kDef, kDef, kDef,
  108, 123, 108 },
  { 110, 110, 124, 125, 140, 153, 125, 127, 140, 109, 111, 143, 127, 111, 79,
  143, 127, 111, 79, kDef, kDef, kDef, kDef, kDef, kDef,
  108, 123, 63 },
};

static const uint8_t
kInitSubblockCsbf[3][CabacCommon::kNumSubblockCsbfCtx] = {
  { 121, 140, 61, 154, },
  { 121, 140, 61, 154, },
  { 91, 171, 134, 141, },
};

static const uint8_t
kInitExtSubblockCsbf[3][CabacCommon::kNumExtSubblockCsbfCtx] = {
  { 122, 143, 91, 141, },
  { 61, 154, 78, 111, },
  { 135, 155, 104, 139, },
};

static const uint8_t
kInitCoeffSig[3][CabacCommon::kNumCoeffSigCtx] = {
  { 170, 154, 139, 153, 139, 123, 123, 63, 124, 166, 183, 140, 136, 153, 154,
  166, 183, 140, 136, 153, 154, 166, 183, 140, 136, 153, 154, 170, 153, 138,
  138, 122, 121, 122, 121, 167, 151, 183, 140, 151, 183, 140, },
  { 155, 154, 139, 153, 139, 123, 123, 63, 153, 166, 183, 140, 136, 153, 154,
  166, 183, 140, 136, 153, 154, 166, 183, 140, 136, 153, 154, 170, 153, 123,
  123, 107, 121, 107, 121, 167, 151, 183, 140, 151, 183, 140, },
  { 111, 111, 125, 110, 110, 94, 124, 108, 124, 107, 125, 141, 179, 153, 125,
  107, 125, 141, 179, 153, 125, 107, 125, 141, 179, 153, 125, 140, 139, 182,
  182, 152, 136, 152, 136, 153, 136, 139, 111, 136, 139, 111, },
};

static const uint8_t
kInitExtCoeffSig[3][CabacCommon::kNumExtCoeffSigCtx] = {
  { 107, 139, 154, 140, 140, 141, 108, 154, 125, 155, 126, 127, 139, 155, 155,
  141, 156, 143, 107, 139, 154, 140, 140, 141, 108, 154, 125, 155, 126, 127,
  139, 155, 155, 141, 156, 143, 107, 139, 154, 140, 140, 141, 108, 154, 125,
  155, 126, 127, 139, 155, 155, 141, 156, 143, 137, 154, 154, 155, 155, 156,
  124, 185, 156, 171, 142, 158, },
  { 121, 167, 153, 139, 154, 140, 137, 168, 139, 154, 169, 155, 167, 169, 169,
  184, 199, 156, 121, 167, 153, 139, 154, 140, 137, 168, 139, 154, 169, 155,
  167, 169, 169, 184, 199, 156, 121, 167, 153, 139, 154, 140, 137, 168, 139,
  154, 169, 155, 167, 169, 169, 184, 199, 156, 136, 153, 139, 154, 125, 140,
  122, 154, 184, 185, 171, 157, },
  { 152, 139, 154, 154, 169, 155, 182, 154, 169, 184, 155, 141, 168, 214, 199,
  170, 170, 171, 152, 139, 154, 154, 169, 155, 182, 154, 169, 184, 155, 141,
  168, 214, 199, 170, 170, 171, 152, 139, 154, 154, 169, 155, 182, 154, 169,
  184, 155, 141, 168, 214, 199, 170, 170, 171, 167, 154, 169, 140, 155, 141,
  153, 171, 185, 156, 171, 172, },
};

static const uint8_t
kInitCoeffGreater1[3][CabacCommon::kNumCoeffGreater1Ctx] = {
  { 154, 196, 167, 167, 154, 152, 167, 182, 182, 134, 149, 136, 153, 121, 136,
  122, 169, 208, 166, 167, 154, 152, 167, 182, },
  { 154, 196, 196, 167, 154, 152, 167, 182, 182, 134, 149, 136, 153, 121, 136,
  137, 169, 194, 166, 167, 154, 167, 137, 182, },
  { 140, 92, 137, 138, 140, 152, 138, 139, 153, 74, 149, 92, 139, 107, 122,
  152, 140, 179, 166, 182, 140, 227, 122, 197, },
};

static const uint8_t
kInitExtCoeffGreater1[3][CabacCommon::kNumExtCoeffGreater1Ctx] = {
  { 121, 135, 123, 124, 139, 125,  92, 124, 154, 125, 155, 138, 169, 155, 170,
  156, 166, 152, 140, 170, 171, 157 },
  { 165,  75, 152, 153, 139, 154, 121, 138, 139, 154, 140, 167, 183, 169, 170,
  156, 193, 181, 169, 170, 171, 172 },
  { 196, 105, 152, 153, 139, 154, 136, 138, 139, 169, 140, 196, 183, 169, 170,
  171, 195, 181, 169, 170, 156, 157 },
};

static const uint8_t
kInitCoeffGreater2[3][CabacCommon::kNumCoeffGreater2Ctx] = {
  { 107, 167, 91, 107, 107, 167, },
  { 107, 167, 91, 122, 107, 167, },
  { 138, 153, 136, 167, 152, 152, },
};

static const uint8_t
kInitMvpIdx[3][CabacCommon::kNumMvpIdxCtx] = {
  { 168 },
  { 168 },
  { kNotUsed },
};

static const uint8_t
kInitSaoMergeFlag[3][CabacCommon::kNumSaoMergeFlagCtx] = {
  { 153, },
  { 153, },
  { 153, },
};

static const uint8_t
InitSaoTypeIdx[3][CabacCommon::kNumSaoTypeIdxCtx] = {
  { 160, },
  { 185, },
  { 200, },
};

static const uint8_t
kInitTransSubdivFlag[3][CabacCommon::kNumTransSubdivFlagCtx] = {
  { 224,  167,  122, },
  { 124,  138,   94, },
  { 153,  138,  138, },
};

static const uint8_t
kInitTransformSkipFlag[3][CabacCommon::kNumTransformSkipFlagCtx] = {
  { 139,  139 },
  { 139,  139 },
  { 139,  139 },
};

static const uint8_t
kInitTransformSelectEnable[3][CabacCommon::kNumTransformSelectEnableCtx] = {
  { kDef, kDef, kDef, kDef, kDef, kDef },
  { kDef, kDef, kDef, kDef, kDef, kDef },
  { kDef, kDef, kDef, kDef, kDef, kDef },
};

static const uint8_t
kInitTransformSelectIdx[3][CabacCommon::kNumTransformSelectIdxCtx] = {
  { kDef, kDef, kDef, kDef },
  { kDef, kDef, kDef, kDef },
  { kDef, kDef, kDef, kDef },
};

template <typename Ctx, size_t N>
inline static void Init(int qp, int slice_type,
                        std::array<Ctx, N> *ctx,
                        const uint8_t init_state[3][N]) {
  for (size_t i = 0; i < N; i++) {
    (*ctx)[i].Init(qp, init_state[slice_type][i]);
  }
}

template <typename Ctx, size_t N, size_t M>
inline static void Init(int qp, int slice_type,
                        std::array<Ctx, N> *ctx_luma,
                        std::array<Ctx, M> *ctx_chroma,
                        const uint8_t init_state[3][N + M]) {
  for (size_t i = 0; i < N; i++) {
    (*ctx_luma)[i].Init(qp, init_state[slice_type][i]);
  }
  for (size_t i = 0; i < M; i++) {
    (*ctx_chroma)[i].Init(qp, init_state[slice_type][N + i]);
  }
}

template<typename Ctx>
void
CabacContexts<Ctx>::ResetStates(const Qp &qp, PicturePredictionType pic_type) {
  int q = qp.GetQpRaw(YuvComponent::kY);
  if (Restrictions::Get().disable_cabac_init_per_qp) {
    q = 32;
  }
  int s = static_cast<int>(pic_type);
  if (Restrictions::Get().disable_cabac_init_per_pic_type) {
    s = static_cast<int>(PicturePredictionType::kBi);
  }
  Init(q, s, &cu_cbf_luma, &cu_cbf_chroma, kInitCuCbf);
  Init(q, s, &cu_part_size, kInitPartSize);
  Init(q, s, &cu_pred_mode, kInitPredMode);
  Init(q, s, &cu_root_cbf, kInitCuRootCbf);
  Init(q, s, &cu_skip_flag, kInitSkipFlag);
  Init(q, s, &cu_split_quad_flag, kInitSplitQuadFlag);
  Init(q, s, &cu_split_binary, kInitSplitBinary);
  Init(q, s, &inter_dir, kInitInterDir);
  Init(q, s, &inter_fullpel_mv, kInitInterFullpelMv);
  Init(q, s, &inter_merge_flag, kInitMergeFlag);
  Init(q, s, &inter_merge_idx, kInitMergeIdx);
  Init(q, s, &inter_mvd, kInitMvd);
  Init(q, s, &inter_mvp_idx, kInitMvpIdx);
  Init(q, s, &inter_ref_idx, kInitRefIdx);
  Init(q, s, &intra_pred_luma, kInitIntraLumaPredMode);
  Init(q, s, &intra_pred_chroma, kInitIntraChromaPredMode);
  Init(q, s, &affine_flag, kInitAffineFlag);
  Init(q, s, &lic_flag, kInitLicFlag);
  Init(q, s, &delta_qp, kInitDqp);
  if (!Restrictions::Get().disable_ext2_cabac_alt_residual_ctx) {
    Init(q, s, &coeff_ext.csbf_luma, &coeff_ext.csbf_chroma,
         kInitExtSubblockCsbf);
    Init(q, s, &coeff_ext.sig_luma, &coeff_ext.sig_chroma, kInitExtCoeffSig);
    Init(q, s, &coeff_ext.greater1_luma, &coeff_ext.greater1_chroma,
         kInitExtCoeffGreater1);
  } else {
    Init(q, s, &coeff.csbf_luma, &coeff.csbf_chroma, kInitSubblockCsbf);
    Init(q, s, &coeff.sig_luma, &coeff.sig_chroma, kInitCoeffSig);
    Init(q, s, &coeff.greater1_luma, &coeff.greater1_chroma,
         kInitCoeffGreater1);
    Init(q, s, &coeff.greater2_luma, &coeff.greater2_chroma,
         kInitCoeffGreater2);
  }
  Init(q, s, &coeff_last_pos_x_luma, &coeff_last_pos_x_chroma, kInitLastPos);
  Init(q, s, &coeff_last_pos_y_luma, &coeff_last_pos_y_chroma, kInitLastPos);
  Init(q, s, &transform_skip_flag, kInitTransformSkipFlag);
  Init(q, s, &transform_select_flag, kInitTransformSelectEnable);
  Init(q, s, &transform_select_idx, kInitTransformSelectIdx);
}

template<typename Ctx>
Ctx& CabacContexts<Ctx>::GetAffineCtx(const CodingUnit &cu) {
  int offset = 0;
  const CodingUnit *tmp;
  if ((tmp = cu.GetCodingUnitLeft()) != nullptr && tmp->GetUseAffine()) {
    offset++;
  }
  if ((tmp = cu.GetCodingUnitAbove()) != nullptr && tmp->GetUseAffine()) {
    offset++;
  }
  return affine_flag[offset];
}

template<typename Ctx>
Ctx& CabacContexts<Ctx>::GetSkipFlagCtx(const CodingUnit &cu) {
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
  return cu_skip_flag[offset];
}

template<typename Ctx>
Ctx& CabacContexts<Ctx>::GetSplitBinaryCtx(const CodingUnit &cu) {
  const CodingUnit *left = cu.GetCodingUnitLeft();
  const CodingUnit *above = cu.GetCodingUnitAbove();
  int depth = (cu.GetDepth() << 1) + cu.GetBinaryDepth();
  int offset = 0;
  if (left) {
    offset +=
      ((left->GetDepth() << 1) + left->GetBinaryDepth()) > depth ? 1 : 0;
  }
  if (above) {
    offset +=
      ((above->GetDepth() << 1) + above->GetBinaryDepth()) > depth ? 1 : 0;
  }
  return cu_split_binary[offset];
}

template<typename Ctx>
Ctx& CabacContexts<Ctx>::GetSplitFlagCtx(const CodingUnit &cu,
                                         int pic_max_depth) {
  int offset = 0;
  const CodingUnit *left = cu.GetCodingUnitLeft();
  const CodingUnit *above = cu.GetCodingUnitAbove();
  if (!Restrictions::Get().disable_cabac_split_flag_ctx) {
    if (left) {
      offset += left->GetDepth() > cu.GetDepth();
    }
    if (above) {
      offset += above->GetDepth() > cu.GetDepth();
    }
  }
  if (!Restrictions::Get().disable_ext_cabac_alt_split_flag_ctx) {
    int min_depth = pic_max_depth;
    int max_depth = 0;
    auto update_min_max =
      [&min_depth, &max_depth, pic_max_depth](const CodingUnit *tmp) {
      if (tmp) {
        min_depth = std::min(min_depth, tmp->GetDepth());
        max_depth = std::max(max_depth, tmp->GetDepth());
      } else {
        min_depth = 0;
        max_depth = pic_max_depth;
      }
    };
    update_min_max(cu.GetCodingUnitLeft());
    update_min_max(cu.GetCodingUnitAbove());
    min_depth = std::max(0, min_depth - 1);
    max_depth = std::min(pic_max_depth, max_depth + 1);
    if (cu.GetDepth() < min_depth) {
      offset = 3;
    } else if (cu.GetDepth() >= max_depth + 1) {
      offset = 4;
    }
  }
  return cu_split_quad_flag[offset];
}

template<typename Ctx>
Ctx& CabacContexts<Ctx>::GetIntraPredictorCtx(IntraMode intra_mode) {
  assert(!Restrictions::Get().disable_ext2_intra_6_predictors);
  static const std::array<uint8_t, kNbrIntraModesExt> kModeToCtxMapExt = {
    1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3
  };
  static const std::array<uint8_t, kNbrIntraModes> kModeToCtxMap = {
    1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
    3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
  };
  if (Restrictions::Get().disable_ext2_intra_67_modes) {
    return intra_pred_luma[kModeToCtxMap[intra_mode]];
  }
  return intra_pred_luma[kModeToCtxMapExt[intra_mode]];
}

template<typename Ctx>
Ctx& CabacContexts<Ctx>::GetInterDirBiCtx(const CodingUnit &cu) {
  if (Restrictions::Get().disable_cabac_inter_dir_ctx) {
    return inter_dir[0];
  }
  int idx = std::min(cu.GetDepth(), static_cast<int>(inter_dir.size()) - 1);
  if (!Restrictions::Get().disable_ext_cabac_alt_inter_dir_ctx) {
    static const int kMaxSize128Log2 = 7;
    int log2_size = (util::SizeToLog2(cu.GetWidth(YuvComponent::kY)) +
                     util::SizeToLog2(cu.GetHeight(YuvComponent::kY)) + 1) >> 1;
    idx = util::Clip3(kMaxSize128Log2 - log2_size, 0, 3);
  }
  return inter_dir[idx];
}

template<typename Ctx>
Ctx& CabacContexts<Ctx>::GetInterFullpelMvCtx(const CodingUnit &cu) {
  int offset = 0;
  const CodingUnit *tmp;
  if ((tmp = cu.GetCodingUnitLeft()) != nullptr && tmp->GetFullpelMv()) {
    offset++;
  }
  if ((tmp = cu.GetCodingUnitAbove()) != nullptr && tmp->GetFullpelMv()) {
    offset++;
  }
  return inter_fullpel_mv[offset];
}

template<typename Ctx>
Ctx&
CabacContexts<Ctx>::GetSubblockCsbfCtx(YuvComponent comp,
                                       const uint8_t *sublock_csbf,
                                       int posx, int posy, int width,
                                       int height, int *pattern_sig_ctx) {
  int below = false;
  int right = false;
  Ctx *ctx_base;
  if (!Restrictions::Get().disable_ext2_cabac_alt_residual_ctx) {
    ctx_base = util::IsLuma(comp) ?
      &coeff_ext.csbf_luma[0] : &coeff_ext.csbf_chroma[0];
  } else {
    ctx_base = util::IsLuma(comp) ? &coeff.csbf_luma[0] : &coeff.csbf_chroma[0];
  }
  if (posx < width - 1) {
    right = sublock_csbf[posy * width + posx + 1] != 0;
  }
  if (posy < height - 1) {
    below = sublock_csbf[(posy + 1) * width + posx] != 0;
  }
  *pattern_sig_ctx = right + (below << 1);
  if (Restrictions::Get().disable_cabac_subblock_csbf_ctx) {
    return ctx_base[0];
  }
  return ctx_base[right | below];
}

template<typename Ctx>
Ctx&
CabacContexts<Ctx>::GetCoeffSigCtx(YuvComponent comp, int pattern_sig_ctx,
                                   ScanOrder scan_order, int posx, int posy,
                                   const Coeff *in_coeff,
                                   ptrdiff_t in_coeff_stride,
                                   int width_log2, int height_log2) {
  static const uint8_t kCtxIndexMap[16] = {
    0, 1, 4, 5, 2, 3, 4, 5, 6, 6, 8, 8, 7, 7, 8, 8
  };
  if (!Restrictions::Get().disable_ext2_cabac_alt_residual_ctx) {
    const int width = 1 << width_log2;
    const int height = 1 << height_log2;
    const int size = (width_log2 + height_log2) >> 1;
    const int posxy = posx + posy;
    if (Restrictions::Get().disable_cabac_coeff_sig_ctx) {
      return coeff_ext.sig_luma[0];
    }
    in_coeff += posx + posy * in_coeff_stride;
    int offset = 0;
    if (posx < width - 1) {
      offset += in_coeff[1] != 0;
      if (posx < width - 2) {
        offset += in_coeff[2] != 0;
      }
      if (posy < height - 1) {
        offset += in_coeff[1 + in_coeff_stride] != 0;
      }
    }
    if (posy < height - 1) {
      offset += in_coeff[in_coeff_stride] != 0;
      if (posy < height - 2) {
        offset += in_coeff[in_coeff_stride * 2] != 0;
      }
    }
    offset = std::min(offset, 5);
    int start_offset = posxy < 2 ? 6 : 0;
    start_offset += util::IsLuma(comp) && posxy < 5 ? 6 : 0;
    start_offset += size > 2 && util::IsLuma(comp) ?
      18 << std::min(1, size - 3) : 0;
    return util::IsLuma(comp) ? coeff_ext.sig_luma[start_offset + offset] :
      coeff_ext.sig_chroma[start_offset + offset];
  } else {
    Ctx *ctx_base = util::IsLuma(comp) ?
      &coeff.sig_luma[0] : &coeff.sig_chroma[0];
    if ((!posx && !posy) || Restrictions::Get().disable_cabac_coeff_sig_ctx) {
      return ctx_base[0];
    }
    if (width_log2 == 2 && height_log2 == 2) {
      return ctx_base[kCtxIndexMap[4 * posy + posx]];
    }
    int start_offset = util::IsLuma(comp) ? 21 : 12;
    if (width_log2 == 3 && height_log2 == 3) {
      start_offset = scan_order == ScanOrder::kDiagonal ? 9 : 15;
    }
    int pos_x_in_subset = posx - ((posx >> 2) << 2);
    int pos_y_in_subset = posy - ((posy >> 2) << 2);
    int cnt = 0;
    if (pattern_sig_ctx == 0) {
      cnt = pos_x_in_subset + pos_y_in_subset <= 2 ?
        (pos_x_in_subset + pos_y_in_subset == 0 ? 2 : 1) : 0;
    } else if (pattern_sig_ctx == 1) {
      cnt = pos_y_in_subset <= 1 ? (pos_y_in_subset == 0 ? 2 : 1) : 0;
    } else if (pattern_sig_ctx == 2) {
      cnt = pos_x_in_subset <= 1 ? (pos_x_in_subset == 0 ? 2 : 1) : 0;
    } else {
      cnt = 2;
    }
    int comp_offset =
      util::IsLuma(comp) && ((posx >> 2) + (posy >> 2)) > 0 ? 3 : 0;
    return ctx_base[start_offset + comp_offset + cnt];
  }
}

template<typename Ctx>
Ctx&
CabacContexts<Ctx>::GetCoeffGreater1Ctx(YuvComponent comp, int ctx_set, int c1,
                                        int posx, int posy, bool is_last_coeff,
                                        const Coeff *in_coeff,
                                        ptrdiff_t in_coeff_stride,
                                        int width, int height) {
  if (!Restrictions::Get().disable_ext2_cabac_alt_residual_ctx) {
    const int posxy = posx + posy;
    if (is_last_coeff || Restrictions::Get().disable_cabac_coeff_greater1_ctx) {
      return util::IsLuma(comp) ?
        coeff_ext.greater1_luma[0] : coeff_ext.greater1_chroma[0];
    }
    in_coeff += posx + posy * in_coeff_stride;
    int offset = 0;
    if (posx < width - 1) {
      offset += std::abs(in_coeff[1]) > 1;
      if (posx < width - 2) {
        offset += std::abs(in_coeff[2]) > 1;
      }
      if (posy < height - 1) {
        offset += std::abs(in_coeff[1 + in_coeff_stride]) > 1;
      }
    }
    if (posy < height - 1) {
      offset += std::abs(in_coeff[in_coeff_stride]) > 1;
      if (posy < height - 2) {
        offset += std::abs(in_coeff[in_coeff_stride * 2]) > 1;
      }
    }
    offset = std::min(offset, 4) + 1;
    int start_offset = util::IsLuma(comp) ?
      (posxy < 3 ? 10 : (posxy < 10 ? 5 : 0)) : 0;
    return util::IsLuma(comp) ? coeff_ext.greater1_luma[start_offset + offset] :
      coeff_ext.greater1_chroma[start_offset + offset];
  } else {
    if (Restrictions::Get().disable_cabac_coeff_greater1_ctx) {
      return util::IsLuma(comp) ?
        coeff.greater1_luma[0] : coeff.greater1_chroma[0];
    }
    const int offset = 4 * ctx_set + c1;
    return util::IsLuma(comp) ?
      coeff.greater1_luma[offset] : coeff.greater1_chroma[offset];
  }
}

template<typename Ctx>
Ctx&
CabacContexts<Ctx>::GetCoeffGreater2Ctx(YuvComponent comp, int ctx_set,
                                        int posx, int posy, bool is_last_coeff,
                                        const Coeff *in_coeff,
                                        ptrdiff_t in_coeff_stride,
                                        int width, int height) {
  if (!Restrictions::Get().disable_ext2_cabac_alt_residual_ctx) {
    const int posxy = posx + posy;
    if (is_last_coeff || Restrictions::Get().disable_cabac_coeff_greater2_ctx) {
      return util::IsLuma(comp) ?
        coeff_ext.greater1_luma[0] : coeff_ext.greater1_chroma[0];
    }
    in_coeff += posx + posy * in_coeff_stride;
    int offset = 0;
    if (posx < width - 1) {
      offset += std::abs(in_coeff[1]) > 2;
      if (posx < width - 2) {
        offset += std::abs(in_coeff[2]) > 2;
      }
      if (posy < height - 1) {
        offset += std::abs(in_coeff[1 + in_coeff_stride]) > 2;
      }
    }
    if (posy < height - 1) {
      offset += std::abs(in_coeff[in_coeff_stride]) > 2;
      if (posy < height - 2) {
        offset += std::abs(in_coeff[in_coeff_stride * 2]) > 2;
      }
    }
    offset = std::min(offset, 4) + 1;
    int start_offset = util::IsLuma(comp) ?
      (posxy < 3 ? 10 : (posxy < 10 ? 5 : 0)) : 0;
    return util::IsLuma(comp) ? coeff_ext.greater1_luma[start_offset + offset] :
      coeff_ext.greater1_chroma[start_offset + offset];
  } else {
    static_assert(1 == constants::kMaxNumC2Flags, "Assumes only 1 c2 flag");
    if (Restrictions::Get().disable_cabac_coeff_greater2_ctx) {
      return util::IsLuma(comp) ?
        coeff_ext.greater1_luma[0] : coeff_ext.greater1_chroma[0];
    }
    return util::IsLuma(comp) ?
      coeff.greater2_luma[ctx_set] : coeff.greater2_chroma[ctx_set];
  }
}

template<typename Ctx>
uint32_t
CabacContexts<Ctx>::GetCoeffGolombRiceK(int posx, int posy,
                                        int width, int height,
                                        const Coeff *in_coeff,
                                        ptrdiff_t in_coeff_stride) {
  assert(!Restrictions::Get().disable_ext2_cabac_alt_residual_ctx);
  in_coeff += posx + posy * in_coeff_stride;
  int offset = 0;
  int num = 0;
  if (posx < width - 1) {
    offset += std::abs(in_coeff[1]);
    num += in_coeff[1] != 0;
    if (posx < width - 2) {
      offset += std::abs(in_coeff[2]);
      num += in_coeff[2] != 0;
    }
    if (posy < height - 1) {
      offset += std::abs(in_coeff[1 + in_coeff_stride]);
      num += in_coeff[1 + in_coeff_stride] != 0;
    }
  }
  if (posy < height - 1) {
    offset += std::abs(in_coeff[in_coeff_stride]);
    num += in_coeff[in_coeff_stride] != 0;
    if (posy < height - 2) {
      offset += std::abs(in_coeff[in_coeff_stride * 2]);
      num += in_coeff[in_coeff_stride * 2] != 0;
    }
  }
  const int num_k =
    static_cast<int>(TransformHelper::kGolombRiceRangeExt.size());
  uint32_t threshold = 4 + static_cast<uint32_t>(offset - num);
  for (uint32_t k = 0; k < num_k; k++) {
    if ((1u << (k + 3)) > threshold) {
      return k;
    }
  }
  return static_cast<uint32_t>(TransformHelper::kGolombRiceRangeExt.size() - 1);
}

template<typename Ctx>
Ctx&
CabacContexts<Ctx>::GetCoeffLastPosCtx(YuvComponent comp, int width, int height,
                                       int pos, bool is_pos_x) {
  const int size = is_pos_x ? width : height;
  if (util::IsLuma(comp)) {
    auto &ctx_base = is_pos_x ? coeff_last_pos_x_luma : coeff_last_pos_y_luma;
    if (Restrictions::Get().disable_cabac_coeff_last_pos_ctx &&
        Restrictions::Get().disable_ext_cabac_alt_last_pos_ctx) {
      return ctx_base[0];
    }
    int offset, shift;
    if (!Restrictions::Get().disable_ext_cabac_alt_last_pos_ctx) {
      static const std::array<uint8_t, 8> kOffsetMappingExt = {
        0, 0, 0, 3, 6, 10, 15, 21   // 1, 2, 4, 8, 16, 32, 64, 128
      };
      const int size_log2 = util::SizeToLog2(size);
      offset = kOffsetMappingExt[size_log2];
      shift = (size_log2 + 1) >> 2;
    } else {
      const int size_bits = util::SizeLog2Bits(size);
      offset = size_bits * 3 + ((size_bits + 1) >> 2);
      shift = (size_bits + 3) >> 2;
    }
    return ctx_base[offset + (pos >> shift)];
  } else {
    auto &ctx_base =
      is_pos_x ? coeff_last_pos_x_chroma : coeff_last_pos_y_chroma;
    if (Restrictions::Get().disable_cabac_coeff_last_pos_ctx &&
        Restrictions::Get().disable_ext_cabac_alt_last_pos_ctx) {
      return ctx_base[0];
    }
    int offset = 0;
    int shift;
    if (!Restrictions::Get().disable_ext_cabac_alt_last_pos_ctx) {
      shift = util::Clip3(size >> 3, 0, 2);
    } else {
      shift = util::SizeLog2Bits(size);
    }
    return ctx_base[offset + (pos >> shift)];
  }
}

template struct CabacContexts<ContextModel>;
template struct CabacContexts<ContextModelDynamic>;
template struct CabacContexts<ContextModelStatic>;

}   // namespace xvc
