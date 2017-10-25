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

#ifndef XVC_COMMON_LIB_CABAC_H_
#define XVC_COMMON_LIB_CABAC_H_

#include <array>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/context_model.h"
#include "xvc_common_lib/picture_types.h"
#include "xvc_common_lib/transform.h"
#include "xvc_common_lib/quantize.h"

namespace xvc {

class Cabac {
public:
  static inline uint8_t RenormTable(uint32_t lps) {
    return kRenormTable_[lps];
  }
  static inline uint8_t RangeTable(uint32_t state, uint32_t range) {
    return kRangeTable_[state][range];
  }

private:
  static const uint8_t kRenormTable_[32];
  static const std::array<std::array<const uint8_t, 4>,
    1 << ContextModel::CONTEXT_STATE_BITS> kRangeTable_;
};

struct CabacContexts {
public:
  static const int kNumSplitQuadFlagCtx = 5;
  static const int kNumSplitBinaryCtx = 6;
  static const int kNumSkipFlagCtx = 3;
  static const int kNumMergeFlagCtx = 1;
  static const int kNumMergeIdxCtx = 1;
  static const int kNumPartSizeCtx = 4;
  static const int kNumPredModeCtx = 1;
  static const int kNumIntraPredCtxLuma = 9;
  static const int kNumIntraPredCtxChroma = 2;
  static const int kNumInterDirCtx = 5;
  static const int kNumMvdCtx = 2;
  static const int kNumRefIdxCtx = 2;
  static const int kNumTransSubdivFlagCtx = 3;
  static const int kNumCuCbfCtx = 2;
  static const int kNumCuCbfCtxLuma = 1;
  static const int kNumCuCbfCtxChroma = 1;
  static const int kNumCuRootCbfCtx = 1;
  static const int kNumDeltaQpCtx = 3;
  static const int kNumSubblockCsbfCtx = 4;
  static const int kNumSubblockCsbfCtxLuma = 2;
  static const int kNumSubblockCsbfCtxChroma = 2;
  static const int kNumCoeffSigCtx = 42;
  static const int kNumCoeffSigCtxLuma = 27;
  static const int kNumCoeffSigCtxChroma = 15;
  static const int kNumCoeffGreater1Ctx = 24;
  static const int kNumCoeffGreater1CtxLuma = 16;
  static const int kNumCoeffGreater1CtxChroma = 8;
  static const int kNumCoeffGreater2Ctx = 6;
  static const int kNumCoeffGreater2CtxLuma = 4;
  static const int kNumCoeffGreater2CtxChroma = 2;
  static const int kNumCoeffLastPosCtx = 28;
  static const int kNumCoeffLastPosCtxLuma = 25;
  static const int kNumCoeffLastPosCtxChroma = 3;
  static const int kNumMvpIdxCtx = 1;
  static const int kNumSaoMergeFlagCtx = 1;
  static const int kNumSaoTypeIdxCtx = 1;
  static const int kNumTransformSkipFlagCtx = 2;
  static const int kNumTransformSelectEnableCtx = 6;
  static const int kNumTransformSelectIdxCtx = 4;
  static const int kNumTquantBypassFlagCtx = 1;

  void ResetStates(const Qp &qp, PicturePredictionType pic_type);

  ContextModel& GetSkipFlagCtx(const CodingUnit &cu);
  ContextModel& GetSplitBinaryCtx(const CodingUnit &cu);
  ContextModel& GetSplitFlagCtx(const CodingUnit &cu, int max_depth);
  ContextModel& GetIntraPredictorCtx(IntraMode intra_mode);
  ContextModel& GetInterDirBiCtx(const CodingUnit &cu);
  ContextModel& GetSubblockCsbfCtx(YuvComponent comp,
                                   const uint8_t *sig_sublock, int posx,
                                   int posy, int width, int height,
                                   int *pattern_sig_ctx);
  ContextModel& GetCoeffSigCtx(YuvComponent comp, int pattern_sig_ctx,
                               ScanOrder scan_order, int posx, int posy,
                               int width_log2, int height_log2);
  ContextModel& GetCoeffGreaterThan1Ctx(YuvComponent comp, int ctx_set,
                                        int c1);
  ContextModel& GetCoeffGreaterThan2Ctx(YuvComponent comp, int ctx_set);
  ContextModel& GetCoeffLastPosCtx(YuvComponent comp, int width, int height,
                                   int pos, bool is_pos_x);

  std::array<ContextModel, kNumCuCbfCtxLuma> cu_cbf_luma;
  std::array<ContextModel, kNumCuCbfCtxChroma> cu_cbf_chroma;
  std::array<ContextModel, kNumPartSizeCtx> cu_part_size;
  std::array<ContextModel, kNumPredModeCtx> cu_pred_mode;
  std::array<ContextModel, kNumCuRootCbfCtx> cu_root_cbf;
  std::array<ContextModel, kNumSkipFlagCtx> cu_skip_flag;
  std::array<ContextModel, kNumSplitQuadFlagCtx> cu_split_quad_flag;
  std::array<ContextModel, kNumSplitBinaryCtx> cu_split_binary;
  std::array<ContextModel, kNumInterDirCtx> inter_dir;
  std::array<ContextModel, kNumMergeFlagCtx> inter_merge_flag;
  std::array<ContextModel, kNumMergeIdxCtx> inter_merge_idx;
  std::array<ContextModel, kNumMvdCtx> inter_mvd;
  std::array<ContextModel, kNumMvpIdxCtx> inter_mvp_idx;
  std::array<ContextModel, kNumRefIdxCtx> inter_ref_idx;
  std::array<ContextModel, kNumIntraPredCtxLuma> intra_pred_luma;
  std::array<ContextModel, kNumIntraPredCtxChroma> intra_pred_chroma;
  std::array<ContextModel, kNumSubblockCsbfCtxLuma> subblock_csbf_luma;
  std::array<ContextModel, kNumSubblockCsbfCtxChroma> subblock_csbf_chroma;
  std::array<ContextModel, kNumCoeffSigCtxLuma> coeff_sig_luma;
  std::array<ContextModel, kNumCoeffSigCtxChroma> coeff_sig_chroma;
  std::array<ContextModel, kNumCoeffGreater1CtxLuma> coeff_greater1_luma;
  std::array<ContextModel, kNumCoeffGreater1CtxChroma> coeff_greater1_chroma;
  std::array<ContextModel, kNumCoeffGreater2CtxLuma> coeff_greater2_luma;
  std::array<ContextModel, kNumCoeffGreater2CtxChroma> coeff_greater2_chroma;
  std::array<ContextModel, kNumCoeffLastPosCtxLuma> coeff_last_pos_x_luma;
  std::array<ContextModel, kNumCoeffLastPosCtxChroma> coeff_last_pos_x_chroma;
  std::array<ContextModel, kNumCoeffLastPosCtxLuma> coeff_last_pos_y_luma;
  std::array<ContextModel, kNumCoeffLastPosCtxChroma> coeff_last_pos_y_chroma;
  std::array<ContextModel, kNumTransformSkipFlagCtx> transform_skip_flag;
  std::array<ContextModel, kNumTransformSelectEnableCtx> transform_select_flag;
  std::array<ContextModel, kNumTransformSelectIdxCtx> transform_select_idx;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_CABAC_H_
