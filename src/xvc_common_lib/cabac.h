/******************************************************************************
* Copyright (C) 2018, Divideon.
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
* This library is also available under a commercial license.
* Please visit https://xvc.io/license/ for more information.
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

struct CabacCommon {
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
  static const int kNumInterFullpelMvCtx = 3;
  static const int kNumAffineCtx = 3;
  static const int kNumLicFlagCtx = 1;
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
  static const int kNumExtSubblockCsbfCtx = 4;
  static const int kNumExtSubblockCsbfCtxLuma = 2;
  static const int kNumExtSubblockCsbfCtxChroma = 2;
  static const int kNumCoeffSigCtx = 42;
  static const int kNumCoeffSigCtxLuma = 27;
  static const int kNumCoeffSigCtxChroma = 15;
  static const int kNumExtCoeffSigCtx = 66;
  static const int kNumExtCoeffSigCtxLuma = 54;
  static const int kNumExtCoeffSigCtxChroma = 12;
  static const int kNumCoeffGreater1Ctx = 24;
  static const int kNumCoeffGreater1CtxLuma = 16;
  static const int kNumCoeffGreater1CtxChroma = 8;
  static const int kNumExtCoeffGreater1Ctx = 22;
  static const int kNumExtCoeffGreater1CtxLuma = 16;
  static const int kNumExtCoeffGreater1CtxChroma = 6;
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
};

template<typename ContextModel>
struct CabacContexts : public CabacCommon {
public:
  CabacContexts();
  void ResetStates(const Qp &qp, PicturePredictionType pic_type);

  ContextModel& GetAffineCtx(const CodingUnit &cu);
  ContextModel& GetSkipFlagCtx(const CodingUnit &cu);
  ContextModel& GetSplitBinaryCtx(const CodingUnit &cu);
  ContextModel& GetSplitFlagCtx(const CodingUnit &cu, int max_depth);
  ContextModel& GetIntraPredictorCtx(IntraMode intra_mode);
  ContextModel& GetInterDirBiCtx(const CodingUnit &cu);
  ContextModel& GetInterFullpelMvCtx(const CodingUnit &cu);
  ContextModel& GetSubblockCsbfCtx(YuvComponent comp,
                                   const uint8_t *sig_sublock, int posx,
                                   int posy, int width, int height,
                                   int *pattern_sig_ctx);
  ContextModel& GetCoeffSigCtx(YuvComponent comp, int pattern_sig_ctx,
                               ScanOrder scan_order, int posx, int posy,
                               const Coeff *coeff, ptrdiff_t coeff_stride,
                               int width_log2, int height_log2);
  ContextModel& GetCoeffGreater1Ctx(YuvComponent comp, int ctx_set, int c1,
                                    int posx, int posy, bool is_last_coeff,
                                    const Coeff *coeff, ptrdiff_t coeff_stride,
                                    int width_log2, int height_log2);
  ContextModel& GetCoeffGreater2Ctx(YuvComponent comp, int ctx_set,
                                    int posx, int posy, bool is_last_coeff,
                                    const Coeff *coeff, ptrdiff_t coeff_stride,
                                    int width_log2, int height_log2);
  uint32_t GetCoeffGolombRiceK(int posx, int posy, int width, int height,
                               const Coeff *coeff, ptrdiff_t coeff_stride);
  ContextModel& GetCoeffLastPosCtx(YuvComponent comp, int width, int height,
                                   int pos, bool is_pos_x);

  const Restrictions *restrictions_;
  std::array<ContextModel, kNumCuCbfCtxLuma> cu_cbf_luma;
  std::array<ContextModel, kNumCuCbfCtxChroma> cu_cbf_chroma;
  std::array<ContextModel, kNumPartSizeCtx> cu_part_size;
  std::array<ContextModel, kNumPredModeCtx> cu_pred_mode;
  std::array<ContextModel, kNumCuRootCbfCtx> cu_root_cbf;
  std::array<ContextModel, kNumSkipFlagCtx> cu_skip_flag;
  std::array<ContextModel, kNumSplitQuadFlagCtx> cu_split_quad_flag;
  std::array<ContextModel, kNumSplitBinaryCtx> cu_split_binary;
  std::array<ContextModel, kNumInterDirCtx> inter_dir;
  std::array<ContextModel, kNumInterFullpelMvCtx> inter_fullpel_mv;
  std::array<ContextModel, kNumMergeFlagCtx> inter_merge_flag;
  std::array<ContextModel, kNumMergeIdxCtx> inter_merge_idx;
  std::array<ContextModel, kNumMvdCtx> inter_mvd;
  std::array<ContextModel, kNumMvpIdxCtx> inter_mvp_idx;
  std::array<ContextModel, kNumRefIdxCtx> inter_ref_idx;
  std::array<ContextModel, kNumIntraPredCtxLuma> intra_pred_luma;
  std::array<ContextModel, kNumIntraPredCtxChroma> intra_pred_chroma;
  std::array<ContextModel, kNumAffineCtx> affine_flag;
  std::array<ContextModel, kNumLicFlagCtx> lic_flag;
  std::array<ContextModel, kNumDeltaQpCtx> delta_qp;
  union {
    struct {
      std::array<ContextModel, kNumSubblockCsbfCtxLuma> csbf_luma;
      std::array<ContextModel, kNumSubblockCsbfCtxChroma> csbf_chroma;
      std::array<ContextModel, kNumCoeffSigCtxLuma> sig_luma;
      std::array<ContextModel, kNumCoeffSigCtxChroma> sig_chroma;
      std::array<ContextModel, kNumCoeffGreater1CtxLuma> greater1_luma;
      std::array<ContextModel, kNumCoeffGreater1CtxChroma> greater1_chroma;
      std::array<ContextModel, kNumCoeffGreater2CtxLuma> greater2_luma;
      std::array<ContextModel, kNumCoeffGreater2CtxChroma> greater2_chroma;
    } coeff;
    struct {
      std::array<ContextModel, kNumExtSubblockCsbfCtxLuma> csbf_luma;
      std::array<ContextModel, kNumExtSubblockCsbfCtxChroma> csbf_chroma;
      std::array<ContextModel, kNumExtCoeffSigCtxLuma> sig_luma;
      std::array<ContextModel, kNumExtCoeffSigCtxChroma> sig_chroma;
      std::array<ContextModel, kNumExtCoeffGreater1CtxLuma> greater1_luma;
      std::array<ContextModel, kNumExtCoeffGreater1CtxChroma> greater1_chroma;
    } coeff_ext;
  };
  std::array<ContextModel, kNumCoeffLastPosCtxLuma> coeff_last_pos_x_luma;
  std::array<ContextModel, kNumCoeffLastPosCtxChroma> coeff_last_pos_x_chroma;
  std::array<ContextModel, kNumCoeffLastPosCtxLuma> coeff_last_pos_y_luma;
  std::array<ContextModel, kNumCoeffLastPosCtxChroma> coeff_last_pos_y_chroma;
  std::array<ContextModel, kNumTransformSkipFlagCtx> transform_skip_flag;
  std::array<ContextModel, kNumTransformSelectEnableCtx> transform_select_flag;
  std::array<ContextModel, kNumTransformSelectIdxCtx> transform_select_idx;
};

extern template struct CabacContexts<ContextModel>;
extern template struct CabacContexts<ContextModelDynamic>;
extern template struct CabacContexts<ContextModelStatic>;

}   // namespace xvc

#endif  // XVC_COMMON_LIB_CABAC_H_
