/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/restrictions.h"

namespace xvc {

Restrictions Restrictions::instance;

Restrictions::Restrictions() {
#if RESTRICTION_DISABLE_INTRA_REF_PADDING
  disable_intra_ref_padding = true;
#endif

#if RESTRICTION_DISABLE_INTRA_REF_SAMPLE_FILTER
  disable_intra_ref_sample_filter = true;
#endif

#if RESTRICTION_DISABLE_INTRA_DC_POST_FILTER
  disable_intra_dc_post_filter = true;
#endif

#if RESTRICTION_DISABLE_INTRA_VER_HOR_POST_FILTER
  disable_intra_ver_hor_post_filter = true;
#endif

#if RESTRICTION_DISABLE_INTRA_PLANAR
  disable_intra_planar = true;
#endif

#if RESTRICTION_DISABLE_INTRA_MPM_PREDICTION
  disable_intra_mpm_prediction = true;
#endif

#if RESTRICTION_DISABLE_INTRA_CHROMA_PREDICTOR
  disable_intra_chroma_predictor = true;
#endif

#if RESTRICTION_DISABLE_INTER_MVP
  disable_inter_mvp = true;
#endif

#if RESTRICTION_DISABLE_INTER_SCALING_MVP
  disable_inter_scaling_mvp = true;
#endif

#if RESTRICTION_DISABLE_INTER_MERGE_CANDIDATES
  disable_inter_merge_candidates = true;
#endif

#if RESTRICTION_DISABLE_INTER_MERGE_MODE
  disable_inter_merge_mode = true;
#endif

#if RESTRICTION_DISABLE_INTER_SKIP_MODE
  disable_inter_skip_mode = true;
#endif

#if RESTRICTION_DISABLE_INTER_CHROMA_SUBPEL
  disable_inter_chroma_subpel = true;
#endif

#if RESTRICTION_DISABLE_INTER_MVD_GREATER_THAN_FLAGS
  disable_inter_mvd_greater_than_flags = true;
#endif

#if RESTRICTION_DISABLE_TRANSFORM_ADAPTIVE_SCAN_ORDER
  disable_transform_adaptive_scan_order = true;
#endif

#if RESTRICTION_DISABLE_TRANSFORM_RESIDUAL_GREATER_THAN_FLAGS
  disable_transform_residual_greater_than_flags = true;
#endif

#if RESTRICTION_DISABLE_TRANSFORM_LAST_POSITION
  disable_transform_last_position = true;
#endif

#if RESTRICTION_DISABLE_TRANSFORM_CBF
  disable_transform_cbf = true;
#endif

#if RESTRICTION_DISABLE_TRANSFORM_SUBBLOCK_CSBF
  disable_transform_subblock_csbf = true;
#endif

#if RESTRICTION_DISABLE_TRANSFORM_ADAPTIVE_EXP_GOLOMB
  disable_transform_adaptive_exp_golomb = true;
#endif

#if RESTRICTION_DISABLE_CABAC_CTX_UPDATE
  disable_cabac_ctx_update = true;
#endif

#if RESTRICTION_DISABLE_CABAC_SPLIT_FLAG_CTX
  disable_cabac_split_flag_ctx = true;
#endif

#if RESTRICTION_DISABLE_CABAC_SKIP_FLAG_CTX
  disable_cabac_skip_flag_ctx = true;
#endif

#if RESTRICTION_DISABLE_CABAC_SUBBLOCK_CSBF_CTX
  disable_cabac_subblock_csbf_ctx = true;
#endif

#if RESTRICTION_DISABLE_CABAC_COEFF_SIG_CTX
  disable_cabac_coeff_sig_ctx = true;
#endif

#if RESTRICTION_DISABLE_CABAC_COEFF_GREATER1_CTX
  disable_cabac_coeff_greater1_ctx = true;
#endif

#if RESTRICTION_DISABLE_CABAC_COEFF_GREATER2_CTX
  disable_cabac_coeff_greater2_ctx = true;
#endif

#if RESTRICTION_DISABLE_CABAC_COEFF_LAST_POS_CTX
  disable_cabac_coeff_last_pos_ctx = true;
#endif

#if RESTRICTION_DISABLE_DEBLOCK_STRONG_FILTER
  disable_deblock_strong_filter = true;
#endif

#if RESTRICTION_DISABLE_DEBLOCK_WEAK_FILTER
  disable_deblock_weak_filter = true;
#endif

#if RESTRICTION_DISABLE_DEBLOCK_CHROMA_FILTER
  disable_deblock_chroma_filter = true;
#endif

#if RESTRICTION_DISABLE_DEBLOCK_BOUNDARY_STRENGTH_ZERO
  disable_deblock_boundary_strength_zero = true;
#endif

#if RESTRICTION_DISABLE_DEBLOCK_BOUNDARY_STRENGTH_ONE
  disable_deblock_boundary_strength_one = true;
#endif

#if RESTRICTION_DISABLE_DEBLOCK_INITIAL_SAMPLE_DECISION
  disable_deblock_initial_sample_decision = true;
#endif

#if RESTRICTION_DISABLE_DEBLOCK_TWO_SAMPLES_WEAK_FILTER
  disable_deblock_two_samples_weak_filter = true;
#endif

#if RESTRICTION_DISABLE_DEBLOCK_DEPENDING_ON_QP
  disable_deblock_depending_on_qp = true;
#endif
}

}  // namespace xvc
