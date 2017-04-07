/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/restrictions.h"

#include <cassert>

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

#if RESTRICTION_DISABLE_INTER_TMVP_MVP
  disable_inter_tmvp_mvp = true;
#endif

#if RESTRICTION_DISABLE_INTER_TMVP_MERGE
  disable_inter_tmvp_merge = true;
#endif

#if RESTRICTION_DISABLE_INTER_TMVP_REF_LIST_DERIVATION
  disable_inter_tmvp_ref_list_derivation = true;
#endif

#if RESTRICTION_DISABLE_INTER_MERGE_CANDIDATES
  disable_inter_merge_candidates = true;
#endif

#if RESTRICTION_DISABLE_INTER_MERGE_MODE
  disable_inter_merge_mode = true;
#endif

#if RESTRICTION_DISABLE_INTER_MERGE_BIPRED
  disable_inter_merge_bipred = true;
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

#if RESTRICTION_DISABLE_INTER_BIPRED
  disable_inter_bipred = true;
#endif

#if RESTRICTION_DISABLE_TRANSFORM_ADAPTIVE_SCAN_ORDER
  disable_transform_adaptive_scan_order = true;
#endif

#if RESTRICTION_DISABLE_TRANSFORM_RESIDUAL_GREATER_THAN_FLAGS
  disable_transform_residual_greater_than_flags = true;
#endif

#if RESTRICTION_DISABLE_TRANSFORM_RESIDUAL_GREATER2
  disable_transform_residual_greater2 = true;
#endif

#if RESTRICTION_DISABLE_TRANSFORM_LAST_POSITION
  disable_transform_last_position = true;
#endif

#if RESTRICTION_DISABLE_TRANSFORM_ROOT_CBF
  disable_transform_root_cbf = true;
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

#if RESTRICTION_DISABLE_CABAC_INTER_DIR_CTX
  disable_cabac_inter_dir_ctx = true;
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

#if RESTRICTION_DISABLE_CABAC_INIT_PER_PIC_TYPE
  disable_cabac_init_per_pic_type = true;
#endif

#if RESTRICTION_DISABLE_CABAC_INIT_PER_QP
  disable_cabac_init_per_qp = true;
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

#if RESTRICTION_DISABLE_DEBLOCK_WEAK_SAMPLE_DECISION
  disable_deblock_weak_sample_decision = true;
#endif

#if RESTRICTION_DISABLE_DEBLOCK_TWO_SAMPLES_WEAK_FILTER
  disable_deblock_two_samples_weak_filter = true;
#endif

#if RESTRICTION_DISABLE_DEBLOCK_DEPENDING_ON_QP
  disable_deblock_depending_on_qp = true;
#endif

#if RESTRICTION_DISABLE_EXT
  disable_ext = true;
#endif

#if RESTRICTION_DISABLE_EXT_TMVP_FULL_RESOLUTION
  disable_ext_tmvp_full_resolution = true;
#endif

#if RESTRICTION_DISABLE_EXT_TMVP_EXCLUDE_INTRA_FROM_REF_LIST
  disable_ext_tmvp_exclude_intra_from_ref_list = true;
#endif

#if RESTRICTION_DISABLE_EXT_REF_LIST_L0_TRIM
  disable_ext_ref_list_l0_trim = true;
#endif

#if RESTRICTION_DISABLE_EXT_IMPLICIT_PARTITION_TYPE
  disable_ext_implicit_partition_type = true;
#endif

#if RESTRICTION_DISABLE_EXT_CABAC_ALT_SPLIT_FLAG_CTX
  disable_ext_cabac_alt_split_flag_ctx = true;
#endif

#if RESTRICTION_DISABLE_EXT_CABAC_ALT_INTER_DIR_CTX
  disable_ext_cabac_alt_inter_dir_ctx = true;
#endif

#if RESTRICTION_DISABLE_EXT_CABAC_ALT_LAST_POS_CTX
  disable_ext_cabac_alt_last_pos_ctx = true;
#endif

#if RESTRICTION_DISABLE_EXT_TWO_CU_TREES
  disable_ext_two_cu_trees = true;
#endif

#if RESTRICTION_DISABLE_EXT_TRANSFORM_SIZE_64
  disable_ext_transform_size_64 = true;
#endif
}

void Restrictions::EnableRestrictedMode(RestrictedMode mode) {
  if (mode == RestrictedMode::kUnrestricted) {
    return;
  }
  if (mode == RestrictedMode::kModeA || mode == RestrictedMode::kModeB) {
    disable_ext_tmvp_full_resolution = true;
    disable_ext_tmvp_exclude_intra_from_ref_list = true;
    disable_ext_ref_list_l0_trim = true;
  }
  if (mode == RestrictedMode::kModeA) {
    disable_ext = true;
    disable_ext_implicit_partition_type = true;
    disable_ext_cabac_alt_split_flag_ctx = true;
    disable_ext_cabac_alt_inter_dir_ctx = true;
    disable_ext_cabac_alt_last_pos_ctx = true;
    disable_ext_two_cu_trees = true;
    disable_ext_transform_size_64 = true;
  }
}

}  // namespace xvc
