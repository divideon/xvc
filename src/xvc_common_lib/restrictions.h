/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_RESTRICTIONS_H_
#define XVC_COMMON_LIB_RESTRICTIONS_H_


namespace xvc {

enum class RestrictedMode {
  kUnrestricted = 0,
  kModeA = 1,
  kModeB = 2,
  kTotalNumber = 3,
};

// The Restrictions struct is used globally in the xvc namespace to check if
// any features should be disabled anywhere in the encoding/decoding process.
// In the first official version of xvc all the disable flags are equal to
// false. In future versions of xvc, one or more of the disable flags may be
// initialized to true.
typedef struct Restrictions {
public:
  static const Restrictions& Get() { return instance; }

  static bool GetIntraRestrictions() {
    return instance.disable_intra_ref_padding ||
      instance.disable_intra_ref_sample_filter ||
      instance.disable_intra_dc_post_filter ||
      instance.disable_intra_ver_hor_post_filter ||
      instance.disable_intra_planar ||
      instance.disable_intra_mpm_prediction ||
      instance.disable_intra_chroma_predictor;
  }

  static bool GetInterRestrictions() {
    return instance.disable_inter_mvp ||
      instance.disable_inter_scaling_mvp ||
      instance.disable_inter_tmvp_mvp ||
      instance.disable_inter_tmvp_merge ||
      instance.disable_inter_tmvp_ref_list_derivation ||
      instance.disable_inter_merge_candidates ||
      instance.disable_inter_merge_mode ||
      instance.disable_inter_merge_bipred ||
      instance.disable_inter_skip_mode ||
      instance.disable_inter_chroma_subpel ||
      instance.disable_inter_mvd_greater_than_flags ||
      instance.disable_inter_bipred;
  }

  static bool GetTransformRestrictions() {
    return instance.disable_transform_adaptive_scan_order ||
      instance.disable_transform_residual_greater_than_flags ||
      instance.disable_transform_residual_greater2 ||
      instance.disable_transform_last_position ||
      instance.disable_transform_root_cbf ||
      instance.disable_transform_cbf ||
      instance.disable_transform_subblock_csbf ||
      instance.disable_transform_adaptive_exp_golomb;
  }

  static bool GetCabacRestrictions() {
    return instance.disable_cabac_ctx_update ||
      instance.disable_cabac_split_flag_ctx ||
      instance.disable_cabac_skip_flag_ctx ||
      instance.disable_cabac_inter_dir_ctx ||
      instance.disable_cabac_subblock_csbf_ctx ||
      instance.disable_cabac_coeff_sig_ctx ||
      instance.disable_cabac_coeff_greater1_ctx ||
      instance.disable_cabac_coeff_greater2_ctx ||
      instance.disable_cabac_coeff_last_pos_ctx ||
      instance.disable_cabac_init_per_pic_type ||
      instance.disable_cabac_init_per_qp;
  }

  static bool GetDeblockRestrictions() {
    return instance.disable_deblock_strong_filter ||
      instance.disable_deblock_weak_filter ||
      instance.disable_deblock_chroma_filter ||
      instance.disable_deblock_boundary_strength_zero ||
      instance.disable_deblock_boundary_strength_one ||
      instance.disable_deblock_initial_sample_decision ||
      instance.disable_deblock_weak_sample_decision ||
      instance.disable_deblock_two_samples_weak_filter ||
      instance.disable_deblock_depending_on_qp;
  }

  static bool GetExtRestrictions() {
    return instance.disable_ext ||
      instance.disable_ext_tmvp_full_resolution ||
      instance.disable_ext_tmvp_exclude_intra_from_ref_list ||
      instance.disable_ext_ref_list_l0_trim ||
      instance.disable_ext_implicit_partition_type ||
      instance.disable_ext_cabac_alt_split_flag_ctx ||
      instance.disable_ext_cabac_alt_inter_dir_ctx ||
      instance.disable_ext_cabac_alt_last_pos_ctx ||
      instance.disable_ext_two_cu_trees ||
      instance.disable_ext_transform_size_64 ||
      instance.disable_ext_alt_num_intra_fast_modes ||
      instance.disable_ext_reuse_mv_candidates;
  }

  bool disable_intra_ref_padding = false;
  bool disable_intra_ref_sample_filter = false;
  bool disable_intra_dc_post_filter = false;
  bool disable_intra_ver_hor_post_filter = false;
  bool disable_intra_planar = false;
  bool disable_intra_mpm_prediction = false;
  bool disable_intra_chroma_predictor = false;
  bool disable_inter_mvp = false;
  bool disable_inter_scaling_mvp = false;
  bool disable_inter_tmvp_mvp = false;
  bool disable_inter_tmvp_merge = false;
  bool disable_inter_tmvp_ref_list_derivation = false;
  bool disable_inter_merge_candidates = false;
  bool disable_inter_merge_mode = false;
  bool disable_inter_merge_bipred = false;
  bool disable_inter_skip_mode = false;
  bool disable_inter_chroma_subpel = false;
  bool disable_inter_mvd_greater_than_flags = false;
  bool disable_inter_bipred = false;
  bool disable_transform_adaptive_scan_order = false;
  bool disable_transform_residual_greater_than_flags = false;
  bool disable_transform_residual_greater2 = false;
  bool disable_transform_last_position = false;
  bool disable_transform_root_cbf = false;
  bool disable_transform_cbf = false;
  bool disable_transform_subblock_csbf = false;
  bool disable_transform_adaptive_exp_golomb = false;
  bool disable_cabac_ctx_update = false;
  bool disable_cabac_split_flag_ctx = false;
  bool disable_cabac_skip_flag_ctx = false;
  bool disable_cabac_inter_dir_ctx = false;
  bool disable_cabac_subblock_csbf_ctx = false;
  bool disable_cabac_coeff_sig_ctx = false;
  bool disable_cabac_coeff_greater1_ctx = false;
  bool disable_cabac_coeff_greater2_ctx = false;
  bool disable_cabac_coeff_last_pos_ctx = false;
  bool disable_cabac_init_per_pic_type = false;
  bool disable_cabac_init_per_qp = false;
  bool disable_deblock_strong_filter = false;
  bool disable_deblock_weak_filter = false;
  bool disable_deblock_chroma_filter = false;
  bool disable_deblock_boundary_strength_zero = false;
  bool disable_deblock_boundary_strength_one = false;
  bool disable_deblock_initial_sample_decision = false;
  bool disable_deblock_weak_sample_decision = false;
  bool disable_deblock_two_samples_weak_filter = false;
  bool disable_deblock_depending_on_qp = false;
  bool disable_ext = false;
  bool disable_ext_tmvp_full_resolution = false;
  bool disable_ext_tmvp_exclude_intra_from_ref_list = false;
  bool disable_ext_ref_list_l0_trim = false;
  bool disable_ext_implicit_partition_type = false;
  bool disable_ext_cabac_alt_split_flag_ctx = false;
  bool disable_ext_cabac_alt_inter_dir_ctx = false;
  bool disable_ext_cabac_alt_last_pos_ctx = false;
  bool disable_ext_two_cu_trees = false;
  bool disable_ext_transform_size_64 = false;
  bool disable_ext_alt_num_intra_fast_modes = false;
  bool disable_ext_reuse_mv_candidates = false;

private:
  // The GetRW function shall be used only when there is a need to
  // modify the restriction flags.
  // It shall not be called anywhere in the code except for
  // 1. in the segment header read function in the decoder and
  // 2. the SetRestrictedMode in the encoder class.
  // For this reason, the GetRW function is private and only
  // accessible by its friend classes.
  friend class SegmentHeaderReader;
  friend class Encoder;
  static Restrictions instance;
  static Restrictions& GetRW() { return instance; }

  Restrictions();
  void EnableRestrictedMode(RestrictedMode mode);
} Restrictions;

}   // namespace xvc

#endif  // XVC_COMMON_LIB_RESTRICTIONS_H_
