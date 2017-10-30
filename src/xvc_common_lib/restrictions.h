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
  Restrictions();
  static const Restrictions &Get() {
    return instance;
  }

  bool GetIntraRestrictions() const {
    return disable_intra_ref_padding ||
      disable_intra_ref_sample_filter ||
      disable_intra_dc_post_filter ||
      disable_intra_ver_hor_post_filter ||
      disable_intra_planar ||
      disable_intra_mpm_prediction ||
      disable_intra_chroma_predictor;
  }

  bool GetInterRestrictions() const {
    return disable_inter_mvp ||
      disable_inter_scaling_mvp ||
      disable_inter_tmvp_mvp ||
      disable_inter_tmvp_merge ||
      disable_inter_tmvp_ref_list_derivation ||
      disable_inter_merge_candidates ||
      disable_inter_merge_mode ||
      disable_inter_merge_bipred ||
      disable_inter_skip_mode ||
      disable_inter_chroma_subpel ||
      disable_inter_mvd_greater_than_flags ||
      disable_inter_bipred;
  }

  bool GetTransformRestrictions() const {
    return disable_transform_adaptive_scan_order ||
      disable_transform_residual_greater_than_flags ||
      disable_transform_residual_greater2 ||
      disable_transform_last_position ||
      disable_transform_root_cbf ||
      disable_transform_cbf ||
      disable_transform_subblock_csbf ||
      disable_transform_skip ||
      disable_transform_sign_hiding ||
      disable_transform_adaptive_exp_golomb;
  }

  bool GetCabacRestrictions() const {
    return disable_cabac_ctx_update ||
      disable_cabac_split_flag_ctx ||
      disable_cabac_skip_flag_ctx ||
      disable_cabac_inter_dir_ctx ||
      disable_cabac_subblock_csbf_ctx ||
      disable_cabac_coeff_sig_ctx ||
      disable_cabac_coeff_greater1_ctx ||
      disable_cabac_coeff_greater2_ctx ||
      disable_cabac_coeff_last_pos_ctx ||
      disable_cabac_init_per_pic_type ||
      disable_cabac_init_per_qp;
  }

  bool GetDeblockRestrictions() const {
    return disable_deblock_strong_filter ||
      disable_deblock_weak_filter ||
      disable_deblock_chroma_filter ||
      disable_deblock_boundary_strength_zero ||
      disable_deblock_boundary_strength_one ||
      disable_deblock_initial_sample_decision ||
      disable_deblock_weak_sample_decision ||
      disable_deblock_two_samples_weak_filter ||
      disable_deblock_depending_on_qp;
  }

  bool GetHighLevelRestrictions() const {
      return disable_high_level_default_checksum_method;
  }

  bool GetExtRestrictions() const {
    return disable_ext_sink ||
      disable_ext_implicit_last_ctu ||
      disable_ext_tmvp_full_resolution ||
      disable_ext_tmvp_exclude_intra_from_ref_list ||
      disable_ext_ref_list_l0_trim ||
      disable_ext_implicit_partition_type ||
      disable_ext_cabac_alt_split_flag_ctx ||
      disable_ext_cabac_alt_inter_dir_ctx ||
      disable_ext_cabac_alt_last_pos_ctx ||
      disable_ext_two_cu_trees ||
      disable_ext_transform_size_64 ||
      disable_ext_intra_unrestricted_predictor ||
      disable_ext_deblock_subblock_size_4 ||
      disable_ext_transform_high_precision ||
      disable_ext_transform_select ||
      disable_ext_intra_extra_modes ||
      disable_ext_intra_extra_predictors ||
      disable_ext_intra_chroma_from_luma ||
      disable_ext_inter_adaptive_fullpel_mv;
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
  bool disable_transform_skip = false;
  bool disable_transform_sign_hiding = false;
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
  bool disable_high_level_default_checksum_method = false;
  bool disable_ext_sink = false;
  bool disable_ext_implicit_last_ctu = false;
  bool disable_ext_intra_extra_modes = false;
  bool disable_ext_intra_extra_predictors = false;
  bool disable_ext_intra_chroma_from_luma = false;
  bool disable_ext_inter_adaptive_fullpel_mv = false;
  bool disable_ext_tmvp_full_resolution = false;
  bool disable_ext_tmvp_exclude_intra_from_ref_list = false;
  bool disable_ext_ref_list_l0_trim = false;
  bool disable_ext_implicit_partition_type = false;
  bool disable_ext_cabac_alt_split_flag_ctx = false;
  bool disable_ext_cabac_alt_inter_dir_ctx = false;
  bool disable_ext_cabac_alt_last_pos_ctx = false;
  bool disable_ext_two_cu_trees = false;
  bool disable_ext_transform_size_64 = false;
  bool disable_ext_intra_unrestricted_predictor = false;
  bool disable_ext_deblock_subblock_size_4 = false;
  bool disable_ext_transform_high_precision = false;
  bool disable_ext_transform_select = false;

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
  friend class Decoder;
  friend class ThreadDecoder;
  static thread_local Restrictions instance;
  static Restrictions &GetRW() { return instance; }

  void EnableRestrictedMode(RestrictedMode mode);
} Restrictions;

}   // namespace xvc

#endif  // XVC_COMMON_LIB_RESTRICTIONS_H_
