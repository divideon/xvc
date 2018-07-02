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

#ifndef XVC_ENC_LIB_ENCODER_SETTINGS_H_
#define XVC_ENC_LIB_ENCODER_SETTINGS_H_

#include <array>
#include <string>

#include "xvc_common_lib/restrictions.h"

namespace xvc {

enum struct SpeedMode {
  kPlacebo = 0,
  kSlow = 1,
  kFast = 2,
  kTotalNumber = 3,
};

enum struct TuneMode {
  kDefault = 0,
  kPsnr = 1,
  kTotalNumber = 2,
};

struct EncoderSettings {
  void Initialize(SpeedMode speed_mode);
  void Initialize(RestrictedMode mode);
  void Tune(TuneMode tune_mode);
  void ParseExplicitSettings(std::string explicit_settings);

  // Encoder rdo behavior
  static constexpr bool kEncoderStrictRdoBitCounting = false;
  static constexpr bool kEncoderCountActualWrittenBits = true;
  static_assert(EncoderSettings::kEncoderCountActualWrittenBits ||
                EncoderSettings::kEncoderStrictRdoBitCounting,
                "Fast bit counting should use strict rdo bit signaling");

  // Fast encoder decisions (always used)
  static const bool rdo_quant = true;
  static const bool fast_cu_split_based_on_full_cu = true;
  static const bool fast_mode_selection_for_cached_cu = true;
  static const bool skip_mode_decision_for_identical_cu = false;
  static const bool fast_inter_transform_dist = true;  // not really any impact
  static const bool fast_inter_root_cbf_zero_bits = false;  // very small loss
  static const int inter_search_range_bi = 4;

  // Speed mode dependent settings
  int inter_search_range_uni_max = 256;
  int inter_search_range_uni_min = 96;
  int bipred_refinement_iterations = -1;
  int always_evaluate_intra_in_inter = -1;
  int default_num_ref_pics = -1;
  int max_binary_split_depth = -1;
  int fast_transform_select_eval = -1;
  int fast_intra_mode_eval_level = -1;
  int fast_transform_size_64 = -1;
  int fast_transform_select = -1;
  int fast_inter_local_illumination_comp = -1;
  int fast_inter_adaptive_fullpel_mv = -1;

  // Settings with default values used in all speed modes
  int fast_merge_eval = 1;
  int fast_quad_split_based_on_binary_split = 1;
  int eval_prev_mv_search_result = 1;
  int fast_inter_pred_bits = 0;
  int rdo_quant_2x2 = 1;
  int intra_qp_offset = 0;
  int smooth_lambda_scaling = 1;
  int adaptive_qp = 2;
  int aqp_strength = 13;
  int structural_ssd = 1;
  int structural_strength = 16;
  int encapsulation_mode = 0;
  int leading_pictures = 0;
  int source_padding = 0;
  int chroma_qp_offset_table = 1;
  int chroma_qp_offset_u = 0;
  int chroma_qp_offset_v = 0;
  int flat_lambda = 0;
  float lambda_scale_a = 1.0f;
  float lambda_scale_b = 0.0f;
  RestrictedMode restricted_mode = RestrictedMode::kUnrestricted;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_ENCODER_SETTINGS_H_
