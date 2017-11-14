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

#include "xvc_enc_lib/encoder_settings.h"

#include <cassert>

namespace xvc {

void EncoderSettings::Initialize(SpeedMode speed_mode) {
  switch (speed_mode) {
    case SpeedMode::kPlacebo:
      bipred_refinement_iterations = 4;
      always_evaluate_intra_in_inter = 1;
      default_num_ref_pics = 3;
      max_binary_split_depth = 3;
      fast_transform_select_eval = 0;
      break;
    case SpeedMode::kSlow:
      bipred_refinement_iterations = 1;
      always_evaluate_intra_in_inter = 0;
      default_num_ref_pics = 2;
      max_binary_split_depth = 2;
      fast_transform_select_eval = 1;
      break;
    default:
      assert(0);
      break;
  }
}

void EncoderSettings::Initialize(RestrictedMode mode) {
  restricted_mode = mode;
  switch (mode) {
    case RestrictedMode::kModeA:
      eval_prev_mv_search_result = 1;
      fast_intra_mode_eval_level = 1;
      fast_inter_pred_bits = 1;
      fast_merge_eval = 0;
      bipred_refinement_iterations = 1;
      always_evaluate_intra_in_inter = 0;
      smooth_lambda_scaling = 0;
      default_num_ref_pics = 2;
      max_binary_split_depth = 0;
      fast_transform_select_eval = 1;
      adaptive_qp = 0;
      structural_ssd = 0;
      chroma_qp_offset_table = 1;
      chroma_qp_offset_u = 0;
      chroma_qp_offset_v = 0;
      break;
    case RestrictedMode::kModeB:
      fast_quad_split_based_on_binary_split = 2;
      eval_prev_mv_search_result = 0;
      fast_intra_mode_eval_level = 2;
      fast_inter_pred_bits = 1;
      fast_merge_eval = 1;
      bipred_refinement_iterations = 1;
      always_evaluate_intra_in_inter = 0;
      smooth_lambda_scaling = 0;
      default_num_ref_pics = 2;
      max_binary_split_depth = 2;
      fast_transform_select_eval = 1;
      adaptive_qp = 0;
      structural_ssd = 0;
      chroma_qp_offset_table = 1;
      chroma_qp_offset_u = 1;
      chroma_qp_offset_v = 1;
      break;
    default:
      assert(0);
      break;
  }
}

void EncoderSettings::Tune(TuneMode tune_mode) {
  switch (tune_mode) {
    case TuneMode::kDefault:
      // No settings are changed in default mode.
      break;
    case TuneMode::kPsnr:
      adaptive_qp = 0;
      structural_ssd = 0;
      chroma_qp_offset_table = 0;
      chroma_qp_offset_u = 1;
      chroma_qp_offset_v = 1;
      break;
    default:
      assert(0);
      break;
  }
}

}   // namespace xvc
