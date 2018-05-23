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
#include <sstream>

namespace xvc {

void EncoderSettings::Initialize(SpeedMode speed_mode) {
  switch (speed_mode) {
    case SpeedMode::kPlacebo:
      inter_search_range_uni_max = 384;
      inter_search_range_uni_min = 96;
      bipred_refinement_iterations = 4;
      always_evaluate_intra_in_inter = 1;
      default_num_ref_pics = 3;
      max_binary_split_depth = 3;
      fast_transform_select_eval = 0;
      fast_intra_mode_eval_level = 1;
      fast_transform_size_64 = 0;
      fast_transform_select = 0;
      fast_inter_local_illumination_comp = 0;
      fast_inter_adaptive_fullpel_mv = 0;
      break;
    case SpeedMode::kSlow:
      bipred_refinement_iterations = 1;
      always_evaluate_intra_in_inter = 0;
      default_num_ref_pics = 2;
      max_binary_split_depth = 2;
      fast_transform_select_eval = 1;
      fast_intra_mode_eval_level = 1;
      fast_transform_size_64 = 0;
      fast_transform_select = 0;
      fast_inter_local_illumination_comp = 0;
      fast_inter_adaptive_fullpel_mv = 0;
      break;
    case SpeedMode::kFast:
      bipred_refinement_iterations = 1;
      always_evaluate_intra_in_inter = 0;
      default_num_ref_pics = 1;
      max_binary_split_depth = 2;
      fast_transform_select_eval = 1;
      fast_intra_mode_eval_level = 2;
      fast_transform_size_64 = 1;
      fast_transform_select = 1;
      fast_inter_local_illumination_comp = 1;
      fast_inter_adaptive_fullpel_mv = 1;
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
      inter_search_range_uni_max = 256;
      inter_search_range_uni_min = 96;
      bipred_refinement_iterations = 1;
      always_evaluate_intra_in_inter = 0;
      default_num_ref_pics = 2;
      max_binary_split_depth = 0;
      fast_transform_select_eval = 1;
      fast_intra_mode_eval_level = 1;
      fast_transform_size_64 = 0;
      fast_transform_select = 0;
      fast_inter_local_illumination_comp = 0;
      fast_inter_adaptive_fullpel_mv = 0;
      fast_merge_eval = 0;
      fast_inter_pred_bits = 1;
      rdo_quant_2x2 = 0;
      smooth_lambda_scaling = 0;
      adaptive_qp = 0;
      structural_ssd = 0;
      source_padding = 1;
      break;
    case RestrictedMode::kModeB:
      inter_search_range_uni_max = 256;
      inter_search_range_uni_min = 96;
      bipred_refinement_iterations = 1;
      always_evaluate_intra_in_inter = 0;
      default_num_ref_pics = 2;
      max_binary_split_depth = 2;
      fast_transform_select_eval = 1;
      fast_intra_mode_eval_level = 2;
      fast_transform_size_64 = 0;
      fast_transform_select = 0;
      fast_inter_local_illumination_comp = 0;
      fast_inter_adaptive_fullpel_mv = 0;
      fast_merge_eval = 1;
      fast_quad_split_based_on_binary_split = 2;
      eval_prev_mv_search_result = 0;
      fast_inter_pred_bits = 1;
      rdo_quant_2x2 = 0;
      smooth_lambda_scaling = 0;
      adaptive_qp = 0;
      structural_ssd = 0;
      source_padding = 1;
      chroma_qp_offset_u = 1;
      chroma_qp_offset_v = 1;
      break;
    case RestrictedMode::kModeC:
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
      source_padding = 1;
      chroma_qp_offset_table = 0;
      break;
    default:
      assert(0);
      break;
  }
}

void EncoderSettings::ParseExplicitSettings(std::string explicit_settings) {
  std::string setting;
  std::stringstream stream(explicit_settings);
  while (stream >> setting) {
    if (setting == "inter_search_range_uni_max") {
      stream >> inter_search_range_uni_max;
    } else if (setting == "inter_search_range_uni_min") {
      stream >> inter_search_range_uni_min;
    } else if (setting == "bipred_refinement_iterations") {
      stream >> bipred_refinement_iterations;
    } else if (setting == "always_evaluate_intra_in_inter") {
      stream >> always_evaluate_intra_in_inter;
    } else if (setting == "default_num_ref_pics") {
      stream >> default_num_ref_pics;
    } else if (setting == "max_binary_split_depth") {
      stream >> max_binary_split_depth;
    } else if (setting == "fast_transform_select_eval") {
      stream >> fast_transform_select_eval;
    } else if (setting == "fast_intra_mode_eval_level") {
      stream >> fast_intra_mode_eval_level;
    } else if (setting == "fast_transform_size_64") {
      stream >> fast_transform_size_64;
    } else if (setting == "fast_transform_select") {
      stream >> fast_transform_select;
    } else if (setting == "fast_inter_local_illumination_comp") {
      stream >> fast_inter_local_illumination_comp;
    } else if (setting == "fast_inter_adaptive_fullpel_mv") {
      stream >> fast_inter_adaptive_fullpel_mv;
    } else if (setting == "fast_merge_eval") {
      stream >> fast_merge_eval;
    } else if (setting == "fast_quad_split_based_on_binary_split") {
      stream >> fast_quad_split_based_on_binary_split;
    } else if (setting == "eval_prev_mv_search_result") {
      stream >> eval_prev_mv_search_result;
    } else if (setting == "fast_inter_pred_bits") {
      stream >> fast_inter_pred_bits;
    } else if (setting == "rdo_quant_2x2") {
      stream >> rdo_quant_2x2;
    } else if (setting == "intra_qp_offset") {
      stream >> intra_qp_offset;
    } else if (setting == "smooth_lambda_scaling") {
      stream >> smooth_lambda_scaling;
    } else if (setting == "adaptive_qp") {
      stream >> adaptive_qp;
    } else if (setting == "aqp_strength") {
      stream >> aqp_strength;
    } else if (setting == "structural_ssd") {
      stream >> structural_ssd;
    } else if (setting == "structural_strength") {
      stream >> structural_strength;
    } else if (setting == "encapsulation_mode") {
      stream >> encapsulation_mode;
    } else if (setting == "leading_pictures") {
      stream >> leading_pictures;
    } else if (setting == "source_padding") {
      stream >> source_padding;
    } else if (setting == "lambda_scale_a") {
      stream >> lambda_scale_a;
    } else if (setting == "lambda_scale_b") {
      stream >> lambda_scale_b;
    }
  }
}

}   // namespace xvc
