/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_LIB_ENCODER_SETTINGS_H_
#define XVC_ENC_LIB_ENCODER_SETTINGS_H_

#include "xvc_common_lib/restrictions.h"

namespace xvc {

enum struct SpeedMode {
  kPlacebo = 0,
  kSlow = 1,
  kTotalNumber = 2,
};

struct EncoderSettings {
  // Initialize based on speed mode setting
  void Initialize(SpeedMode speed_mode) {
    switch (speed_mode) {
      case SpeedMode::kPlacebo:
        eval_prev_mv_search_result = 1;
        fast_intra_mode_eval_level = 0;
        bipred_refinement_iterations = 4;
        always_evaluate_intra_in_inter = 1;
        smooth_lambda_scaling = 1;
        default_num_ref_pics = 3;
        break;
      case SpeedMode::kSlow:
        eval_prev_mv_search_result = 1;
        fast_intra_mode_eval_level = 1;
        bipred_refinement_iterations = 1;
        always_evaluate_intra_in_inter = 0;
        smooth_lambda_scaling = 1;
        default_num_ref_pics = 2;
        break;
      default:
        assert(0);
        break;
    }
  }

  // Initialize based on restricted mode setting
  void Initialize(RestrictedMode restricted_mode) {
    switch (restricted_mode) {
      case RestrictedMode::kModeA:
        eval_prev_mv_search_result = 1;
        fast_intra_mode_eval_level = 1;
        bipred_refinement_iterations = 1;
        always_evaluate_intra_in_inter = 0;
        smooth_lambda_scaling = 0;
        default_num_ref_pics = 2;
        break;
      case RestrictedMode::kModeB:
        eval_prev_mv_search_result = 0;
        fast_intra_mode_eval_level = 2;
        bipred_refinement_iterations = 1;
        always_evaluate_intra_in_inter = 0;
        smooth_lambda_scaling = 0;
        default_num_ref_pics = 2;
        break;
      default:
        assert(0);
        break;
    }
  }

  static const bool fast_quad_split_from_binary_split = true;
  static const bool fast_mode_selection_for_cached_cu = true;
  static const bool skip_mode_decision_for_identical_cu = false;

  int eval_prev_mv_search_result = -1;
  int fast_intra_mode_eval_level = -1;
  int bipred_refinement_iterations = -1;
  int always_evaluate_intra_in_inter = -1;
  int smooth_lambda_scaling = -1;
  int default_num_ref_pics = -1;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_ENCODER_SETTINGS_H_
