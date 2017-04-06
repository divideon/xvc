/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_LIB_SPEED_SETTINGS_H_
#define XVC_ENC_LIB_SPEED_SETTINGS_H_

#include "xvc_common_lib/restrictions.h"

namespace xvc {

enum struct SpeedMode {
  kPlacebo = 0,
  kSlow = 1,
  kTotalNumber = 2,
};

struct SpeedSettings {
  // Initialize based on speed mode setting
  void Initialize(SpeedMode speed_mode) {
    switch (speed_mode) {
      case SpeedMode::kPlacebo:
        eval_prev_mv_search_result = 1;
        fast_intra_mode_eval_level = 0;
        bipred_refinement_iterations = 4;
        fast_merge_rdo = 0;
        break;
      case SpeedMode::kSlow:
        eval_prev_mv_search_result = 1;
        fast_intra_mode_eval_level = 1;
        bipred_refinement_iterations = 1;
        fast_merge_rdo = 1;
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
        fast_merge_rdo = 1;
        break;
      case RestrictedMode::kModeB:
        eval_prev_mv_search_result = 0;
        fast_intra_mode_eval_level = 2;
        bipred_refinement_iterations = 1;
        fast_merge_rdo = 1;
        break;
      default:
        assert(0);
        break;
    }
  }

  int eval_prev_mv_search_result = -1;
  int fast_intra_mode_eval_level = -1;
  int bipred_refinement_iterations = -1;
  int fast_merge_rdo = -1;
};


}   // namespace xvc

#endif  // XVC_ENC_LIB_SPEED_SETTINGS_H_
