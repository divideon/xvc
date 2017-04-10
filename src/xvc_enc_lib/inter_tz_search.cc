/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/inter_tz_search.h"

#include <cmath>

#include "xvc_common_lib/restrictions.h"
#include "xvc_enc_lib/inter_search.h"

namespace xvc {

using const_mv = const MotionVector;

template<typename TOrig>
class TZSearch::DistortionWrapper {
public:
  DistortionWrapper(MetricType metric, YuvComponent comp, const CodingUnit &cu,
                    const QP &qp, int bitdepth,
                    const DataBuffer<const TOrig> &src1, const YuvPicture &src2)
    : comp_(comp),
    width_(cu.GetWidth(comp)),
    height_(cu.GetHeight(comp)),
    src1_(src1.GetDataPtr()),
    stride1_(src1.GetStride()),
    src2_(src2.GetSamplePtr(comp, cu.GetPosX(comp), cu.GetPosY(comp))),
    stride2_(src2.GetStride(comp)),
    metric_(metric, qp, bitdepth) {
  }

  Distortion GetDist(int mv_x, int mv_y) {
    const Sample *src2_ptr = src2_ + mv_y * stride2_ + mv_x;
    return metric_.CompareSample(comp_, width_, height_, src1_, stride1_,
                                 src2_ptr, stride2_);
  }

private:
  YuvComponent comp_;
  int width_;
  int height_;
  const TOrig *src1_;
  const ptrdiff_t stride1_;
  const Sample *src2_;
  const ptrdiff_t stride2_;
  SampleMetric metric_;
};

struct TZSearch::SearchState {
  SearchState(DistortionWrapper<Sample> *dist_wrap, const MotionVector &mvpred,
              const MotionVector &mvmin, const MotionVector &mvmax)
    : dist(dist_wrap), mvp(mvpred), mv_min(mvmin), mv_max(mvmax) {
  }
  DistortionWrapper<Sample> *dist;
  const MotionVector &mvp;
  const MotionVector &mv_min;
  const MotionVector &mv_max;
  MotionVector mv_best = MotionVector(0, 0);
  Distortion cost_best = 0;
  int last_position = 0;
  int last_range_ = 0;
  int mv_precision = 0;
  uint32_t lambda = 0;
};

MotionVector
TZSearch::Search(const CodingUnit &cu, const QP &qp, MetricType metric,
                 const MotionVector &mvp, const YuvPicture &ref_pic,
                 const MotionVector &mv_min, const MotionVector &mv_max,
                 const MotionVector &prev_search) {
  static const int kDiamondSearchThreshold = 3;
  static const int kFullSearchGranularity = 5;
  const YuvComponent comp = YuvComponent::kY;
  auto orig_buffer =
    orig_pic_.GetSampleBuffer(comp, cu.GetPosX(comp), cu.GetPosY(comp));
  DistortionWrapper<Sample> dist_wrap(metric, YuvComponent::kY, cu, qp,
                                      bitdepth_, orig_buffer, ref_pic);
  SearchState state(&dist_wrap, mvp, mv_min, mv_max);
  state.mv_precision = constants::kMvPrecisionShift;
  state.lambda =
    static_cast<uint32_t>(std::floor(65536.0 * qp.GetLambdaSqrt()));
  MotionVector fullsearch_min = mv_min;
  MotionVector fullsearch_max = mv_max;

  // Initial cost of predictor at fullpel resolution
  MotionVector mvp_fullpel = mvp;
  inter_pred_.ClipMV(cu, ref_pic, &mvp_fullpel.x, &mvp_fullpel.y);
  mvp_fullpel.x >>= constants::kMvPrecisionShift;
  mvp_fullpel.y >>= constants::kMvPrecisionShift;
  state.cost_best = GetCost(&state, mvp_fullpel.x, mvp_fullpel.y);
  state.mv_best = mvp_fullpel;

  // Compare with zero mv
  bool change_min_max = CheckCostBest(&state, 0, 0);
  state.last_range_ = 0;

  // Check MV from previous CU search (can be either same or a different size)
  if (cu.GetDepth() != 0 &&
      speed_settings_.eval_prev_mv_search_result) {
    int prev_subpel_x = prev_search.x * (1 << constants::kMvPrecisionShift);
    int prev_subpel_y = prev_search.y * (1 << constants::kMvPrecisionShift);
    inter_pred_.ClipMV(cu, ref_pic, &prev_subpel_x, &prev_subpel_y);
    MotionVector prev_fullpel(prev_subpel_x >> constants::kMvPrecisionShift,
                              prev_subpel_y >> constants::kMvPrecisionShift);
    change_min_max |= CheckCostBest(&state, prev_fullpel.x, prev_fullpel.y);
    if (change_min_max) {
      int best_subpel_x = state.mv_best.x * (1 << constants::kMvPrecisionShift);
      int best_subpel_y = state.mv_best.y * (1 << constants::kMvPrecisionShift);
      inter_pred_.DetermineMinMaxMv(cu, ref_pic, best_subpel_x, best_subpel_y,
                                    search_range_, &fullsearch_min,
                                    &fullsearch_max);
    }
  }

  // Initial search around mvp
  MotionVector mv_base = state.mv_best;
  int rounds_with_no_match = 0;
  for (int range = 1; range <= search_range_; range *= 2) {
    bool changed = FullpelDiamondSearch(&state, mv_base, range);
    if (changed) {
      rounds_with_no_match = 0;
    } else if (++rounds_with_no_match >= kDiamondSearchThreshold) {
      break;
    }
  }
  if (state.last_range_ == 1) {
    state.last_range_ = 0;
    FullpelNeighborPointSearch(&state);
  }

  // Full search in search window
  if (state.last_range_ > kFullSearchGranularity) {
    state.last_range_ = kFullSearchGranularity;
    const int step_size = kFullSearchGranularity;
    for (int y = fullsearch_min.y; y <= fullsearch_max.y; y += step_size) {
      for (int x = fullsearch_min.x; x <= fullsearch_max.x; x += step_size) {
        CheckCostBest(&state, x, y);
      }
    }
  }

  // Iterative refinement of start position
  while (state.last_range_ > 0) {
    MotionVector mv_start = state.mv_best;
    state.last_range_ = 0;
    for (int range = 1; range <= search_range_; range *= 2) {
      FullpelDiamondSearch(&state, mv_start, range);
    }
    if (state.last_range_ == 1) {
      state.last_range_ = 0;
      FullpelNeighborPointSearch(&state);
    }
  }

  return state.mv_best;
}

bool TZSearch::FullpelDiamondSearch(SearchState *state,
                                    const MotionVector &mv_base, int range) {
  bool mod = false;
  if (range == 1) {
    mod |= CheckCost1<Up>(state, mv_base.x, mv_base.y - range, range);
    mod |= CheckCost1<Left>(state, mv_base.x - range, mv_base.y, range);
    mod |= CheckCost1<Right>(state, mv_base.x + range, mv_base.y, range);
    mod |= CheckCost1<Down>(state, mv_base.x, mv_base.y + range, range);
  } else if (range <= 8) {
    int r2 = range >> 1;
    mod |= CheckCost1<Up>(state, mv_base.x, mv_base.y - range, range);
    mod |= CheckCost2<Up, Left>(state, mv_base.x - r2, mv_base.y - r2, r2);
    mod |= CheckCost2<Up, Right>(state, mv_base.x + r2, mv_base.y - r2, r2);
    mod |= CheckCost1<Left>(state, mv_base.x - range, mv_base.y, range);
    mod |= CheckCost1<Right>(state, mv_base.x + range, mv_base.y, range);
    mod |= CheckCost2<Down, Left>(state, mv_base.x - r2, mv_base.y + r2, r2);
    mod |= CheckCost2<Down, Right>(state, mv_base.x + r2, mv_base.y + r2, r2);
    mod |= CheckCost1<Down>(state, mv_base.x, mv_base.y + range, range);
  } else {
    mod |= CheckCost1<Up>(state, mv_base.x, mv_base.y - range, range);
    mod |= CheckCost1<Left>(state, mv_base.x - range, mv_base.y, range);
    mod |= CheckCost1<Right>(state, mv_base.x + range, mv_base.y, range);
    mod |= CheckCost1<Down>(state, mv_base.x, mv_base.y + range, range);
    for (int i = 1; i < 4; i++) {
      int range14 = i * (range >> 2);
      int range34 = range - range14;
      mod |= CheckCost2<Up, Left>(state, mv_base.x - range14,
                                  mv_base.y - range34, range);
      mod |= CheckCost2<Up, Right>(state, mv_base.x + range14,
                                   mv_base.y - range34, range);
      mod |= CheckCost2<Down, Left>(state, mv_base.x - range14,
                                    mv_base.y + range34, range);
      mod |= CheckCost2<Down, Right>(state, mv_base.x + range14,
                                     mv_base.y + range34, range);
    }
  }
  return mod;
}

void TZSearch::FullpelNeighborPointSearch(SearchState *state) {
  const int r = 1;
  MotionVector mv_base = state->mv_best;
  switch (state->last_position) {
    case Up::index + Left::index:
      CheckCost1<Left>(state, mv_base.x - r, mv_base.y, r);
      CheckCost1<Up>(state, mv_base.x, mv_base.y - r, r);
      break;

    case Up::index:
      CheckCost2<Up, Left>(state, mv_base.x - r, mv_base.y - r, r);
      CheckCost2<Up, Right>(state, mv_base.x + r, mv_base.y - r, r);
      break;

    case Up::index + Right::index:
      CheckCost1<Up>(state, mv_base.x, mv_base.y - r, r);
      CheckCost1<Right>(state, mv_base.x + r, mv_base.y, r);
      break;

    case Left::index:
      CheckCost2<Down, Left>(state, mv_base.x - r, mv_base.y + r, r);
      CheckCost2<Up, Left>(state, mv_base.x - r, mv_base.y - r, r);
      break;

    case Right::index:
      CheckCost2<Up, Right>(state, mv_base.x + r, mv_base.y - r, r);
      CheckCost2<Down, Right>(state, mv_base.x + r, mv_base.y + r, r);
      break;

    case Down::index + Left::index:
      CheckCost1<Left>(state, mv_base.x - r, mv_base.y, r);
      CheckCost1<Down>(state, mv_base.x, mv_base.y + r, r);
      break;

    case Down::index:
      CheckCost2<Down, Left>(state, mv_base.x - r, mv_base.y + r, r);
      CheckCost2<Down, Right>(state, mv_base.x + r, mv_base.y + r, r);
      break;

    case Down::index + Right::index:
      CheckCost1<Right>(state, mv_base.x + r, mv_base.y, r);
      CheckCost1<Down>(state, mv_base.x, mv_base.y + r, r);
      break;

    default:
      break;
  }
}

Distortion TZSearch::GetCost(SearchState *state, int mv_x, int mv_y) {
  const int mv_scale = state->mv_precision;
  Distortion dist = state->dist->GetDist(mv_x, mv_y);
  Bits mvd = InterSearch::GetMvdBits(state->mvp, mv_x, mv_y, mv_scale);
  Bits bits = ((state->lambda * mvd) >> 16);
  return dist + bits;
}

bool TZSearch::CheckCostBest(SearchState *state, int mv_x, int mv_y) {
  Distortion cost = GetCost(state, mv_x, mv_y);
  if (cost < state->cost_best) {
    state->cost_best = cost;
    state->mv_best.x = mv_x;
    state->mv_best.y = mv_y;
    return true;
  }
  return false;
}

template<class Dir>
inline bool IsInside(int mv_x, int mv_y, const_mv *mv_min, const_mv *mv_max) {
  return false;
}
template<>
inline bool IsInside<TZSearch::Up>(int mv_x, int mv_y,
                                   const_mv *mv_min, const_mv *mv_max) {
  return mv_y >= mv_min->y;
}
template<>
inline bool IsInside<TZSearch::Down>(int mv_x, int mv_y,
                                     const_mv *mv_min, const_mv *mv_max) {
  return mv_y <= mv_max->y;
}
template<>
inline bool IsInside<TZSearch::Left>(int mv_x, int mv_y,
                                     const_mv *mv_min, const_mv *mv_max) {
  return mv_x >= mv_min->x;
}
template<>
inline bool IsInside<TZSearch::Right>(int mv_x, int mv_y,
                                      const_mv *mv_min, const_mv *mv_max) {
  return mv_x <= mv_max->x;
}

template<class Dir>
bool TZSearch::CheckCost1(SearchState *state, int mv_x, int mv_y,
                          int range) {
  if (!IsInside<Dir>(mv_x, mv_y, &state->mv_min, &state->mv_max)) {
    return false;
  }
  if (!CheckCostBest(state, mv_x, mv_y)) {
    return false;
  }
  state->last_position = Dir::index;
  state->last_range_ = range;
  return true;
}

template<class Dir1, class Dir2>
bool TZSearch::CheckCost2(SearchState *state, int mv_x, int mv_y,
                          int range) {
  if (!IsInside<Dir1>(mv_x, mv_y, &state->mv_min, &state->mv_max) ||
      !IsInside<Dir2>(mv_x, mv_y, &state->mv_min, &state->mv_max)) {
    return false;
  }
  if (!CheckCostBest(state, mv_x, mv_y)) {
    return false;
  }
  state->last_position = Dir1::index + Dir2::index;
  state->last_range_ = range;
  return true;
}

}   // namespace xvc
