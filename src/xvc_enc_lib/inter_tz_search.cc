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

#include "xvc_enc_lib/inter_tz_search.h"

#include <cmath>
#include <limits>

#include "xvc_common_lib/restrictions.h"
#include "xvc_enc_lib/inter_search.h"

namespace xvc {

template<typename TOrig>
class TzSearch::DistortionWrapper {
public:
  DistortionWrapper(MetricType metric, YuvComponent comp, const CodingUnit &cu,
                    const Qp &qp, int bitdepth,
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

struct TzSearch::SearchState {
  SearchState(DistortionWrapper<Sample> *dist_wrap, const MotionVector &mvpred,
              const MvFullpel &mvmin, const MvFullpel &mvmax)
    : dist(dist_wrap), mvp(mvpred), mv_min(mvmin), mv_max(mvmax) {
  }
  DistortionWrapper<Sample> *dist;
  const MotionVector &mvp;
  const MvFullpel &mv_min;
  const MvFullpel &mv_max;
  MvFullpel mv_best = MvFullpel(0, 0);
  Distortion cost_best = std::numeric_limits<Distortion>::max();
  int last_position = 0;
  int last_range_ = 0;
  int mvd_downshift = 0;
  uint32_t lambda = 0;
};

MvFullpel
TzSearch::Search(const CodingUnit &cu, const Qp &qp, MetricType metric,
                 const MotionVector &mvp, const YuvPicture &ref_pic,
                 const MvFullpel &mv_min, const MvFullpel &mv_max,
                 const MvFullpel &prev_search) {
  static const int kDiamondSearchThreshold = 3;
  static const int kFullSearchGranularity = 5;
  const YuvComponent comp = YuvComponent::kY;
  auto orig_buffer =
    orig_pic_.GetSampleBuffer(comp, cu.GetPosX(comp), cu.GetPosY(comp));
  DistortionWrapper<Sample> dist_wrap(metric, YuvComponent::kY, cu, qp,
                                      bitdepth_, orig_buffer, ref_pic);
  SearchState state(&dist_wrap, mvp, mv_min, mv_max);
  state.mvd_downshift = cu.GetFullpelMv() ? MvDelta::kPrecisionShift : 0;
  state.lambda =
    static_cast<uint32_t>(std::floor(65536.0 * qp.GetLambdaSqrt()));
  MvFullpel fullsearch_min = mv_min;
  MvFullpel fullsearch_max = mv_max;

  // Initial cost of predictor at fullpel resolution
  MotionVector mvp_clip = mvp;
  inter_pred_.ClipMv(cu, ref_pic, &mvp_clip);
  MvFullpel mvp_fullpel(mvp_clip);
  CheckCostBest(&state, mvp_fullpel.x, mvp_fullpel.y);

  // Compare with zero mv
  bool change_min_max = false;
  if (state.mv_best.x != 0 || state.mv_best.y != 0) {
    change_min_max = CheckCostBest(&state, 0, 0);
  }
  state.last_range_ = 0;

  // Check MV from previous CU search (can be either same or a different size)
  if (cu.GetDepth() != 0 &&
      encoder_settings_.eval_prev_mv_search_result) {
    MotionVector prev_clip = MotionVector(prev_search);
    inter_pred_.ClipMv(cu, ref_pic, &prev_clip);
    MvFullpel prev_fullpel = prev_clip;
    change_min_max |= CheckCostBest(&state, prev_fullpel.x, prev_fullpel.y);
    if (change_min_max) {
      MotionVector best_subpel = MotionVector(state.mv_best);
      inter_pred_.DetermineMinMaxMv(cu, ref_pic, best_subpel, search_range_,
                                    &fullsearch_min, &fullsearch_max);
    }
  }

  // Initial search around mvp
  MvFullpel mv_base = state.mv_best;
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
    MvFullpel mv_start = state.mv_best;
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

bool TzSearch::FullpelDiamondSearch(SearchState *state,
                                    const MvFullpel &mv_base, int range) {
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

void TzSearch::FullpelNeighborPointSearch(SearchState *state) {
  const int r = 1;
  MvFullpel mv_base = state->mv_best;
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

bool TzSearch::CheckCostBest(SearchState *state, int mv_x, int mv_y) {
  Distortion dist = state->dist->GetDist(mv_x, mv_y);
  if (dist >= state->cost_best) {
    return false;
  }
  Bits bits = InterSearch::GetMvdBitsFullpel(state->mvp, mv_x, mv_y,
                                             state->mvd_downshift);
  Distortion cost = dist + ((state->lambda * bits) >> 16);
  if (cost < state->cost_best) {
    state->cost_best = cost;
    state->mv_best.x = mv_x;
    state->mv_best.y = mv_y;
    return true;
  }
  return false;
}

template<class Dir>
bool
TzSearch::IsInside(int mv_x, int mv_y, const_mv *mv_min, const_mv *mv_max) {
  return false;
}
template<>
bool TzSearch::IsInside<TzSearch::Up>(int mv_x, int mv_y,
                                      const_mv *mv_min, const_mv *mv_max) {
  return mv_y >= mv_min->y;
}
template<>
bool TzSearch::IsInside<TzSearch::Down>(int mv_x, int mv_y,
                                        const_mv *mv_min, const_mv *mv_max) {
  return mv_y <= mv_max->y;
}
template<>
bool TzSearch::IsInside<TzSearch::Left>(int mv_x, int mv_y,
                                        const_mv *mv_min, const_mv *mv_max) {
  return mv_x >= mv_min->x;
}
template<>
bool TzSearch::IsInside<TzSearch::Right>(int mv_x, int mv_y,
                                         const_mv *mv_min, const_mv *mv_max) {
  return mv_x <= mv_max->x;
}

template<class Dir>
bool TzSearch::CheckCost1(SearchState *state, int mv_x, int mv_y,
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
bool TzSearch::CheckCost2(SearchState *state, int mv_x, int mv_y,
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
