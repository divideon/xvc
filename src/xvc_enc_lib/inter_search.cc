/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/inter_search.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "xvc_common_lib/utils.h"

namespace xvc {

using const_mv = const MotionVector;
static const std::array<std::pair<int8_t, int8_t>, 9> kSquareXYHalf = { {
  {0, 0}, {0, -1}, {0, 1}, {-1, 0}, {1, 0}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1}
} };
static const std::array<std::pair<int8_t, int8_t>, 9> kSquareXYQpel = { {
  {0, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {1, 1}
} };

struct InterSearch::SearchState {
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

template<typename TOrig>
class InterSearch::DistortionWrapper {
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

InterSearch::InterSearch(int bitdepth, const YuvPicture &orig_pic,
                         const ReferencePictureLists &ref_pic_list)
  : InterPrediction(bitdepth),
  bitdepth_(bitdepth),
  orig_pic_(orig_pic),
  bipred_orig_buffer_(constants::kMaxBlockSize, constants::kMaxBlockSize),
  bipred_pred_buffer_(constants::kMaxBlockSize, constants::kMaxBlockSize) {
  std::vector<int> l1_mapping;
  ref_pic_list.GetSamePocMappingFor(RefPicList::kL1, &l1_mapping);
  assert(l1_mapping.size() <= same_poc_in_l0_mapping_.size());
  std::copy(l1_mapping.begin(), l1_mapping.end(),
            same_poc_in_l0_mapping_.begin());
}

void InterSearch::Search(CodingUnit *cu, const QP &qp,
                         PicturePredictionType pic_type,
                         const CuWriter &cu_writer,
                         const SyntaxWriter &bitstream_writer,
                         SampleBuffer *pred_buffer) {
  const YuvComponent comp = YuvComponent::kY;
  DataBuffer<const Sample> orig_luma =
    orig_pic_.GetSampleBuffer(comp, cu->GetPosX(comp), cu->GetPosY(comp));
  Sample *pred = pred_buffer->GetDataPtr();
  ptrdiff_t pred_stride = pred_buffer->GetStride();

  cu->SetPredMode(PredictionMode::kInter);
  cu->SetMergeFlag(false);

  CodingUnit::InterState state_l0;
  cu->SetInterDir(InterDir::kL0);
  Distortion cost_l0 = std::numeric_limits<Distortion>::max();
  cost_l0 = SearchRefIdx(cu, qp, pic_type, RefPicList::kL0,
                         cu_writer, bitstream_writer, orig_luma,
                         pred, pred_stride, cost_l0, &state_l0, nullptr);
  if (pic_type == PicturePredictionType::kUni) {
    return;
  }

  CodingUnit::InterState state_bi;
  CodingUnit::InterState state_l1_unique_poc;
  Distortion cost_l1_unique_poc;
  cu->SetInterDir(InterDir::kL1);
  Distortion cost_l1 = std::numeric_limits<Distortion>::max();
  cost_l1 = SearchRefIdx(cu, qp, pic_type, RefPicList::kL1,
                         cu_writer, bitstream_writer, orig_luma,
                         pred, pred_stride, cost_l1, &state_bi,
                         &state_l1_unique_poc, &cost_l1_unique_poc);

  // Prepare initial bi-prediction state with best from both lists
  assert(cu->GetInterDir() == InterDir::kL1);
  cu->SetMv(state_l0.mv[0], RefPicList::kL0);
  cu->SetRefIdx(state_l0.ref_idx[0], RefPicList::kL0);
  cu->SetMvDelta(state_l0.mvd[0], RefPicList::kL0);
  cu->SetMvpIdx(state_l0.mvp_idx[0], RefPicList::kL0);
  InterDir best_uni_dir = cost_l0 <= cost_l1 ? InterDir::kL0 : InterDir::kL1;
  Distortion cost_best_bi = SearchBiIterative(cu, qp, pic_type, cu_writer,
                                              bitstream_writer, best_uni_dir,
                                              pred, pred_stride, &state_bi);

  if (cost_best_bi <= cost_l0 && cost_best_bi <= cost_l1_unique_poc) {
    cu->LoadStateFrom(state_bi);
  } else if (cost_l0 <= cost_l1_unique_poc) {
    cu->LoadStateFrom(state_l0);
  } else {
    cu->LoadStateFrom(state_l1_unique_poc);
  }
}

Distortion
InterSearch::SearchBiIterative(CodingUnit *cu, const QP &qp,
                               PicturePredictionType pic_type,
                               const CuWriter &cu_writer,
                               const SyntaxWriter &bitstream_writer,
                               InterDir best_uni_dir,
                               Sample *pred_buf, ptrdiff_t pred_stride,
                               CodingUnit::InterState *best_state) {
  const YuvComponent comp = YuvComponent::kY;
  DataBuffer<const Sample> orig_luma =
    orig_pic_.GetSampleBuffer(comp, cu->GetPosX(comp), cu->GetPosY(comp));
  int width = cu->GetWidth(comp);
  int height = cu->GetHeight(comp);

  // Start searching the second best list
  RefPicList search_list =
    best_uni_dir == InterDir::kL0 ? RefPicList::kL1 : RefPicList::kL0;

  Distortion cost_best = std::numeric_limits<Distortion>::max();
  for (int iteration = 0; iteration < kMaxIterationsBi; iteration++) {
    // If searching in L1 use original without L0 prediction
    cu->SetInterDir(search_list == RefPicList::kL0 ?
                    InterDir::kL1 : InterDir::kL0);
    MotionCompensation(*cu, comp, bipred_pred_buffer_.GetDataPtr(),
                       bipred_pred_buffer_.GetStride());
    bipred_orig_buffer_.SubtractWeighted(width, height, orig_luma,
                                         bipred_pred_buffer_);
    cu->SetInterDir(InterDir::kBi);

    Distortion prev_best = cost_best;
    cost_best = SearchRefIdx(cu, qp, pic_type, search_list, cu_writer,
                             bitstream_writer, bipred_orig_buffer_,
                             pred_buf, pred_stride, cost_best, best_state);
    if (cost_best == prev_best) {
      break;
    }
    search_list = ReferencePictureLists::Inverse(search_list);
  }
  return cost_best;
}

template<typename TOrig>
Distortion
InterSearch::SearchRefIdx(CodingUnit *cu, const QP &qp,
                          PicturePredictionType pic_type, RefPicList ref_list,
                          const CuWriter &cu_writer,
                          const SyntaxWriter &bitstream_writer,
                          const DataBuffer<TOrig> &orig_buffer,
                          Sample *pred, ptrdiff_t pred_stride,
                          Distortion initial_best_cost,
                          CodingUnit::InterState *best_state,
                          CodingUnit::InterState *best_state_unique,
                          Distortion *out_cost_unique) {
  const int num_ref_idx = cu->GetRefPicLists()->GetNumRefPics(ref_list);
  const uint32_t lambda =
    static_cast<uint32_t>(std::floor(65536.0 * qp.GetLambdaSqrt()));
  const bool bi_pred = cu->GetInterDir() == InterDir::kBi;
  const double weight = bi_pred ? 0.5 : 1;
  const SearchMethod search_method =
    bi_pred ? SearchMethod::FullSearch : SearchMethod::TZSearch;
  Distortion cost_best = initial_best_cost;
  Distortion cost_best_unique = std::numeric_limits<Distortion>::max();
  if (!bi_pred) {
    // Clear out old search for deblocking
    RefPicList other_list = ReferencePictureLists::Inverse(ref_list);
    cu->SetMv(MotionVector(), other_list);
    cu->SetRefIdx(-1, other_list);
  }

  for (int ref_idx = 0; ref_idx < num_ref_idx; ref_idx++) {
    InterPredictorList mvp_list = GetMvPredictors(*cu, ref_list, ref_idx);
    const MotionVector *mv_start = nullptr;
    int mvp_idx;
    if (!bi_pred) {
      const YuvPicture *ref_pic =
        cu->GetRefPicLists()->GetRefPic(ref_list, ref_idx);
      mvp_idx = EvalStartMvp(*cu, qp, mvp_list, *ref_pic, pred, pred_stride);
    } else {
      mv_start = &unipred_best_mv_[static_cast<int>(ref_list)][ref_idx];
      mvp_idx = unipred_best_mvp_idx_[static_cast<int>(ref_list)][ref_idx];
    }
    Distortion dist = 0;
    MotionVector mv_subpel;
    if (!bi_pred && ref_list == RefPicList::kL1 &&
        same_poc_in_l0_mapping_[ref_idx] >= 0) {
      // Encoder speed-up for already searched ref pictures in L0
      int l0_list_idx = static_cast<int>(RefPicList::kL0);
      int l0_ref_idx = same_poc_in_l0_mapping_[ref_idx];
      mv_subpel = unipred_best_mv_[l0_list_idx][l0_ref_idx];
      dist = unipred_best_dist_[l0_list_idx][l0_ref_idx];
      // TODO(Dev) also update previous_fullpel_ to seed search for next CU?
    } else {
      mv_subpel =
        MotionEstimation(*cu, qp, search_method, ref_list, ref_idx, orig_buffer,
                         mvp_list[mvp_idx], mv_start, pred, pred_stride, &dist);
    }
    mvp_idx = EvalFinalMvpIdx(*cu, mvp_list, mv_subpel, mvp_idx);
    if (!bi_pred || kHasMultipleIterationsBi) {
      unipred_best_mv_[static_cast<int>(ref_list)][ref_idx] = mv_subpel;
      unipred_best_mvp_idx_[static_cast<int>(ref_list)][ref_idx] = mvp_idx;
      unipred_best_dist_[static_cast<int>(ref_list)][ref_idx] = dist;
    }
    MotionVector mvd(mv_subpel.x - mvp_list[mvp_idx].x,
                     mv_subpel.y - mvp_list[mvp_idx].y);

    cu->SetRefIdx(ref_idx, ref_list);
    cu->SetMvpIdx(mvp_idx, ref_list);
    cu->SetMvDelta(mvd, ref_list);
    cu->SetMv(mv_subpel, ref_list);
    Bits bits = GetInterPredBits(*cu, cu_writer, bitstream_writer, pic_type);
    Distortion dist_scaled =
      static_cast<Distortion>(std::floor(dist * weight));
    Distortion cost = dist_scaled + ((bits * lambda) >> 16);
    if (cost < cost_best) {
      cost_best = cost;
      cu->SaveStateTo(best_state);
    }
    if (best_state_unique && !bi_pred && ref_list == RefPicList::kL1) {
      bool unique_ref_pic = same_poc_in_l0_mapping_[ref_idx] < 0;
      if (unique_ref_pic && cost < cost_best_unique) {
        cost_best_unique = cost;
        cu->SaveStateTo(best_state_unique);
      }
    }
  }
  cu->LoadStateFrom(*best_state);
  if (out_cost_unique) {
    *out_cost_unique = cost_best_unique;
  }
  return cost_best;
}

template<typename TOrig>
MotionVector
InterSearch::MotionEstimation(const CodingUnit &cu, const QP &qp,
                              SearchMethod search_method,
                              RefPicList ref_list, int ref_idx,
                              const DataBuffer<TOrig> &orig_buffer,
                              const MotionVector &mvp,
                              const MotionVector *bipred_mv_start,
                              Sample *pred, ptrdiff_t pred_stride,
                              Distortion *out_dist) {
  const YuvPicture *ref_pic =
    cu.GetRefPicLists()->GetRefPic(ref_list, ref_idx);
  MotionVector clip_min, clip_max;
  if (!bipred_mv_start) {
    DetermineMinMaxMv(cu, *ref_pic, mvp.x, mvp.y, kSearchRangeUni,
                      &clip_min, &clip_max);
  } else {
    DetermineMinMaxMv(cu, *ref_pic, bipred_mv_start->x, bipred_mv_start->y,
                      kSearchRangeBi, &clip_min, &clip_max);
  }

  MotionVector mv_fullpel;
  if (search_method == SearchMethod::FullSearch) {
    mv_fullpel = FullSearch(cu, qp, mvp, *ref_pic, clip_min, clip_max);
  } else if (search_method == SearchMethod::TZSearch) {
    mv_fullpel =
      TZSearch(cu, qp, mvp, *ref_pic, clip_min, clip_max,
               previous_fullpel_[static_cast<int>(ref_list)][ref_idx]);
    previous_fullpel_[static_cast<int>(ref_list)][ref_idx] = mv_fullpel;
  } else {
    assert(0);
  }
  MotionVector mv_subpel =
    SubpelSearch(cu, qp, *ref_pic, mvp, mv_fullpel, orig_buffer, pred,
                 pred_stride, out_dist);
  return mv_subpel;
}

MotionVector InterSearch::FullSearch(const CodingUnit &cu, const QP &qp,
                                     const MotionVector &mvp,
                                     const YuvPicture &ref_pic,
                                     const MotionVector &mv_min,
                                     const MotionVector &mv_max) {
  const int mv_precision = constants::kMvPrecisionShift;
  uint32_t lambda =
    static_cast<uint32_t>(std::floor(65536.0 * qp.GetLambdaSqrt()));
  MetricType fullpel_metric = GetFullpelMetric(cu);
  DistortionWrapper<int16_t> metric(fullpel_metric, YuvComponent::kY, cu, qp,
                                    bitdepth_, bipred_orig_buffer_, ref_pic);
  Distortion cost_best = std::numeric_limits<Distortion>::max();
  MotionVector mv_best;
  for (int mv_y = mv_min.y; mv_y <= mv_max.y; mv_y++) {
    for (int mv_x = mv_min.x; mv_x <= mv_max.x; mv_x++) {
      Distortion dist = metric.GetDist(mv_x, mv_y);
      Bits bits = ((lambda * GetMvdBits(mvp, mv_x, mv_y, mv_precision)) >> 16);
      Distortion cost = dist + bits;
      if (cost < cost_best) {
        cost_best = cost;
        mv_best.x = mv_x;
        mv_best.y = mv_y;
      }
    }
  }
  return mv_best;
}

MotionVector
InterSearch::TZSearch(const CodingUnit &cu, const QP &qp,
                      const MotionVector &mvp, const YuvPicture &ref_pic,
                      const MotionVector &mv_min, const MotionVector &mv_max,
                      const MotionVector &prev_search) {
  static const int kDiamondSearchThreshold = 3;
  static const int kFullSearchGranularity = 5;
  const YuvComponent comp = YuvComponent::kY;
  MetricType fullpel_metric = GetFullpelMetric(cu);
  auto orig_buffer =
    orig_pic_.GetSampleBuffer(comp, cu.GetPosX(comp), cu.GetPosY(comp));
  DistortionWrapper<Sample> dist_wrap(fullpel_metric, YuvComponent::kY, cu, qp,
                                      bitdepth_, orig_buffer, ref_pic);
  SearchState state(&dist_wrap, mvp, mv_min, mv_max);
  state.mv_precision = constants::kMvPrecisionShift;
  state.lambda =
    static_cast<uint32_t>(std::floor(65536.0 * qp.GetLambdaSqrt()));
  MotionVector fullsearch_min = mv_min;
  MotionVector fullsearch_max = mv_max;

  // Initial cost of predictor at fullpel resolution
  MotionVector mvp_fullpel = mvp;
  ClipMV(cu, ref_pic, &mvp_fullpel.x, &mvp_fullpel.y);
  mvp_fullpel.x >>= constants::kMvPrecisionShift;
  mvp_fullpel.y >>= constants::kMvPrecisionShift;
  state.cost_best = GetCost(&state, mvp_fullpel.x, mvp_fullpel.y);
  state.mv_best = mvp_fullpel;

  // Compare with zero mv
  bool change_min_max = CheckCostBest(&state, 0, 0);
  state.last_range_ = 0;

  // Check MV from previous CU search (can be either same or a different size)
  if (cu.GetDepth() != 0) {
    int prev_subpel_x = prev_search.x << constants::kMvPrecisionShift;
    int prev_subpel_y = prev_search.y << constants::kMvPrecisionShift;
    ClipMV(cu, ref_pic, &prev_subpel_x, &prev_subpel_y);
    MotionVector prev_fullpel(prev_subpel_x >> constants::kMvPrecisionShift,
                              prev_subpel_y >> constants::kMvPrecisionShift);
    change_min_max |= CheckCostBest(&state, prev_fullpel.x, prev_fullpel.y);
    if (change_min_max) {
      int best_subpel_x = state.mv_best.x << constants::kMvPrecisionShift;
      int best_subpel_y = state.mv_best.y << constants::kMvPrecisionShift;
      DetermineMinMaxMv(cu, ref_pic, best_subpel_x, best_subpel_y,
                        kSearchRangeUni, &fullsearch_min, &fullsearch_max);
    }
  }

  // Initial search around mvp
  MotionVector mv_base = state.mv_best;
  int rounds_with_no_match = 0;
  for (int range = 1; range <= kSearchRangeUni; range *= 2) {
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
    for (int range = 1; range <= kSearchRangeUni; range *= 2) {
      FullpelDiamondSearch(&state, mv_start, range);
    }
    if (state.last_range_ == 1) {
      state.last_range_ = 0;
      FullpelNeighborPointSearch(&state);
    }
  }

  return state.mv_best;
}

bool InterSearch::FullpelDiamondSearch(SearchState *state,
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

void InterSearch::FullpelNeighborPointSearch(SearchState *state) {
  const int r = 1;
  MotionVector mv_base = state->mv_best;
  switch (state->last_position) {
    case Up::index + Left::index:
      CheckCost1<Up>(state, mv_base.x, mv_base.y - r, r);
      CheckCost1<Left>(state, mv_base.x - r, mv_base.y, r);
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
      CheckCost2<Up, Left>(state, mv_base.x - r, mv_base.y - r, r);
      CheckCost2<Down, Left>(state, mv_base.x - r, mv_base.y + r, r);
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

template<typename TOrig>
MotionVector
InterSearch::SubpelSearch(const CodingUnit &cu, const QP &qp,
                          const YuvPicture &ref_pic, const MotionVector &mvp,
                          const MotionVector &mv_fullpel,
                          const DataBuffer<TOrig> &orig_buffer,
                          Sample *buffer, ptrdiff_t buffer_stride,
                          Distortion *out_dist) {
  SampleMetric metric = SampleMetric(MetricType::kSATD, qp, bitdepth_);
  uint32_t lambda =
    static_cast<uint32_t>(std::floor(65536.0 * qp.GetLambdaSqrt()));
  MotionVector mv_subpel(mv_fullpel.x << constants::kMvPrecisionShift,
                         mv_fullpel.y << constants::kMvPrecisionShift);
  Distortion best_cost = std::numeric_limits<Distortion>::max();
  int best_idx = 0;

  // Half-pel
  for (int i = 0; i < static_cast<int>(kSquareXYHalf.size()); i++) {
    int mv_x = mv_subpel.x + kSquareXYHalf[i].first * 2;
    int mv_y = mv_subpel.y + kSquareXYHalf[i].second * 2;
    Distortion dist = GetSubpelDistortion(cu, ref_pic, &metric, mv_x, mv_y,
                                          orig_buffer, buffer, buffer_stride);
    Bits bits = ((lambda * GetMvdBits(mvp, mv_x, mv_y, 0))) >> 16;
    Distortion cost = dist + bits;
    if (cost < best_cost) {
      best_cost = cost;
      best_idx = i;
      *out_dist = dist;
    }
  }
  if (best_idx > 0) {
    mv_subpel.x += kSquareXYHalf[best_idx].first * 2;
    mv_subpel.y += kSquareXYHalf[best_idx].second * 2;
  }
  best_idx = 0;

  // Qpel
  for (int i = 1; i < static_cast<int>(kSquareXYQpel.size()); i++) {
    int mv_x = mv_subpel.x + kSquareXYQpel[i].first;
    int mv_y = mv_subpel.y + kSquareXYQpel[i].second;
    Distortion dist = GetSubpelDistortion(cu, ref_pic, &metric, mv_x, mv_y,
                                          orig_buffer, buffer, buffer_stride);
    Bits bits = ((lambda * GetMvdBits(mvp, mv_x, mv_y, 0))) >> 16;
    Distortion cost = dist + bits;
    if (cost < best_cost) {
      best_cost = cost;
      best_idx = i;
      *out_dist = dist;
    }
  }
  if (best_idx > 0) {
    mv_subpel.x += kSquareXYQpel[best_idx].first;
    mv_subpel.y += kSquareXYQpel[best_idx].second;
  }
  return mv_subpel;
}

template<typename TOrig>
Distortion
InterSearch::GetSubpelDistortion(const CodingUnit &cu,
                                 const YuvPicture &ref_pic,
                                 SampleMetric *metric, int mv_x, int mv_y,
                                 const DataBuffer<TOrig> &orig_buffer,
                                 Sample *buf, ptrdiff_t buf_stride) {
  YuvComponent comp = YuvComponent::kY;
  int width = cu.GetWidth(comp);
  int height = cu.GetHeight(comp);
  MotionCompensationMv(cu, comp, ref_pic, mv_x, mv_y, buf, buf_stride);
  return metric->CompareSample(comp, width, height, orig_buffer.GetDataPtr(),
                               orig_buffer.GetStride(), &buf[0], buf_stride);
}

void
InterSearch::DetermineMinMaxMv(const CodingUnit &cu, const YuvPicture &ref_pic,
                               int center_x, int center_y, int search_range,
                               MotionVector *mv_min, MotionVector *mv_max) {
  const int mvscale = constants::kMvPrecisionShift;
  ClipMV(cu, ref_pic, &center_x, &center_y);
  int search_range_qpel = search_range << mvscale;
  int search_min_x = center_x - search_range_qpel;
  int search_min_y = center_y - search_range_qpel;
  int search_max_x = center_x + search_range_qpel;
  int search_max_y = center_y + search_range_qpel;
  ClipMV(cu, ref_pic, &search_min_x, &search_min_y);
  ClipMV(cu, ref_pic, &search_max_x, &search_max_y);
  *mv_min = MotionVector(search_min_x >> mvscale, search_min_y >> mvscale);
  *mv_max = MotionVector(search_max_x >> mvscale, search_max_y >> mvscale);
}

int InterSearch::EvalStartMvp(const CodingUnit &cu, const QP &qp,
                              const InterPredictorList &mvp_list,
                              const YuvPicture &ref_pic, Sample *pred_buf,
                              ptrdiff_t pred_stride) {
  SampleMetric metric(MetricType::kSAD, qp, bitdepth_);
  uint32_t lambda =
    static_cast<uint32_t>(std::floor(65536.0 * qp.GetLambdaSqrt()));
  int best_mvp_idx = 0;
  Cost best_mvp_cost = std::numeric_limits<Cost>::max();
  for (int i = 0; i < static_cast<int>(mvp_list.size()); i++) {
    MotionVector mv = mvp_list[i];
    ClipMV(cu, ref_pic, &mv.x, &mv.y);
    MotionCompensationMv(cu, YuvComponent::kY, ref_pic, mv.x, mv.y,
                         pred_buf, pred_stride);
    Distortion dist = metric.CompareSample(cu, YuvComponent::kY, orig_pic_,
                                           pred_buf, pred_stride);
    Bits bits = GetMvpBits(i, static_cast<int>(mvp_list.size()));
    Cost cost = dist + (static_cast<uint32_t>(bits * lambda + 0.5) >> 16);
    if (cost < best_mvp_cost) {
      best_mvp_cost = cost;
      best_mvp_idx = i;
    }
  }
  return best_mvp_idx;
}

int InterSearch::EvalFinalMvpIdx(const CodingUnit &cu,
                                 const InterPredictorList &mvp_list,
                                 const MotionVector &mv_final,
                                 int mvp_idx_start) {
  int best_mvp_idx = 0;
  Bits best_cost = std::numeric_limits<Bits>::max();
  for (int i = 0; i < static_cast<int>(mvp_list.size()); i++) {
    Bits cost = GetMvpBits(i, static_cast<int>(mvp_list.size())) +
      GetMvdBits(mvp_list[i], mv_final.x, mv_final.y, 0);
    if (cost < best_cost || (cost == best_cost && i == mvp_idx_start)) {
      best_cost = cost;
      best_mvp_idx = i;
    }
  }
  return best_mvp_idx;
}

Distortion InterSearch::GetCost(SearchState *state, int mv_x, int mv_y) {
  const int mv_scale = state->mv_precision;
  Distortion dist = state->dist->GetDist(mv_x, mv_y);
  Bits bits =
    ((state->lambda * GetMvdBits(state->mvp, mv_x, mv_y, mv_scale)) >> 16);
  return dist + bits;
}

bool InterSearch::CheckCostBest(SearchState *state, int mv_x, int mv_y) {
  Distortion cost = GetCost(state, mv_x, mv_y);
  if (cost < state->cost_best) {
    state->cost_best = cost;
    state->mv_best.x = mv_x;
    state->mv_best.y = mv_y;
    return true;
  }
  return false;
}

Bits InterSearch::GetInterPredBits(const CodingUnit &cu,
                                   const CuWriter &cu_writer,
                                   const SyntaxWriter &bitstream_writer,
                                   PicturePredictionType pic_pred_type) {
#if HM_STRICT
  const ReferencePictureLists *ref_pic_list = cu.GetRefPicLists();
  if (cu.GetInterDir() != InterDir::kBi) {
    RefPicList ref_list =
      cu.GetInterDir() == InterDir::kL0 ? RefPicList::kL0 : RefPicList::kL1;
    int num_ref_idx = ref_pic_list->GetNumRefPics(ref_list);
    int bits = pic_pred_type == PicturePredictionType::kUni ? 1 : 3;
    bits += num_ref_idx <= 1 ? 0 : cu.GetRefIdx(ref_list) + 1;
    bits -= num_ref_idx > 1 && cu.GetRefIdx(ref_list) == num_ref_idx - 1;
    bits += GetMvpBits(cu.GetMvpIdx(ref_list),
                       constants::kNumInterMvPredictors);
    bits += GetNumExpGolombBits(cu.GetMvDelta(ref_list).x);
    bits += GetNumExpGolombBits(cu.GetMvDelta(ref_list).y);
    return bits;
  } else {
    int bits = 5;
    for (int i = 0; i < 2; i++) {
      RefPicList ref_list = static_cast<RefPicList>(i);
      int num_ref_idx = ref_pic_list->GetNumRefPics(ref_list);
      bits += num_ref_idx <= 1 ? 0 : cu.GetRefIdx(ref_list) + 1;
      bits -= num_ref_idx > 1 && cu.GetRefIdx(ref_list) == num_ref_idx - 1;
      bits += GetMvpBits(cu.GetMvpIdx(ref_list),
                         constants::kNumInterMvPredictors);
      bits += GetNumExpGolombBits(cu.GetMvDelta(ref_list).x);
      bits += GetNumExpGolombBits(cu.GetMvDelta(ref_list).y);
    }
    return bits;
  }
#else
  RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
  cu_writer.WriteInterPrediction(cu, YuvComponent::kY, rdo_writer);
  return rdo_writer.GetNumWrittenBits();
#endif
}

Bits InterSearch::GetMvpBits(int mvp_idx, int num_mvp) {
  if (num_mvp == 1) {
    return 0;
  }
  if (!mvp_idx) {
    return 1;
  }
  assert(num_mvp == 2);
  return 1;
}

Bits InterSearch::GetMvdBits(const MotionVector &mvp, int mv_x, int mv_y,
                             int mv_scale) {
  int mvd_x = (mv_x << mv_scale) - mvp.x;
  int mvd_y = (mv_y << mv_scale) - mvp.y;
  return GetNumExpGolombBits(mvd_x) + GetNumExpGolombBits(mvd_y);
}

Bits InterSearch::GetNumExpGolombBits(int mvd) {
  int length = 1;
  Bits mvd_unsigned = mvd <= 0 ? (-mvd << 1) + 1 : (mvd << 1) + 0;
  while (mvd_unsigned != 1) {
    mvd_unsigned >>= 1;
    length += 2;
  }
  return length;
}

template<class Dir>
inline bool IsInside(int mv_x, int mv_y, const_mv *mv_min, const_mv *mv_max) {
  return false;
}
template<>
inline bool IsInside<InterSearch::Up>(int mv_x, int mv_y,
                                      const_mv *mv_min, const_mv *mv_max) {
  return mv_y >= mv_min->y;
}
template<>
inline bool IsInside<InterSearch::Down>(int mv_x, int mv_y,
                                        const_mv *mv_min, const_mv *mv_max) {
  return mv_y <= mv_max->y;
}
template<>
inline bool IsInside<InterSearch::Left>(int mv_x, int mv_y,
                                        const_mv *mv_min, const_mv *mv_max) {
  return mv_x >= mv_min->x;
}
template<>
inline bool IsInside<InterSearch::Right>(int mv_x, int mv_y,
                                         const_mv *mv_min, const_mv *mv_max) {
  return mv_x <= mv_max->x;
}

template<class Dir>
bool InterSearch::CheckCost1(SearchState *state, int mv_x, int mv_y,
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
bool InterSearch::CheckCost2(SearchState *state, int mv_x, int mv_y,
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

MetricType InterSearch::GetFullpelMetric(const CodingUnit & cu) {
  return cu.GetWidth(YuvComponent::kY) > 8 ?
    MetricType::kSADFast : MetricType::kSAD;
}

}   // namespace xvc
