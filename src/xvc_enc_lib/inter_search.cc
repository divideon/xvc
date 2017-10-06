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

#include "xvc_enc_lib/inter_search.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"
#include "xvc_enc_lib/inter_tz_search.h"
#include "xvc_enc_lib/cu_writer.h"

namespace xvc {

static const std::array<std::array<int8_t, 2>, 9> kSquareXYHalf = { {
  {0, 0}, {0, -1}, {0, 1}, {-1, 0}, {1, 0}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1}
} };
static const std::array<std::array<int8_t, 2>, 9> kSquareXYQpel = { {
  {0, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1}, {-1, 0}, {1, 0}, {-1, 1}, {1, 1}
} };

InterSearch::InterSearch(const SimdFunctions &simd, const PictureData &pic_data,
                         const YuvPicture &orig_pic,
                         const ReferencePictureLists &ref_pic_list,
                         const EncoderSettings &encoder_settings)
  : InterPrediction(simd.inter_prediction, pic_data.GetBitdepth()),
  bitdepth_(pic_data.GetBitdepth()),
  max_components_(pic_data.GetMaxNumComponents()),
  orig_pic_(orig_pic),
  encoder_settings_(encoder_settings),
  cu_writer_(pic_data, nullptr),
  bipred_orig_buffer_(constants::kMaxBlockSize, constants::kMaxBlockSize),
  bipred_pred_buffer_(constants::kMaxBlockSize, constants::kMaxBlockSize) {
  std::vector<int> l1_mapping;
  ref_pic_list.GetSamePocMappingFor(RefPicList::kL1, &l1_mapping);
  assert(l1_mapping.size() <= same_poc_in_l0_mapping_.size());
  std::copy(l1_mapping.begin(), l1_mapping.end(),
            same_poc_in_l0_mapping_.begin());
}

Distortion
InterSearch::CompressInter(CodingUnit *cu, const Qp &qp,
                           const SyntaxWriter &bitstream_writer,
                           TransformEncoder *encoder, YuvPicture *rec_pic) {
  bool uni_pred_only = cu->GetPicType() == PicturePredictionType::kUni;
  SampleBuffer &pred_buffer = encoder->GetPredBuffer();
  SearchMotion(cu, qp, uni_pred_only, bitstream_writer, &pred_buffer);
  return CompressAndEvalCbf(cu, qp, bitstream_writer, encoder, rec_pic);
}

Distortion
InterSearch::CompressInterFast(CodingUnit *cu, YuvComponent comp, const Qp &qp,
                               const SyntaxWriter &bitstream_writer,
                               TransformEncoder *encoder, YuvPicture *rec_pic) {
  if (!cu->GetCbf(comp)) {
    // Write prediction directly to reconstruction
    SampleBuffer reco =
      rec_pic->GetSampleBuffer(comp, cu->GetPosX(comp), cu->GetPosY(comp));
    MotionCompensation(*cu, comp, reco.GetDataPtr(), reco.GetStride());
    MetricType m = encoder_settings_.structural_ssd > 0 &&
      comp == YuvComponent::kY ? MetricType::kStructuralSsd : MetricType::kSsd;
    SampleMetric metric(m, qp, rec_pic->GetBitdepth());
    return metric.CompareSample(*cu, comp, orig_pic_, reco);
  } else {
    SampleBuffer &pred = encoder->GetPredBuffer();
    MotionCompensation(*cu, comp, pred.GetDataPtr(), pred.GetStride());
    return encoder->CompressAndEvalTransform(cu, comp, qp, bitstream_writer,
                                             orig_pic_, &cu_writer_, rec_pic);
  }
}

Distortion
InterSearch::CompressMergeCand(CodingUnit *cu, const Qp &qp,
                               const SyntaxWriter &bitstream_writer,
                               const InterMergeCandidateList &merge_list,
                               int merge_idx, bool force_skip,
                               TransformEncoder *encoder, YuvPicture *rec_pic) {
  cu->SetPredMode(PredictionMode::kInter);
  cu->SetMergeFlag(true);
  cu->SetSkipFlag(!force_skip ? false : true);
  cu->SetMergeIdx(merge_idx);
  ApplyMerge(cu, merge_list[merge_idx]);
  Distortion dist;
  if (!force_skip) {
    dist = CompressAndEvalCbf(cu, qp, bitstream_writer, encoder, rec_pic);
  } else {
    dist = CompressSkipOnly(cu, qp, bitstream_writer, encoder, rec_pic);
  }
  return dist;
}

int
InterSearch::SearchMergeCandidates(CodingUnit *cu, const Qp &qp,
                                   const SyntaxWriter & bitstream_writer,
                                   const InterMergeCandidateList &merge_list,
                                   TransformEncoder *encoder,
                                   MergeCandLookup *out_cand_list) {
  constexpr int max_merge_cand = constants::kNumInterMergeCandidates;
  SampleMetric metric(MetricType::kSatd, qp, bitdepth_);
  SampleBuffer pred_buffer = encoder->GetPredBuffer();
  std::array<std::pair<int, double>, max_merge_cand> cand_cost;
  for (int merge_idx = 0; merge_idx < max_merge_cand; merge_idx++) {
    ApplyMerge(cu, merge_list[merge_idx]);
    MotionCompensation(*cu, YuvComponent::kY, pred_buffer.GetDataPtr(),
                       pred_buffer.GetStride());
    Distortion dist =
      metric.CompareSample(*cu, YuvComponent::kY, orig_pic_, pred_buffer);
    Bits bits = merge_idx + 1 - (merge_idx < max_merge_cand - 1 ? 0 : 1);
    double cost = dist + bits * qp.GetLambdaSqrt();
    cand_cost[merge_idx] = std::make_pair(merge_idx, cost);
  }
  std::stable_sort(cand_cost.begin(), cand_cost.end(),
                   [](std::pair<int, double> a, std::pair<int, double> b) {
    return a.second < b.second;
  });
  int num_merge_cand = kFastMergeNumCand;
  for (int merge_idx = kFastMergeNumCand; merge_idx >= 0; merge_idx--) {
    (*out_cand_list)[merge_idx] = cand_cost[merge_idx].first;
    if (cand_cost[merge_idx].second >
        cand_cost[0].second * kFastMergeCostFactor) {
      num_merge_cand = merge_idx;
    }
  }
  return num_merge_cand;
}

void InterSearch::SearchMotion(CodingUnit *cu, const Qp &qp,
                               bool uni_prediction_only,
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
  cost_l0 = SearchRefIdx(cu, qp, RefPicList::kL0, bitstream_writer,
                         orig_luma, pred, pred_stride, cost_l0, &state_l0,
                         nullptr);
  if (uni_prediction_only) {
    return;
  }

  CodingUnit::InterState state_bi;
  CodingUnit::InterState state_l1_unique_poc;
  Distortion cost_l1_unique_poc;
  cu->SetInterDir(InterDir::kL1);
  Distortion cost_l1 = std::numeric_limits<Distortion>::max();
  cost_l1 = SearchRefIdx(cu, qp, RefPicList::kL1, bitstream_writer,
                         orig_luma, pred, pred_stride, cost_l1, &state_bi,
                         &state_l1_unique_poc, &cost_l1_unique_poc);

  // Prepare initial bi-prediction state with best from both lists
  assert(cu->GetInterDir() == InterDir::kL1);
  cu->SetMv(state_l0.mv[0], RefPicList::kL0);
  cu->SetRefIdx(state_l0.ref_idx[0], RefPicList::kL0);
  cu->SetMvDelta(state_l0.mvd[0], RefPicList::kL0);
  cu->SetMvpIdx(state_l0.mvp_idx[0], RefPicList::kL0);
  InterDir best_uni_dir = cost_l0 <= cost_l1 ? InterDir::kL0 : InterDir::kL1;
  Distortion cost_best_bi =
    SearchBiIterative(cu, qp, bitstream_writer, best_uni_dir,
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
InterSearch::CompressAndEvalCbf(CodingUnit *cu, const Qp &qp,
                                const SyntaxWriter &bitstream_writer,
                                TransformEncoder *encoder,
                                YuvPicture *rec_pic) {
  std::array<bool, constants::kMaxYuvComponents> cbf_modified = { false };
  Distortion final_dist = 0;
  Distortion sum_dist_zero = 0;
  Distortion sum_dist_fast = 0;

  for (int c = 0; c < max_components_; c++) {
    const YuvComponent comp = YuvComponent(c);
    MetricType m = encoder_settings_.structural_ssd > 0 &&
      comp == YuvComponent::kY ? MetricType::kStructuralSsd : MetricType::kSsd;
    SampleMetric metric(m, qp, bitdepth_);
    SampleBuffer &pred_buffer = encoder->GetPredBuffer();
    MotionCompensation(*cu, comp, pred_buffer.GetDataPtr(),
                       pred_buffer.GetStride());
    // TODO(PH) Should update contexts after each component for rdo quant
    Distortion dist_orig =
      encoder->CompressAndEvalTransform(cu, comp, qp, bitstream_writer,
                                        orig_pic_, &cu_writer_, rec_pic);
    Distortion dist_zero =
      metric.CompareSample(*cu, comp, orig_pic_, encoder->GetPredBuffer());
    Distortion dist_fast = dist_orig;
    if (encoder_settings_.fast_inter_transform_dist && cu->GetCbf(comp) &&
        !encoder_settings_.structural_ssd) {
      // TODO(PH) Consider remove this, it's not really faster since we are
      // calculating the actual distortion above anyway
      dist_fast = encoder->GetResidualDist(*cu, comp, &metric);
    }
    bool force_comp_zero = false;
    if (cu->GetCbf(comp) &&
        !Restrictions::Get().disable_transform_cbf) {
      force_comp_zero = encoder->EvalCbfZero(cu, qp, comp, bitstream_writer,
                                             &cu_writer_, dist_fast, dist_zero);
    }
    // if cbf was already zero from start then dist_zero == dist_orig
    final_dist += force_comp_zero ? dist_zero : dist_orig;
    sum_dist_zero += dist_zero;
    sum_dist_fast += force_comp_zero ? dist_zero : dist_fast;
    cbf_modified[c] = force_comp_zero;
  }
  cu->SetRootCbf(cu->GetHasAnyCbf());
  cu->SetSkipFlag(cu->GetMergeFlag() && !cu->GetRootCbf());

  if (cu->GetRootCbf() && !(Restrictions::Get().disable_transform_cbf &&
                            Restrictions::Get().disable_transform_root_cbf)) {
    bool zero_root_cbf =
      encoder->EvalRootCbfZero(cu, qp, bitstream_writer, &cu_writer_,
                               sum_dist_fast, sum_dist_zero);
    if (zero_root_cbf) {
      for (int c = 0; c < constants::kMaxYuvComponents; c++) {
        cbf_modified[c] |= cu->GetCbf(YuvComponent(c));
      }
      final_dist = sum_dist_zero;
    }
  }

  for (int c = 0; c < constants::kMaxYuvComponents; c++) {
    if (!cbf_modified[c]) {
      continue;
    }
    YuvComponent comp = YuvComponent(c);
    cu->ClearCbf(comp);
    // TODO(Dev) Faster to save and reuse predicition buffers
    SampleBuffer reco =
      rec_pic->GetSampleBuffer(comp, cu->GetPosX(comp), cu->GetPosY(comp));
    MotionCompensation(*cu, comp, reco.GetDataPtr(), reco.GetStride());
  }
  cu->SetRootCbf(cu->GetHasAnyCbf());
  cu->SetSkipFlag(cu->GetMergeFlag() && !cu->GetRootCbf());
  return final_dist;
}

Distortion
InterSearch::CompressSkipOnly(CodingUnit *cu, const Qp &qp,
                              const SyntaxWriter &bitstream_writer,
                              TransformEncoder *encoder, YuvPicture *rec_pic) {
  assert(cu->GetPredMode() == PredictionMode::kInter);
  cu->SetSkipFlag(true);
  cu->SetRootCbf(false);

  Distortion sum_dist = 0;
  for (int c = 0; c < max_components_; c++) {
    const YuvComponent comp = YuvComponent(c);
    MetricType m = encoder_settings_.structural_ssd > 0 &&
      comp == YuvComponent::kY ? MetricType::kStructuralSsd : MetricType::kSsd;
    SampleMetric metric(m, qp, rec_pic->GetBitdepth());
    int posx = cu->GetPosX(comp);
    int posy = cu->GetPosY(comp);
    SampleBuffer reco_buffer = rec_pic->GetSampleBuffer(comp, posx, posy);
    MotionCompensation(*cu, comp, reco_buffer.GetDataPtr(),
                       reco_buffer.GetStride());
    cu->ClearCbf(comp);
    Distortion dist = metric.CompareSample(*cu, comp, orig_pic_, reco_buffer);
    sum_dist += dist;
  }
  return sum_dist;
}

Distortion
InterSearch::SearchBiIterative(CodingUnit *cu, const Qp &qp,
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
  int num_iterations = encoder_settings_.bipred_refinement_iterations;
  for (int iteration = 0; iteration < num_iterations; iteration++) {
    // If searching in L1 use original without L0 prediction
    cu->SetInterDir(search_list == RefPicList::kL0 ?
                    InterDir::kL1 : InterDir::kL0);
    MotionCompensation(*cu, comp, bipred_pred_buffer_.GetDataPtr(),
                       bipred_pred_buffer_.GetStride());
    bipred_orig_buffer_.SubtractWeighted(width, height, orig_luma,
                                         bipred_pred_buffer_);
    cu->SetInterDir(InterDir::kBi);

    Distortion prev_best = cost_best;
    cost_best = SearchRefIdx(cu, qp, search_list, bitstream_writer,
                             bipred_orig_buffer_, pred_buf, pred_stride,
                             cost_best, best_state);
    if (cost_best == prev_best) {
      break;
    }
    search_list = ReferencePictureLists::Inverse(search_list);
  }
  return cost_best;
}

template<typename TOrig>
Distortion
InterSearch::SearchRefIdx(CodingUnit *cu, const Qp &qp, RefPicList ref_list,
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
  const bool bipred = cu->GetInterDir() == InterDir::kBi;
  const double weight = bipred ? 0.5 : 1;
  const SearchMethod search_method =
    bipred ? SearchMethod::FullSearch : SearchMethod::TzSearch;
  Distortion cost_best = initial_best_cost;
  Distortion cost_best_unique = std::numeric_limits<Distortion>::max();
  if (!bipred) {
    // Clear out old search for deblocking
    RefPicList other_list = ReferencePictureLists::Inverse(ref_list);
    cu->SetMv(MotionVector(), other_list);
    cu->SetRefIdx(-1, other_list);
  }

  for (int ref_idx = 0; ref_idx < num_ref_idx; ref_idx++) {
    InterPredictorList mvp_list = GetMvPredictors(*cu, ref_list, ref_idx);
    const MotionVector *mv_start = nullptr;
    int mvp_idx;
    if (!bipred) {
      const YuvPicture *ref_pic =
        cu->GetRefPicLists()->GetRefPic(ref_list, ref_idx);
      mvp_idx = EvalStartMvp(*cu, qp, mvp_list, *ref_pic, pred, pred_stride);
    } else {
      mv_start = &unipred_best_mv_[static_cast<int>(ref_list)][ref_idx];
      mvp_idx = unipred_best_mvp_idx_[static_cast<int>(ref_list)][ref_idx];
    }
    Distortion dist = 0;
    MotionVector mv_subpel;
    if (!bipred && ref_list == RefPicList::kL1 &&
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
    if (!bipred || encoder_settings_.bipred_refinement_iterations > 1) {
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
    Bits bits = GetInterPredBits(*cu, bitstream_writer);
    Distortion dist_scaled =
      static_cast<Distortion>(std::floor(dist * weight));
    Distortion cost = dist_scaled + ((bits * lambda) >> 16);
    if (cost < cost_best) {
      cost_best = cost;
      cu->SaveStateTo(best_state);
    }
    if (best_state_unique && !bipred && ref_list == RefPicList::kL1) {
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
InterSearch::MotionEstimation(const CodingUnit &cu, const Qp &qp,
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
  } else if (search_method == SearchMethod::TzSearch) {
    MetricType metric_type = GetFullpelMetric(cu);
    TzSearch tz_search(bitdepth_, orig_pic_, *this, encoder_settings_,
                       kSearchRangeUni);
    mv_fullpel =
      tz_search.Search(cu, qp, metric_type, mvp, *ref_pic, clip_min, clip_max,
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

MotionVector InterSearch::FullSearch(const CodingUnit &cu, const Qp &qp,
                                     const MotionVector &mvp,
                                     const YuvPicture &ref_pic,
                                     const MotionVector &mv_min,
                                     const MotionVector &mv_max) {
  const YuvComponent comp = YuvComponent::kY;
  const int mv_precision = constants::kMvPrecisionShift;
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  uint32_t lambda =
    static_cast<uint32_t>(std::floor(65536.0 * qp.GetLambdaSqrt()));
  MetricType fullpel_metric = GetFullpelMetric(cu);
  SampleMetric metric(fullpel_metric, qp, bitdepth_);
  const Sample *ref_cu = ref_pic.GetSamplePtr(comp, cu.GetPosX(comp),
                                              cu.GetPosY(comp));
  intptr_t ref_stride = ref_pic.GetStride(comp);
  Distortion cost_best = std::numeric_limits<Distortion>::max();
  MotionVector mv_best;
  for (int mv_y = mv_min.y; mv_y <= mv_max.y; mv_y++) {
    for (int mv_x = mv_min.x; mv_x <= mv_max.x; mv_x++) {
      const Sample *ref_mv = ref_cu + mv_y * ref_stride + mv_x;
      Distortion dist = metric.CompareSample(comp, width, height,
                                             bipred_orig_buffer_.GetDataPtr(),
                                             bipred_orig_buffer_.GetStride(),
                                             ref_mv, ref_stride);
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

template<typename TOrig>
MotionVector
InterSearch::SubpelSearch(const CodingUnit &cu, const Qp &qp,
                          const YuvPicture &ref_pic, const MotionVector &mvp,
                          const MotionVector &mv_fullpel,
                          const DataBuffer<TOrig> &orig_buffer,
                          Sample *buffer, ptrdiff_t buffer_stride,
                          Distortion *out_dist) {
  SampleMetric metric = SampleMetric(MetricType::kSatd, qp, bitdepth_);
  uint32_t lambda =
    static_cast<uint32_t>(std::floor(65536.0 * qp.GetLambdaSqrt()));
  MotionVector mv_subpel(mv_fullpel.x * (1 << constants::kMvPrecisionShift),
                         mv_fullpel.y * (1 << constants::kMvPrecisionShift));
  Distortion best_cost = std::numeric_limits<Distortion>::max();
  int best_idx = 0;

  // Half-pel
  for (int i = 0; i < static_cast<int>(kSquareXYHalf.size()); i++) {
    int mv_x = mv_subpel.x + kSquareXYHalf[i][0] * 2;
    int mv_y = mv_subpel.y + kSquareXYHalf[i][1] * 2;
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
    mv_subpel.x += kSquareXYHalf[best_idx][0] * 2;
    mv_subpel.y += kSquareXYHalf[best_idx][1] * 2;
  }
  best_idx = 0;

  // Qpel
  for (int i = 1; i < static_cast<int>(kSquareXYQpel.size()); i++) {
    int mv_x = mv_subpel.x + kSquareXYQpel[i][0];
    int mv_y = mv_subpel.y + kSquareXYQpel[i][1];
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
    mv_subpel.x += kSquareXYQpel[best_idx][0];
    mv_subpel.y += kSquareXYQpel[best_idx][1];
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

int InterSearch::EvalStartMvp(const CodingUnit &cu, const Qp &qp,
                              const InterPredictorList &mvp_list,
                              const YuvPicture &ref_pic, Sample *pred_buf,
                              ptrdiff_t pred_stride) {
  SampleMetric metric(MetricType::kSad, qp, bitdepth_);
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

MetricType InterSearch::GetFullpelMetric(const CodingUnit & cu) {
  return cu.GetHeight(YuvComponent::kY) > 8 ?
    MetricType::kSadFast : MetricType::kSad;
}

Bits InterSearch::GetInterPredBits(const CodingUnit &cu,
                                   const SyntaxWriter &bitstream_writer) {
  if (encoder_settings_.fast_inter_pred_bits) {
    // TODO(PH) Consider removing this "faster" version
    PicturePredictionType pic_pred_type = cu.GetPicType();
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
      for (int i = 0; i < static_cast<int>(RefPicList::kTotalNumber); i++) {
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
  } else {
    RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
    cu_writer_.WriteInterPrediction(cu, YuvComponent::kY, &rdo_writer);
    return rdo_writer.GetNumWrittenBits();
  }
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
  int mvd_x = (mv_x * (1 << mv_scale)) - mvp.x;
  int mvd_y = (mv_y * (1 << mv_scale)) - mvp.y;
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

}   // namespace xvc
