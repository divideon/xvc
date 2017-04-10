/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/cu_encoder.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <limits>
#include <utility>

#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"
#include "xvc_enc_lib/sample_metric.h"

namespace xvc {

struct CuEncoder::RdoCost {
  RdoCost() = default;
  explicit RdoCost(Cost c) : cost(c), dist(0) {}
  RdoCost(Cost c, Distortion d) : cost(c), dist(d) {}
  bool operator<(const RdoCost &other) { return cost < other.cost; }
  Cost cost;
  Distortion dist;
};

CuEncoder::CuEncoder(const QP &qp, const YuvPicture &orig_pic,
                     YuvPicture *rec_pic, PictureData *pic_data,
                     const SpeedSettings &speed_settings)
  : TransformEncoder(rec_pic->GetBitdepth(), orig_pic),
  pic_qp_(qp),
  orig_pic_(orig_pic),
  speed_settings_(speed_settings),
  rec_pic_(*rec_pic),
  pic_data_(*pic_data),
  inter_search_(rec_pic->GetBitdepth(), orig_pic, *pic_data->GetRefPicLists(),
                speed_settings),
  intra_pred_(rec_pic->GetBitdepth()),
  cu_writer_(pic_data_, &intra_pred_) {
  for (int tree_idx = 0; tree_idx < constants::kMaxNumCuTrees; tree_idx++) {
    const CuTree cu_tree = static_cast<CuTree>(tree_idx);
    const int max_depth = static_cast<int>(rdo_temp_cu_[tree_idx].size());
    for (int depth = 0; depth < max_depth; depth++) {
      int width = constants::kMaxBlockSize >> depth;
      int height = constants::kMaxBlockSize >> depth;
      rdo_temp_cu_[tree_idx][depth] =
        pic_data_.CreateCu(cu_tree, depth, -1, -1, width, height);
    }
  }
}

CuEncoder::~CuEncoder() {
  for (int tree_idx = 0; tree_idx < constants::kMaxNumCuTrees; tree_idx++) {
    const int max_depth = static_cast<int>(rdo_temp_cu_[tree_idx].size());
    for (int depth = 0; depth < max_depth; depth++) {
      pic_data_.ReleaseCu(rdo_temp_cu_[tree_idx][depth]);
    }
  }
}

void CuEncoder::EncodeCtu(int rsaddr, SyntaxWriter *bitstream_writer) {
  RdoSyntaxWriter rdo_writer(*bitstream_writer);
  CodingUnit *ctu = pic_data_.GetCtu(CuTree::Primary, rsaddr);
  CompressCu(&ctu, &rdo_writer);
  pic_data_.SetCtu(CuTree::Primary, rsaddr, ctu);
  if (pic_data_.HasSecondaryCuTree()) {
    RdoSyntaxWriter rdo_writer2(*bitstream_writer);
    CodingUnit *ctu2 = pic_data_.GetCtu(CuTree::Secondary, rsaddr);
    CompressCu(&ctu2, &rdo_writer2);
    pic_data_.SetCtu(CuTree::Secondary, rsaddr, ctu2);
  }

  WriteCtu(rsaddr, bitstream_writer);
}

Distortion CuEncoder::CompressCu(CodingUnit **best_cu,
                                 RdoSyntaxWriter *writer) {
  const QP &qp = pic_qp_;
  const int kMaxTrSize =
    !Restrictions::Get().disable_ext_transform_size_64 ? 64 : 32;
  const int depth = (*best_cu)->GetDepth();
  const bool do_split = depth < pic_data_.GetMaxDepth((*best_cu)->GetCuTree());
  const bool do_full = (*best_cu)->IsFullyWithinPicture() &&
    (*best_cu)->GetWidth(YuvComponent::kY) <= kMaxTrSize &&
    (*best_cu)->GetHeight(YuvComponent::kY) <= kMaxTrSize;
  assert(do_split || do_full);
  if (!do_split) {
    return CompressNoSplit(best_cu, writer);
  }
  Bits start_bits = writer->GetNumWrittenBits();

  Distortion full_dist = std::numeric_limits<Distortion>::max();
  Cost full_cost = std::numeric_limits<Cost>::max();
  CodingUnit::ReconstructionState *fullcu_state = &temp_cu_state_[depth];
  RdoSyntaxWriter fullcu_writer(*writer);
  if (do_full) {
    full_dist = CompressNoSplit(best_cu, &fullcu_writer);
    Bits full_bits = fullcu_writer.GetNumWrittenBits() - start_bits;
    full_cost =
      full_dist + static_cast<Cost>(full_bits * qp.GetLambda() + 0.5);
    (*best_cu)->SaveStateTo(fullcu_state, rec_pic_);
  }

  // After full and split CU have been evaluated best_cu will not change
  CodingUnit *cu = *best_cu;
  RdoSyntaxWriter splitcu_writer(*writer);
  Bits frac_bits_before_split = 0;
  Distortion split_dist =
    CompressSplitCu(cu, &splitcu_writer, &frac_bits_before_split);
  Bits split_bits = splitcu_writer.GetNumWrittenBits() - start_bits;
  Cost split_cost =
    split_dist + static_cast<Cost>(split_bits * qp.GetLambda() + 0.5);
  if (split_cost < full_cost) {
    *writer = splitcu_writer;
    return split_dist;
  }

  // No split case
  cu->UnSplit();
  cu->LoadStateFrom(*fullcu_state, &rec_pic_);
  pic_data_.MarkUsedInPic(cu);
  *writer = fullcu_writer;
  return full_dist;
}

Distortion CuEncoder::CompressSplitCu(CodingUnit *cu,
                                      RdoSyntaxWriter *rdo_writer,
                                      Bits *frac_bits_before_split) {
  cu->SplitQuad();
  pic_data_.ClearMarkCuInPic(cu);
  Distortion dist = 0;
  for (auto &sub_cu : cu->GetSubCu()) {
    if (sub_cu) {
      dist += CompressCu(&sub_cu, rdo_writer);
    }
  }
  *frac_bits_before_split = rdo_writer->GetFractionalBits();
  if (cu->IsFullyWithinPicture()) {
    int max_depth = pic_data_.GetMaxDepth(cu->GetCuTree());
    assert(cu->GetDepth() < max_depth);
    rdo_writer->WriteSplitFlag(*cu, max_depth, true);
  }
  return dist;
}

Distortion CuEncoder::CompressNoSplit(CodingUnit **best_cu,
                                      RdoSyntaxWriter *writer) {
  const QP &qp = pic_qp_;
  RdoCost best_cost(std::numeric_limits<Cost>::max());
  CodingUnit::ReconstructionState *best_state =
    &temp_cu_state_[(*best_cu)->GetDepth() + 1];
  CodingUnit *cu = *best_cu;
  cu->SetQp(qp);
  cu->SetSplit(false);

  if (pic_data_.IsIntraPic()) {
    best_cost = CompressIntra(cu, qp, *writer);
  } else {
    // Inter pic type
    const int cu_tree = static_cast<int>(cu->GetCuTree());
    CodingUnit *temp_cu = rdo_temp_cu_[cu_tree][cu->GetDepth()];
    temp_cu->SetPosition(cu->GetPosX(YuvComponent::kY),
                         cu->GetPosY(YuvComponent::kY));
    temp_cu->SetQp(qp);
    temp_cu->SetSplit(false);

    RdoCost cost;
    if (!Restrictions::Get().disable_inter_merge_mode) {
      cost = CompressMerge(temp_cu, qp, *writer);
      if (cost < best_cost) {
        best_cost = cost;
        temp_cu->SaveStateTo(best_state, rec_pic_);
        std::swap(cu, temp_cu);
      }
    }

    cost = CompressInter(temp_cu, qp, *writer);
    if (cost < best_cost) {
      best_cost = cost;
      temp_cu->SaveStateTo(best_state, rec_pic_);
      std::swap(cu, temp_cu);
    }

    if (speed_settings_.always_evaluate_intra_in_inter || cu->GetHasAnyCbf()) {
      cost = CompressIntra(temp_cu, qp, *writer);
      if (cost < best_cost) {
        best_cost = cost;
        temp_cu->SaveStateTo(best_state, rec_pic_);
        std::swap(cu, temp_cu);
      }
    }

    *best_cu = cu;
    rdo_temp_cu_[cu_tree][cu->GetDepth()] = temp_cu;
    cu->LoadStateFrom(*best_state, &rec_pic_);
  }
  cu->SetRootCbf(cu->GetHasAnyCbf());
  pic_data_.MarkUsedInPic(cu);

  for (YuvComponent comp : pic_data_.GetComponents(cu->GetCuTree())) {
    cu_writer_.WriteComponent(*cu, comp, writer);
  }
  int max_depth = pic_data_.GetMaxDepth(cu->GetCuTree());
  if (cu->GetDepth() < max_depth) {
    writer->WriteSplitFlag(*cu, max_depth, false);
  }
  return best_cost.dist;
}

CuEncoder::RdoCost
CuEncoder::CompressIntra(CodingUnit *cu, const QP &qp,
                         const SyntaxWriter &writer) {
  cu->SetPredMode(PredictionMode::kIntra);
  cu->SetSkipFlag(false);
  Distortion dist = 0;
  for (YuvComponent comp : pic_data_.GetComponents(cu->GetCuTree())) {
    if (util::IsLuma(comp)) {
      IntraMode best_mode = SearchIntraLuma(cu, comp, qp, writer);
      cu->SetIntraModeLuma(best_mode);
    } else if (util::IsFirstChroma(comp)) {
      IntraChromaMode chroma_mode = SearchIntraChroma(cu, comp, qp, writer);
      cu->SetIntraModeChroma(chroma_mode);
    }
    dist += CompressComponent(cu, comp, qp);
  }
  return ComputeDistCostNoSplit(*cu, qp, writer, dist);
}

CuEncoder::RdoCost
CuEncoder::CompressInter(CodingUnit *cu, const QP &qp,
                         const SyntaxWriter &bitstream_writer) {
  PicturePredictionType pic_pred_type = pic_data_.GetPredictionType();
  inter_search_.Search(cu, qp, pic_pred_type, bitstream_writer, &temp_pred_);
  Distortion dist = CompressAndEvalCbf(cu, qp, bitstream_writer);
  return ComputeDistCostNoSplit(*cu, qp, bitstream_writer, dist);
}

CuEncoder::RdoCost
CuEncoder::CompressMerge(CodingUnit *cu, const QP &qp,
                         const SyntaxWriter &bitstream_writer) {
  const bool kFastMergeRdo = (speed_settings_.fast_merge_rdo == 1);   // FDM
#if HM_STRICT
  const bool kAlwaysExplicitSkipFlagRdo = true;
#else
  const bool kAlwaysExplicitSkipFlagRdo = false;
#endif
  cu->SetPredMode(PredictionMode::kInter);
  cu->SetMergeFlag(true);

  InterMergeCandidateList merge_list = inter_search_.GetMergeCandidates(*cu);
  RdoCost best_cost(std::numeric_limits<Cost>::max());
  Cost best_non_skip = std::numeric_limits<Cost>::max();
  CodingUnit::TransformState best_state;
  int best_merge_idx = -1;
#if HM_STRICT
  bool explicit_skip = false;
#endif
  bool fast_eval_skip_only = false;

  int num_merge_cand = Restrictions::Get().disable_inter_merge_candidates ?
    1 : static_cast<int>(merge_list.size());
  for (int idx = 0; idx < num_merge_cand; idx++) {
    cu->SetMergeIdx(idx);
    inter_search_.ApplyMerge(cu, merge_list, idx);
    Distortion dist_zero = 0;
    bool skip_reconstruction_applied = false;
    if (!fast_eval_skip_only) {
      // First step is to evaluate normal merge using both cbf and root cbf rdo
      // A non HM strict encoder will calculate cost based on skip flag and
      // not root cbf flag
      Distortion dist =
        CompressAndEvalCbf(cu, qp, bitstream_writer, &dist_zero);
      RdoCost cost = ComputeDistCostNoSplit(*cu, qp, bitstream_writer, dist);
#if HM_STRICT
      const bool bias_non_skip = explicit_skip && cost.cost == best_cost.cost;
#else
      const bool bias_non_skip = false;
#endif
      if (cost.cost < best_cost.cost || bias_non_skip) {
        best_cost = cost;
        best_merge_idx = idx;
#if HM_STRICT
        explicit_skip = false;
#endif
        cu->SaveStateTo(&best_state, rec_pic_);
      }
      if (kFastMergeRdo && cost.cost < best_non_skip) {
        fast_eval_skip_only = cu->GetSkipFlag();
        best_non_skip = cost.cost;
      }
      if (cu->GetSkipFlag()) {
        continue;
      }
      if (kAlwaysExplicitSkipFlagRdo) {
        // Fast skip rdo
        cu->SetSkipFlag(true);
        cu->SetRootCbf(false);
        for (int c = 0; c < constants::kMaxYuvComponents; c++) {
          cu->SetCbf(YuvComponent(c), false);
        }
        skip_reconstruction_applied = false;
      }
    } else {
      dist_zero = CompressSkipOnly(cu, qp, bitstream_writer);
      skip_reconstruction_applied = true;
    }
    // A strict HM encoder will always perform extra skip rdo with correct flag
    // Otherwise only check explicit split when doing fast merge rdo search
    if (kAlwaysExplicitSkipFlagRdo || fast_eval_skip_only) {
      RdoCost cost_skip =
        ComputeDistCostNoSplit(*cu, qp, bitstream_writer, dist_zero);
      if (cost_skip.cost < best_cost.cost) {
        best_cost = cost_skip;
        best_merge_idx = idx;
#if HM_STRICT
        explicit_skip = true;
#endif
        if (!skip_reconstruction_applied) {
          // TODO(Dev) Remove when keeping prediction buffer for all components
          CompressSkipOnly(cu, qp, bitstream_writer, false);
        }
        cu->SaveStateTo(&best_state, rec_pic_);
      }
    }
  }

  cu->SetMergeIdx(best_merge_idx);
  inter_search_.ApplyMerge(cu, merge_list, best_merge_idx);
  cu->LoadStateFrom(best_state, &rec_pic_);
  cu->SetSkipFlag(!cu->GetRootCbf());
  return best_cost;
}

Distortion CuEncoder::CompressAndEvalCbf(CodingUnit *cu, const QP &qp,
                                         const SyntaxWriter &bitstream_writer,
                                         Distortion *out_dist_zero) {
  SampleMetric metric(MetricType::kSSE, qp, rec_pic_.GetBitdepth());
  std::array<bool, constants::kMaxYuvComponents> cbf_modified = { false };
  Distortion final_dist = 0;
  Distortion sum_dist_zero = 0;
  Distortion sum_dist_fast = 0;

  for (int c = 0; c < pic_data_.GetMaxNumComponents(); c++) {
    const YuvComponent comp = YuvComponent(c);
    Distortion dist_orig = CompressComponent(cu, comp, qp);
    Distortion dist_zero =
      metric.CompareSample(*cu, comp, orig_pic_, temp_pred_);
    Distortion dist_fast = dist_orig;
    // TODO(Dev) Investigate trade-off with using clipped samples for rdo
#if HM_STRICT
    if (cu->GetCbf(comp)) {
      dist_fast =
        metric.CompareShort(comp, cu->GetWidth(comp), cu->GetHeight(comp),
                            temp_resi_orig_, temp_resi_);
    }
#endif
    bool force_comp_zero = false;
    if (!Restrictions::Get().disable_transform_cbf) {
      force_comp_zero = EvalCbfZero(cu, qp, comp, bitstream_writer,
                                    dist_fast, dist_zero);
    }
    final_dist += force_comp_zero ? dist_zero : dist_orig;
    sum_dist_zero += dist_zero;
    sum_dist_fast += force_comp_zero ? dist_zero : dist_fast;
    cbf_modified[c] = force_comp_zero;
  }
  cu->SetRootCbf(cu->GetHasAnyCbf());
  cu->SetSkipFlag(cu->GetMergeFlag() && !cu->GetRootCbf());
  if (out_dist_zero) {
    *out_dist_zero = sum_dist_zero;
  }

  if (cu->GetRootCbf() && !(Restrictions::Get().disable_transform_cbf &&
                            Restrictions::Get().disable_transform_root_cbf)) {
    bool force_all_zero = EvalRootCbfZero(cu, qp, bitstream_writer,
                                          sum_dist_fast, sum_dist_zero);
    if (force_all_zero) {
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
    cu->SetCbf(comp, false);
    // TODO(Dev) Faster to save and reuse predicition buffers
    Sample *reco = rec_pic_.GetSamplePtr(comp, cu->GetPosX(comp),
                                         cu->GetPosY(comp));
    ptrdiff_t reco_stride = rec_pic_.GetStride(comp);
    inter_search_.MotionCompensation(*cu, comp, reco, reco_stride);
  }
  cu->SetRootCbf(cu->GetHasAnyCbf());
  cu->SetSkipFlag(cu->GetMergeFlag() && !cu->GetRootCbf());
  return final_dist;
}

Distortion CuEncoder::CompressSkipOnly(CodingUnit *cu, const QP &qp,
                                       const SyntaxWriter &bitstream_writer,
                                       bool calc_distortion) {
  assert(cu->GetPredMode() == PredictionMode::kInter);
  cu->SetSkipFlag(true);
  cu->SetRootCbf(false);

  SampleMetric metric(MetricType::kSSE, qp, rec_pic_.GetBitdepth());
  Distortion sum_dist = 0;
  for (int c = 0; c < pic_data_.GetMaxNumComponents(); c++) {
    const YuvComponent comp = YuvComponent(c);
    int posx = cu->GetPosX(comp);
    int posy = cu->GetPosY(comp);
    SampleBuffer reco_buffer = rec_pic_.GetSampleBuffer(comp, posx, posy);
    inter_search_.MotionCompensation(*cu, comp, reco_buffer.GetDataPtr(),
                                     reco_buffer.GetStride());
    cu->SetCbf(comp, false);
    if (calc_distortion) {
      Distortion dist = metric.CompareSample(*cu, comp, orig_pic_, reco_buffer);
      sum_dist += dist;
    }
  }
  return sum_dist;
}

bool CuEncoder::EvalCbfZero(CodingUnit *cu, const QP &qp,
                            YuvComponent comp,
                            const SyntaxWriter &rdo_writer,
                            Distortion dist_non_zero,
                            Distortion dist_zero) {
  if (!cu->GetCbf(comp)) {
    return false;
  }
  RdoSyntaxWriter non_zero_writer(rdo_writer, 0);
  non_zero_writer.WriteCbf(*cu, comp, true);
  non_zero_writer.WriteCoefficients(*cu, comp, cu->GetCoeff(comp),
                                    cu->GetCoeffStride());
  Bits non_zero_bits = non_zero_writer.GetNumWrittenBits();

  RdoSyntaxWriter zero_writer(rdo_writer, 0);
  zero_writer.WriteCbf(*cu, comp, false);
  Bits bits_zero = zero_writer.GetNumWrittenBits();

  Cost cost_non_zero = dist_non_zero +
    static_cast<Cost>(non_zero_bits * qp.GetLambda() + 0.5);
  Cost cost_zero = dist_zero +
    static_cast<Cost>(bits_zero * qp.GetLambda() + 0.5);
  if (cost_zero < cost_non_zero) {
    cu->SetCbf(comp, false);
    return true;
  }
  return false;
}

bool CuEncoder::EvalRootCbfZero(CodingUnit *cu, const QP &qp,
                                const SyntaxWriter &bitstream_writer,
                                Distortion sum_dist_non_zero,
                                Distortion sum_dist_zero) {
  RdoSyntaxWriter rdo_writer_nonzero(bitstream_writer, 0);
  // TODO(Dev) Investigate gains of correct root cbf signaling
#if HM_STRICT
  for (int c = 0; c < pic_data_.GetMaxNumComponents(); c++) {
    const YuvComponent comp = YuvComponent(c);
    bool cbf = cu->GetCbf(comp);
    rdo_writer_nonzero.WriteCbf(*cu, comp, cbf);
    if (cbf) {
      rdo_writer_nonzero.WriteCoefficients(*cu, comp, cu->GetCoeff(comp),
                                           cu->GetCoeffStride());
    }
  }
#else
  for (int c = 0; c < pic_data_.GetMaxNumComponents(); c++) {
    cu_writer_.WriteCoefficients(*cu, YuvComponent(c), &rdo_writer_nonzero);
  }
#endif
  Bits bits_non_zero = rdo_writer_nonzero.GetNumWrittenBits();

  // TODO(Dev) Investigate gains of correct start state
#if HM_STRICT
  RdoSyntaxWriter rdo_writer_zero(rdo_writer_nonzero, 0);
#else
  RdoSyntaxWriter rdo_writer_zero(bitstream_writer, 0);
#endif
  // TODO(Dev) Investigate gains of using correct skip syntax
#if HM_STRICT
  rdo_writer_zero.WriteRootCbf(false);
#else
  if (cu->GetSkipFlag()) {
    rdo_writer_zero.WriteSkipFlag(*cu, true);
  } else {
    rdo_writer_zero.WriteRootCbf(false);
  }
#endif
  Bits bits_zero = rdo_writer_zero.GetNumWrittenBits();

  Cost cost_zero = sum_dist_zero +
    static_cast<Cost>(bits_zero * qp.GetLambda() + 0.5);
  Cost cost_non_zero = sum_dist_non_zero +
    static_cast<Cost>(bits_non_zero * qp.GetLambda() + 0.5);
  return cost_zero < cost_non_zero;
}

IntraMode CuEncoder::SearchIntraLuma(CodingUnit *cu, YuvComponent comp,
                                     const QP &qp,
                                     const SyntaxWriter &bitstream_writer) {
  static const std::array<std::array<uint8_t, 8>, 8> kNumIntraFastModesExt = { {
      // 1, 2, 4, 8, 16, 32, 64, 128
      { 0, 0, 0, 0, 0, 0, 0, 0 },   // 1
      { 0, 0, 0, 0, 0, 0, 0, 0 },   // 2
      { 0, 0, 3, 3, 3, 3, 2, 2 },   // 4
      { 0, 0, 3, 3, 3, 3, 3, 2 },   // 8
      { 0, 0, 3, 3, 3, 3, 3, 2 },   // 16
      { 0, 0, 3, 3, 3, 3, 3, 2 },   // 32
      { 0, 0, 2, 3, 3, 3, 3, 2 },   // 64
      { 0, 0, 2, 2, 2, 2, 2, 3 },   // 128
    } };
  static const std::array<uint8_t, 7> kNumIntraFastModesNoExt = {
    /*1x1: */ 0, /*2x2: */ 3, /*4x4: */ 8, /*8x8: */ 8, /*16x16: */ 3,
    /*32x32: */ 3, /*64x64: */ 3
  };
  Sample *reco =
    rec_pic_.GetSamplePtr(comp, cu->GetPosX(comp), cu->GetPosY(comp));
  const ptrdiff_t reco_stride = rec_pic_.GetStride(comp);
  IntraPredictorLuma mpm = intra_pred_.GetPredictorLuma(*cu);
  IntraPrediction::State intra_state =
    intra_pred_.ComputeReferenceState(*cu, comp, reco, reco_stride);

  SampleMetric metric(MetricType::kSATD, qp, rec_pic_.GetBitdepth());
  std::array<std::pair<IntraMode, double>, IntraMode::kTotalNumber> modes_cost;
  for (int i = 0; i < IntraMode::kTotalNumber; i++) {
    IntraMode intra_mode = static_cast<IntraMode>(i);
    intra_pred_.Predict(intra_mode, *cu, comp, intra_state,
                        temp_pred_.GetDataPtr(), temp_pred_.GetStride());

    // Bits
    RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
    rdo_writer.WriteIntraMode(intra_mode, mpm);
    size_t bits = rdo_writer.GetNumWrittenBits();

    uint64_t sad = metric.CompareSample(*cu, comp, orig_pic_,
                                        temp_pred_.GetDataPtr(),
                                        temp_pred_.GetStride());
    double cost = sad + bits * qp.GetLambdaSqrt();
    modes_cost[i] = std::make_pair(intra_mode, cost);
  }
  std::stable_sort(modes_cost.begin(), modes_cost.end(),
                   [](std::pair<IntraMode, double> p1,
                      std::pair<IntraMode, double> p2) {
    return p1.second < p2.second;
  });

  // Extend shortlist with mpm modes if not already included
  int width_log2 = util::SizeToLog2(cu->GetWidth(comp));
  int height_log2 = util::SizeToLog2(cu->GetHeight(comp));
  int num_modes_for_slow_rdo = kNumIntraFastModesNoExt[width_log2];
  if (speed_settings_.fast_intra_mode_eval_level == 2) {
    num_modes_for_slow_rdo = kNumIntraFastModesExt[width_log2][height_log2];
  } else if (speed_settings_.fast_intra_mode_eval_level == 0) {
    num_modes_for_slow_rdo = 33;
  }
  for (int i = 0; i < mpm.num_neighbor_modes; i++) {
    bool found = false;
    for (int j = 0; j < num_modes_for_slow_rdo; j++) {
      if (modes_cost[j].first == mpm[i]) {
        found = true;
        break;
      }
    }
    if (!found) {
      modes_cost[num_modes_for_slow_rdo++].first = mpm[i];
    }
  }

  IntraMode best_mode = static_cast<IntraMode>(0);
  Cost best_cost = std::numeric_limits<Cost>::max();
  for (int i = 0; i < num_modes_for_slow_rdo; i++) {
    IntraMode intra_mode = modes_cost[i].first;
    cu->SetIntraModeLuma(intra_mode);

    // Full reconstruction
    Distortion ssd = CompressComponent(cu, comp, qp);

    // Bits
    RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
    cu_writer_.WriteComponent(*cu, comp, &rdo_writer);
    size_t bits = rdo_writer.GetNumWrittenBits();

    Cost cost = ssd + static_cast<Cost>(bits * qp.GetLambda() + 0.5);
    if (cost < best_cost) {
      best_cost = cost;
      best_mode = intra_mode;
    }
  }
  return best_mode;
}

IntraChromaMode
CuEncoder::SearchIntraChroma(CodingUnit *cu, YuvComponent comp,
                             const QP &qp,
                             const SyntaxWriter &bitstream_writer) {
  const CodingUnit *luma_cu = pic_data_.GetLumaCu(cu);
  IntraMode luma_mode = luma_cu->GetIntraMode(YuvComponent::kY);
  IntraPredictorChroma chroma_modes =
    intra_pred_.GetPredictorsChroma(luma_mode);
  IntraChromaMode best_mode = IntraChromaMode::kDMChroma;
  Cost best_cost = std::numeric_limits<Cost>::max();
  if (!Restrictions::Get().disable_intra_chroma_predictor) {
    for (int i = 0; i < static_cast<int>(chroma_modes.size()); i++) {
      IntraChromaMode chroma_mode = chroma_modes[i];
      cu->SetIntraModeChroma(chroma_mode);

      // Full reconstruction
      Distortion ssd = 0;
      ssd += CompressComponent(cu, YuvComponent::kU, qp);
      ssd += CompressComponent(cu, YuvComponent::kV, qp);

      // Bits
      RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
      cu_writer_.WriteComponent(*cu, YuvComponent::kU, &rdo_writer);
      cu_writer_.WriteComponent(*cu, YuvComponent::kV, &rdo_writer);
      size_t bits = rdo_writer.GetNumWrittenBits();

      Cost cost = ssd + static_cast<Cost>(bits * qp.GetLambda() + 0.5);
      if (cost < best_cost) {
        best_cost = cost;
        best_mode = chroma_modes[i];
      }
    }
  }
  return best_mode;
}

Distortion CuEncoder::CompressComponent(CodingUnit *cu, YuvComponent comp,
                                        const QP &qp) {
  // Predict
  if (cu->IsIntra()) {
    int cu_x = cu->GetPosX(comp);
    int cu_y = cu->GetPosY(comp);
    Sample *reco = rec_pic_.GetSamplePtr(comp, cu_x, cu_y);
    ptrdiff_t reco_stride = rec_pic_.GetStride(comp);
    IntraMode intra_mode = cu->GetIntraMode(comp);
    intra_pred_.Predict(intra_mode, *cu, comp, reco, reco_stride,
                        temp_pred_.GetDataPtr(), temp_pred_.GetStride());
  } else {
    inter_search_.MotionCompensation(*cu, comp, temp_pred_.GetDataPtr(),
                                     temp_pred_.GetStride());
  }

  return TransformAndReconstruct(cu, comp, qp, orig_pic_, &rec_pic_);
}

CuEncoder::RdoCost
CuEncoder::ComputeDistCostNoSplit(const CodingUnit &cu, const QP &qp,
                                  const SyntaxWriter &bitstream_writer,
                                  Distortion ssd) {
  RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
  for (YuvComponent comp : pic_data_.GetComponents(cu.GetCuTree())) {
    cu_writer_.WriteComponent(cu, comp, &rdo_writer);
  }
  Bits bits = rdo_writer.GetNumWrittenBits();
  Cost cost = ssd + static_cast<Cost>(bits * qp.GetLambda() + 0.5);
  return RdoCost(cost, ssd);
}

void CuEncoder::WriteCtu(int rsaddr, SyntaxWriter *writer) {
  writer->ResetBitCounting();
  CodingUnit *ctu = pic_data_.GetCtu(CuTree::Primary, rsaddr);
  cu_writer_.WriteCu(*ctu, writer);
  if (pic_data_.HasSecondaryCuTree()) {
    CodingUnit *ctu2 = pic_data_.GetCtu(CuTree::Secondary, rsaddr);
    cu_writer_.WriteCu(*ctu2, writer);
  }
#if HM_STRICT
  writer->WriteEndOfSlice(false);
#endif
}

}   // namespace xvc
