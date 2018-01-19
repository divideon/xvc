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
#include <cstdlib>
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

InterSearch::InterSearch(const EncoderSimdFunctions &simd,
                         const PictureData &pic_data,
                         const YuvPicture &orig_pic,
                         const YuvPicture &rec_pic,
                         const ReferencePictureLists &ref_pic_list,
                         const EncoderSettings &encoder_settings)
  : InterPrediction(simd.inter_prediction, rec_pic, pic_data.GetBitdepth()),
  bitdepth_(pic_data.GetBitdepth()),
  max_components_(pic_data.GetMaxNumComponents()),
  poc_(pic_data.GetPoc()),
  sub_gop_length_(static_cast<int>(pic_data.GetSubGopLength())),
  orig_pic_(orig_pic),
  encoder_settings_(encoder_settings),
  simd_(simd),
  cu_metric_(simd.sample_metric, bitdepth_, encoder_settings.structural_ssd ?
             MetricType::kStructuralSsd : MetricType::kSsd),
  satd_metric_(simd.sample_metric, bitdepth_, MetricType::kSatd),
  cu_writer_(pic_data, nullptr),
  bipred_orig_buffer_(constants::kMaxBlockSize, constants::kMaxBlockSize),
  bipred_pred_buffer_(constants::kMaxBlockSize, constants::kMaxBlockSize) {
  std::vector<int> l1_mapping =
    ref_pic_list.GetSamePocMappingFor(RefPicList::kL1);
  assert(l1_mapping.size() <= same_poc_in_l0_mapping_.size());
  std::copy(l1_mapping.begin(), l1_mapping.end(),
            same_poc_in_l0_mapping_.begin());
}

Distortion
InterSearch::CompressInter(CodingUnit *cu, const Qp &qp,
                           const SyntaxWriter &bitstream_writer,
                           InterSearchFlags search_flags,
                           Cost best_cu_cost, TransformEncoder *encoder,
                           YuvPicture *rec_pic) {
  SampleBuffer &pred_buffer = encoder->GetPredBuffer(YuvComponent::kY);
  // 1st pass
  InterSearchFlags first_pass_flags = search_flags & ~InterSearchFlags::kAffine;
  Distortion best_cost =
    SearchMotion(cu, qp, bitstream_writer, first_pass_flags, &pred_buffer);
  // 2nd pass
  if ((search_flags & InterSearchFlags::kAffine) != InterSearchFlags(0)) {
    CodingUnit::InterState best_state;
    cu->SaveStateTo(&best_state);
    Distortion cost =
      SearchMotion(cu, qp, bitstream_writer, search_flags, &pred_buffer);
    if (best_cost <= cost) {
      cu->LoadStateFrom(best_state);
    }
  }
  if (cu->GetFullpelMv() && cu->HasZeroMvd()) {
    return std::numeric_limits<Distortion>::max();
  }
  return CompressAndEvalCbf(cu, qp, bitstream_writer, best_cu_cost,
                            encoder, rec_pic);
}

Distortion
InterSearch::CompressInterFast(CodingUnit *cu, YuvComponent comp, const Qp &qp,
                               const SyntaxWriter &bitstream_writer,
                               TransformEncoder *encoder, YuvPicture *rec_pic) {
  if (!cu->GetCbf(comp)) {
    // Write prediction directly to reconstruction
    SampleBuffer reco =
      rec_pic->GetSampleBuffer(comp, cu->GetPosX(comp), cu->GetPosY(comp));
    MotionCompensation(*cu, comp, &reco);
    return cu_metric_.CompareSample(*cu, comp, orig_pic_, reco);
  } else {
    SampleBuffer &pred = encoder->GetPredBuffer(comp);
    MotionCompensation(*cu, comp, &pred);
    return encoder->TransformAndReconstruct(cu, comp, qp, bitstream_writer,
                                            orig_pic_, rec_pic);
  }
}

Distortion
InterSearch::CompressMergeCand(CodingUnit *cu, const Qp &qp,
                               const SyntaxWriter &bitstream_writer,
                               const InterMergeCandidateList &merge_list,
                               int merge_idx, bool force_skip,
                               Cost best_cu_cost, TransformEncoder *encoder,
                               YuvPicture *rec_pic) {
  cu->SetSkipFlag(!force_skip ? false : true);
  cu->SetMergeIdx(merge_idx);
  ApplyMergeCand(cu, merge_list[merge_idx]);
  Distortion dist;
  if (!force_skip) {
    dist = CompressAndEvalCbf(cu, qp, bitstream_writer, best_cu_cost,
                              encoder, rec_pic);
  } else {
    dist = CompressSkipOnly(cu, qp, bitstream_writer, encoder, rec_pic);
  }
  if (Restrictions::Get().disable_inter_skip_mode) {
    cu->SetSkipFlag(false);
  }
  return dist;
}

Distortion
InterSearch::CompressAffineMerge(CodingUnit *cu, const Qp &qp,
                                 const SyntaxWriter &bitstream_writer,
                                 const AffineMergeCandidate &merge_cand,
                                 bool force_skip,
                                 Cost best_cu_cost, TransformEncoder *encoder,
                                 YuvPicture *rec_pic) {
  cu->SetSkipFlag(!force_skip ? false : true);
  cu->SetMergeIdx(0);
  ApplyMergeCand(cu, merge_cand);
  Distortion dist;
  if (!force_skip) {
    dist = CompressAndEvalCbf(cu, qp, bitstream_writer, best_cu_cost,
                              encoder, rec_pic);
  } else {
    dist = CompressSkipOnly(cu, qp, bitstream_writer, encoder, rec_pic);
  }
  if (Restrictions::Get().disable_inter_skip_mode) {
    cu->SetSkipFlag(false);
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
  SampleMetric metric(simd_.sample_metric, bitdepth_, MetricType::kSatd);
  SampleBuffer pred_buffer = encoder->GetPredBuffer(YuvComponent::kY);
  std::array<std::pair<int, double>, max_merge_cand> cand_cost;
  for (int merge_idx = 0; merge_idx < max_merge_cand; merge_idx++) {
    ApplyMergeCand(cu, merge_list[merge_idx]);
    MotionCompensation(*cu, YuvComponent::kY, &pred_buffer);
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

Distortion InterSearch::SearchMotion(CodingUnit *cu, const Qp &qp,
                                     const SyntaxWriter &bitstream_writer,
                                     InterSearchFlags search_flags,
                                     SampleBuffer *pred_buffer) {
  const YuvComponent comp = YuvComponent::kY;
  SampleBufferConst orig_luma =
    orig_pic_.GetSampleBuffer(comp, cu->GetPosX(comp), cu->GetPosY(comp));

  cu->ResetPredictionState();
  cu->SetPredMode(PredictionMode::kInter);
  if ((search_flags & InterSearchFlags::kFullPelMv) != InterSearchFlags(0)) {
    cu->SetFullpelMv(true);
  }
  if ((search_flags & InterSearchFlags::kLic) != InterSearchFlags(0)) {
    cu->SetUseLic(true);
  }
  if ((search_flags & InterSearchFlags::kAffine) != InterSearchFlags(0)) {
    assert(!cu->GetFullpelMv());
    assert(!cu->GetUseLic());
    cu->SetUseAffine(true);
  }

  CodingUnit::InterState state_l0;
  cu->SetInterDir(InterDir::kL0);
  Distortion cost_l0 = std::numeric_limits<Distortion>::max();
  cost_l0 = SearchRefIdx(cu, qp, RefPicList::kL0, bitstream_writer,
                         orig_luma, cost_l0, pred_buffer, &state_l0, nullptr);
  if ((search_flags & InterSearchFlags::kUniPredOnly) != InterSearchFlags(0)) {
    return cost_l0;
  }

  CodingUnit::InterState state_bi;
  CodingUnit::InterState state_l1_unique_poc;
  Distortion cost_l1_unique_poc;
  cu->SetInterDir(InterDir::kL1);
  Distortion cost_l1 = std::numeric_limits<Distortion>::max();
  cost_l1 = SearchRefIdx(cu, qp, RefPicList::kL1, bitstream_writer,
                         orig_luma, cost_l1, pred_buffer, &state_bi,
                         &state_l1_unique_poc, &cost_l1_unique_poc);

  // Prepare initial bi-prediction state with best from both lists
  assert(cu->GetInterDir() == InterDir::kL1);
  cu->LoadStateFrom(state_l0, RefPicList::kL0);
  InterDir best_uni_dir = cost_l0 <= cost_l1 ? InterDir::kL0 : InterDir::kL1;
  Distortion cost_best_bi =
    SearchBiIterative(cu, qp, bitstream_writer, best_uni_dir, pred_buffer,
                      &state_bi);

  Distortion best_cost;
  if (cost_best_bi <= cost_l0 && cost_best_bi <= cost_l1_unique_poc) {
    best_cost = cost_best_bi;
    cu->LoadStateFrom(state_bi);
  } else if (cost_l0 <= cost_l1_unique_poc) {
    best_cost = cost_l0;
    cu->LoadStateFrom(state_l0);
  } else {
    best_cost = cost_l1_unique_poc;
    cu->LoadStateFrom(state_l1_unique_poc);
  }
  return best_cost;
}

Distortion
InterSearch::CompressAndEvalCbf(CodingUnit *cu, const Qp &qp,
                                const SyntaxWriter &bitstream_writer,
                                Cost best_cu_cost,
                                TransformEncoder *encoder,
                                YuvPicture *rec_pic) {
  auto get_zero_cost = [&](Distortion dist) {
    RdoSyntaxWriter rdo_writer_zero(bitstream_writer, 0);
    // Note that root cbf is not used in case of merge/skip
    rdo_writer_zero.WriteRootCbf(false);
    Bits bits_zero = rdo_writer_zero.GetNumWrittenBits();
    return dist + static_cast<Cost>(bits_zero * qp.GetLambda() + 0.5);
  };
  std::array<TransformEncoder::RdCost, constants::kMaxYuvComponents> best_cost;
  std::array<Distortion, constants::kMaxYuvComponents> comp_dist_zero;
  Distortion sum_dist_resi = 0;
  Distortion sum_dist_final = 0;
  Distortion sum_dist_zero = 0;

  TxSearchFlags tx_rd_flags = TxSearchFlags::kFullEval;
  int nbr_tx_passes = 1;
  if (encoder_settings_.fast_transform_select_eval) {
    tx_rd_flags &= ~TxSearchFlags::kTransformSelect;
    nbr_tx_passes = 2;
  }

  for (int tx_pass = 0; tx_pass < nbr_tx_passes; tx_pass++) {
    bool modified = false;
    for (int c = 0; c < max_components_; c++) {
      const YuvComponent comp = YuvComponent(c);
      SampleBuffer &pred_buffer = encoder->GetPredBuffer(comp);
      if (tx_pass == 0) {
        MotionCompensation(*cu, comp, &pred_buffer);
      }
      Cost *best_cost_comp = tx_pass == 0 ? nullptr : &best_cost[c].cost;
      // TODO(PH) Should update contexts after each component for rdo quant
      TransformEncoder::RdCost tx_cost =
        encoder->CompressAndEvalTransform(cu, comp, qp, bitstream_writer,
                                          orig_pic_, tx_rd_flags,
                                          best_cost_comp, &comp_dist_zero[c],
                                          &cu_writer_, rec_pic);
      if (tx_pass == 0) {
        sum_dist_resi += tx_cost.dist_resi;
        sum_dist_final += tx_cost.dist_reco;
        sum_dist_zero += comp_dist_zero[c];
        best_cost[c] = tx_cost;
      } else if (tx_cost.cost < best_cost[c].cost) {
        sum_dist_resi -= best_cost[c].dist_resi;
        sum_dist_resi += tx_cost.dist_resi;
        sum_dist_final -= best_cost[c].dist_reco;
        sum_dist_final += tx_cost.dist_reco;
        best_cost[c] = tx_cost;
        modified = true;
      }
    }
    cu->SetRootCbf(cu->GetHasAnyCbf());
    cu->SetSkipFlag(cu->GetMergeFlag() && !cu->GetRootCbf());

    // Evaluate root cbf zero
    if ((tx_pass == 0 || modified) &&
        !Restrictions::Get().disable_transform_cbf &&
        !Restrictions::Get().disable_transform_root_cbf) {
      Bits bits_non_zero =
        encoder->GetCuBitsResidual(*cu, bitstream_writer, &cu_writer_);
      Cost cost_non_zero =
        sum_dist_resi + static_cast<Cost>(bits_non_zero * qp.GetLambda() + 0.5);
      Cost cost_zero = get_zero_cost(sum_dist_zero);
      if (cost_zero < cost_non_zero) {
        sum_dist_resi = sum_dist_zero;
        sum_dist_final = sum_dist_zero;
        for (int c = 0; c < max_components_; c++) {
          best_cost[c].dist_resi = comp_dist_zero[c];
          best_cost[c].dist_reco = comp_dist_zero[c];
          const YuvComponent comp = YuvComponent(c);
          cu->ClearCbf(comp);
          const SampleBuffer &pred_buffer = encoder->GetPredBuffer(comp);
          SampleBuffer reco = rec_pic->GetSampleBuffer(comp, cu->GetPosX(comp),
                                                       cu->GetPosY(comp));
          reco.CopyFrom(cu->GetWidth(comp), cu->GetHeight(comp), pred_buffer);
        }
        cu->SetRootCbf(cu->GetHasAnyCbf());
        cu->SetSkipFlag(cu->GetMergeFlag() && !cu->GetRootCbf());
      }
    }

    // Early termination of 2nd pass
    if (encoder_settings_.fast_transform_select_eval) {
      if (!cu->GetCbf(YuvComponent::kY)) {
        break;
      }
      Bits bits_full =
        encoder->GetCuBitsFull(*cu, bitstream_writer, &cu_writer_);
      // TODO(PH) Consider using final distortion here
      Cost cost_full =
        sum_dist_resi + static_cast<Cost>(bits_full * qp.GetLambda() + 0.5);
      if (cost_full > best_cu_cost * kFastTransformSelectCostFactor) {
        break;
      }
      // Only evaluate transform types in second pass
      tx_rd_flags = TxSearchFlags::kTransformSelect;
    }
  }

  return sum_dist_final;
}

Distortion
InterSearch::CompressSkipOnly(CodingUnit *cu, const Qp &qp,
                              const SyntaxWriter &bitstream_writer,
                              TransformEncoder *encoder, YuvPicture *rec_pic) {
  assert(cu->GetPredMode() == PredictionMode::kInter);
  if (!Restrictions::Get().disable_inter_skip_mode) {
    cu->SetSkipFlag(true);
  }
  cu->SetRootCbf(false);

  Distortion sum_dist = 0;
  for (int c = 0; c < max_components_; c++) {
    const YuvComponent comp = YuvComponent(c);
    int posx = cu->GetPosX(comp);
    int posy = cu->GetPosY(comp);
    SampleBuffer reco_buffer = rec_pic->GetSampleBuffer(comp, posx, posy);
    MotionCompensation(*cu, comp, &reco_buffer);
    cu->ClearCbf(comp);
    sum_dist += cu_metric_.CompareSample(*cu, comp, orig_pic_, reco_buffer);
  }
  return sum_dist;
}

Distortion
InterSearch::SearchBiIterative(CodingUnit *cu, const Qp &qp,
                               const SyntaxWriter &bitstream_writer,
                               InterDir best_uni_dir, SampleBuffer *pred_buffer,
                               CodingUnit::InterState *best_state) {
  const YuvComponent comp = YuvComponent::kY;
  SampleBufferConst orig_luma =
    orig_pic_.GetSampleBuffer(comp, cu->GetPosX(comp), cu->GetPosY(comp));
  int width = cu->GetWidth(comp);
  int height = cu->GetHeight(comp);
  cu->SetInterDir(InterDir::kBi);

  // Start searching the second best list
  RefPicList search_list =
    best_uni_dir == InterDir::kL0 ? RefPicList::kL1 : RefPicList::kL0;

  Distortion cost_best = std::numeric_limits<Distortion>::max();
  int num_iterations = encoder_settings_.bipred_refinement_iterations;
  if (cu->GetPicData()->GetForceBipredL1MvdZero()) {
    num_iterations = 1;
    search_list = RefPicList::kL0;
  }
  for (int iteration = 0; iteration < num_iterations; iteration++) {
    // If searching in L1 use original without L0 prediction
    cu->SetInterDir(search_list == RefPicList::kL0 ?
                    InterDir::kL1 : InterDir::kL0);
    MotionCompensation(*cu, comp, &bipred_pred_buffer_);
    bipred_orig_buffer_.SubtractWeighted(width, height, orig_luma,
                                         bipred_pred_buffer_);
    cu->SetInterDir(InterDir::kBi);

    Distortion prev_best = cost_best;
    cost_best =
      SearchRefIdx(cu, qp, search_list, bitstream_writer, bipred_orig_buffer_,
                   cost_best, pred_buffer, best_state);
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
                          Distortion initial_best_cost,
                          SampleBuffer *pred_buffer,
                          CodingUnit::InterState *best_state,
                          CodingUnit::InterState *best_state_unique,
                          Distortion *out_cost_unique) {
  if (cu->GetUseAffine()) {
    return SearchRefIdx<true, MotionVector3>(
      cu, qp, ref_list, bitstream_writer, orig_buffer, initial_best_cost,
      pred_buffer, best_state, best_state_unique, out_cost_unique);
  } else {
    return SearchRefIdx<false, MotionVector>(
      cu, qp, ref_list, bitstream_writer, orig_buffer, initial_best_cost,
      pred_buffer, best_state, best_state_unique, out_cost_unique);
  }
}

template<bool IsAffine, typename MotionVec, typename TOrig>
Distortion
InterSearch::SearchRefIdx(CodingUnit *cu, const Qp &qp, RefPicList ref_list,
                          const SyntaxWriter &bitstream_writer,
                          const DataBuffer<TOrig> &orig_buffer,
                          Distortion initial_best_cost,
                          SampleBuffer *pred_buffer,
                          CodingUnit::InterState *best_state,
                          CodingUnit::InterState *best_state_unique,
                          Distortion *out_cost_unique) {
  const int num_ref_idx = cu->GetRefPicLists()->GetNumRefPics(ref_list);
  const uint32_t lambda =
    static_cast<uint32_t>(std::floor(65536.0 * qp.GetLambdaSqrt()));
  const bool bipred = cu->GetInterDir() == InterDir::kBi;
  const bool force_mvd_zero = cu->GetPicData()->GetForceBipredL1MvdZero() &&
    ref_list == RefPicList::kL1;
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
  assert(!(bipred && force_mvd_zero));

  for (int ref_idx = 0; ref_idx < num_ref_idx; ref_idx++) {
    const bool unique_ref_pic = ref_list == RefPicList::kL1 &&
      same_poc_in_l0_mapping_[ref_idx] < 0;
    cu->SetRefIdx(ref_idx, ref_list);

    std::array<MotionVec, constants::kNumInterMvPredictors> mvp_list;
    mvp_list =
      GetMvpListHelper<IsAffine, MotionVec>(*cu, ref_list, ref_idx,
                                            constants::kNumInterMvPredictors);
    int mvp_idx;
    const MotionVec *mv_bootstrap = nullptr;
    MotionVector3 mv_bootstrap_tmp;
    if (bipred) {
      // Reuse start mv predictor from uni-pred search
      mvp_idx = unipred_best_mvp_idx_[static_cast<int>(ref_list)][ref_idx];
      mv_bootstrap = &GetBestUniPredMv<IsAffine, MotionVec>(ref_list, ref_idx);
    } else {
      // Determine best start mv predictor
      const YuvPicture *ref_pic =
        cu->GetRefPicLists()->GetRefPic(ref_list, ref_idx);
      Distortion mvp_cost = std::numeric_limits<Distortion>::max();
      mvp_idx = EvalStartMvp<IsAffine>(*cu, qp, mvp_list, *ref_pic,
                                       pred_buffer, &mvp_cost);
      if (force_mvd_zero) {
        if (mvp_cost < cost_best) {
          cu->SetRefIdx(ref_idx, ref_list);
          cu->SetMvpIdx(mvp_idx, ref_list);
          cu->SetMv(mvp_list[mvp_idx], ref_list);
          if (IsAffine) {
            cu->SetMvdAffine(0, MvDelta(0, 0), ref_list);
            cu->SetMvdAffine(1, MvDelta(0, 0), ref_list);
          } else {
            cu->SetMvDelta(MvDelta(0, 0), ref_list);
          }
          cost_best = mvp_cost;
          cu->SaveStateTo(best_state);
        }
        if (bipred || !unique_ref_pic) {
          continue;
        }
      }
      if (IsAffine) {
        // Bootstrap search based on best non-affine search mv
        const MotionVector &mv_normal =
          unipred_best_mv_[static_cast<int>(ref_list)][ref_idx];
        mv_bootstrap_tmp = DeriveMvAffine(*cu, *ref_pic, mv_normal, mv_normal);
        mv_bootstrap = reinterpret_cast<MotionVec*>(&mv_bootstrap_tmp);
      }
    }
    Distortion dist = 0;
    // Search best motion vector
    MotionVec mv;
    if (!bipred && !unique_ref_pic && ref_list == RefPicList::kL1) {
      // Encoder speed-up for already searched ref pictures in L0
      int l0_list_idx = static_cast<int>(RefPicList::kL0);
      int l0_ref_idx = same_poc_in_l0_mapping_[ref_idx];
      mv = GetBestUniPredMv<IsAffine, MotionVec>(RefPicList::kL0, l0_ref_idx);
      dist = unipred_best_dist_[l0_list_idx][l0_ref_idx];
      // TODO(Dev) also update previous_fullpel_ to seed search for next CU?
    } else {
      mv = MotionEstimation(*cu, qp, search_method, ref_list, ref_idx, bipred,
                            orig_buffer, mvp_list[mvp_idx], mv_bootstrap,
                            pred_buffer, &dist);
    }
    // Refine best start mv predictor
    mvp_idx = EvalFinalMvpIdx(*cu, mvp_list, mv, mvp_idx);
    if (!bipred || encoder_settings_.bipred_refinement_iterations > 1) {
      SetBestUniPredMv<IsAffine>(ref_list, ref_idx, mv);
      unipred_best_mvp_idx_[static_cast<int>(ref_list)][ref_idx] = mvp_idx;
      unipred_best_dist_[static_cast<int>(ref_list)][ref_idx] = dist;
    }

    cu->SetMvpIdx(mvp_idx, ref_list);
    cu->SetMv(mv, ref_list);
    SetMvd(cu, ref_list, mvp_list[mvp_idx], mv);

    // If using fullpel mv we might end up with a zero mvd,
    // which is invalid for uni-prediction but might be ok for bi-prediction
    Bits bits = GetInterPredBits(*cu, bitstream_writer);
    Distortion cost = dist + ((bits * lambda) >> 16);
    if (!force_mvd_zero && cost < cost_best) {
      cost_best = cost;
      cu->SaveStateTo(best_state);
    }
    if (best_state_unique && unique_ref_pic && cost < cost_best_unique) {
      cost_best_unique = cost;
      cu->SaveStateTo(best_state_unique);
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
                              RefPicList ref_list, int ref_idx, bool bipred,
                              const DataBuffer<TOrig> &orig_buffer,
                              const MotionVector &mvp,
                              const MotionVector *mv_bootstrap,
                              SampleBuffer *pred_buffer, Distortion *out_dist) {
  return MotionEstNormal(cu, qp, search_method, ref_list, ref_idx, bipred,
                         orig_buffer, mvp, mv_bootstrap, pred_buffer, out_dist);
}

template<typename TOrig>
MotionVector3
InterSearch::MotionEstimation(const CodingUnit &cu, const Qp &qp,
                              SearchMethod search_method, RefPicList ref_list,
                              int ref_idx, bool bipred,
                              const DataBuffer<TOrig>& orig_buffer,
                              const MotionVector3 &mvp,
                              const MotionVector3 *mv_bootstrap,
                              SampleBuffer *pred_buffer, Distortion *out_dist) {
  return MotionEstAffine(cu, qp, search_method, ref_list, ref_idx, bipred,
                         orig_buffer, mvp, mv_bootstrap, pred_buffer, out_dist);
}

template<typename TOrig>
MotionVector
InterSearch::MotionEstNormal(const CodingUnit &cu, const Qp &qp,
                             SearchMethod search_method,
                             RefPicList ref_list, int ref_idx, bool bipred,
                             const DataBuffer<TOrig> &orig_buffer,
                             const MotionVector &mvp,
                             const MotionVector *mv_bootstrap,
                             SampleBuffer *pred_buffer, Distortion *out_dist) {
  const YuvPicture *ref_pic =
    cu.GetRefPicLists()->GetRefPic(ref_list, ref_idx);
  const PicNum ref_poc =
    cu.GetRefPicLists()->GetRefPoc(ref_list, ref_idx);
  const int search_range = search_method == SearchMethod::FullSearch ?
    encoder_settings_.inter_search_range_bi : GetSearchRangeUniPred(ref_poc);
  MvFullpel clip_min, clip_max;
  if (!mv_bootstrap) {
    DetermineMinMaxMv(cu, *ref_pic, mvp, search_range, &clip_min, &clip_max);
  } else {
    DetermineMinMaxMv(cu, *ref_pic, *mv_bootstrap, search_range,
                      &clip_min, &clip_max);
  }

  MvFullpel mv_fullpel;
  SampleMetric fullpel_metric(simd_.sample_metric, bitdepth_,
                              GetFullpelMetric(cu));
  if (search_method == SearchMethod::FullSearch) {
    mv_fullpel =
      FullSearch(cu, qp, fullpel_metric, mvp, *ref_pic, clip_min, clip_max);
  } else if (search_method == SearchMethod::TzSearch) {
    TzSearch tz_search(orig_pic_, *this, encoder_settings_, search_range);
    mv_fullpel =
      tz_search.Search(cu, qp, fullpel_metric, mvp, *ref_pic,
                       clip_min, clip_max,
                       previous_fullpel_[static_cast<int>(ref_list)][ref_idx]);
    previous_fullpel_[static_cast<int>(ref_list)][ref_idx] = mv_fullpel;
  } else {
    assert(0);
  }

  MotionVector mv_subpel;

  SampleMetric subpel_metric(simd_.sample_metric, bitdepth_,
                             GetSubpelMetric(cu));
  Distortion dist = std::numeric_limits<Distortion>::max();
  if (cu.GetFullpelMv()) {
    mv_subpel = MotionVector(mv_fullpel);
    dist = GetSubpelDist(cu, qp, *ref_pic, subpel_metric, mv_subpel,
                         orig_buffer, pred_buffer);
  } else {
    mv_subpel =
      SubpelSearch(cu, qp, subpel_metric, *ref_pic, mvp, mv_fullpel,
                   orig_buffer, pred_buffer, &dist);
  }
  *out_dist = bipred ? (dist >> 1) : dist;
  return mv_subpel;
}

template<typename TOrig>
MotionVector3
InterSearch::MotionEstAffine(const CodingUnit &cu, const Qp &qp,
                             SearchMethod search_method, RefPicList ref_list,
                             int ref_idx, bool bipred,
                             const DataBuffer<TOrig>& orig_buffer,
                             const MotionVector3 &mvp,
                             const MotionVector3 *mv_bootstrap,
                             SampleBuffer *pred_buffer, Distortion *out_dist) {
  const YuvComponent comp = YuvComponent::kY;
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const uint32_t lambda =
    static_cast<uint32_t>(std::floor(65536.0 * qp.GetLambdaSqrt()));
  const YuvPicture *ref_pic = cu.GetRefPicLists()->GetRefPic(ref_list, ref_idx);
  const bool force_mv_bootstrap = bipred;   // TODO(PH) Potential gains?
  const int bi_dist_shift = bipred ? 1 : 0;
  const int max_iterations = bipred ? 5 : 7;
  SampleMetric metric_mvp(simd_.sample_metric, bitdepth_, GetMvpMetricType(cu));
  SampleMetric metric(simd_.sample_metric, bitdepth_, GetFullpelMetric(cu));
  ResidualBufferStorage err_buffer(constants::kMaxBlockSize,
                                   constants::kMaxBlockSize);

  // Start search around mvp
  // TODO(PH) Dist for mvp has already been calculated during mvp selection
  MotionVector3 best_mv = mvp;
  MotionCompensationMv(cu, comp, *ref_pic, mvp, false, pred_buffer);
  Distortion best_dist = metric_mvp.CompareSample(qp, comp, width, height,
                                                  orig_buffer, *pred_buffer);
  Bits mvp_bits = GetMvdBits(mvp, best_mv, 0);
  Distortion best_cost =
    (best_dist >> bi_dist_shift) + ((lambda * mvp_bits) >> 16);

  if (mv_bootstrap && *mv_bootstrap != best_mv) {
    // Check extra start position
    const MotionVector3 &mv = *mv_bootstrap;
    MotionCompensationMv(cu, comp, *ref_pic, mv, false, pred_buffer);
    Distortion dist = metric_mvp.CompareSample(qp, comp, width, height,
                                               orig_buffer, *pred_buffer);
    Bits bits = GetMvdBits(mvp, mv, 0);
    Distortion cost = (dist >> bi_dist_shift) + ((lambda * bits) >> 16);
    if (cost < best_cost || force_mv_bootstrap) {
      best_cost = cost;
      best_dist = dist;
      best_mv = mv;
    } else {
      // TODO(PH) Maybe saving prediction samples instead of re-calculating
      MotionCompensationMv(cu, comp, *ref_pic, best_mv, false, pred_buffer);
    }
  }

  // TODO(PH) This is redundant to use different metrics above and below
  best_dist =
    metric.CompareSample(qp, comp, width, height, orig_buffer, *pred_buffer);
  mvp_bits = GetMvdBits(mvp, best_mv, 0);
  best_cost = (best_dist >> bi_dist_shift) + ((lambda * mvp_bits) >> 16);

  // Gradient search
  MotionVector3 mv = best_mv;
  for (int iter = 0; iter < max_iterations; iter++) {
    err_buffer.Subtract(width, height, orig_buffer, *pred_buffer);
    MvDelta2 mvd =
      AffineGradientSearch(width, height, *pred_buffer, err_buffer);
    if (mvd[0].x == 0 && mvd[0].y == 0 && mvd[1].x == 0 && mvd[1].y == 0) {
      break;
    }
    // Update mv relative to what was used in last prediction
    mv[0] += mvd[0];
    mv[1] += mvd[1];
    mv = DeriveMvAffine(cu, *ref_pic, mv[0], mv[1]);

    MotionCompensationMv(cu, comp, *ref_pic, mv, false, pred_buffer);
    Distortion dist =
      metric.CompareSample(qp, comp, width, height, orig_buffer, *pred_buffer);
    Bits bits = GetMvdBits(mvp, mv, 0);
    Distortion cost = (dist >> bi_dist_shift) + ((lambda * bits) >> 16);

    if (cost < best_cost) {
      best_cost = cost;
      best_dist = dist;
      best_mv = mv;
    }
  }
  if (out_dist) {
    *out_dist = best_dist >> bi_dist_shift;
  }
  return best_mv;
}

MvDelta2
InterSearch::AffineGradientSearch(int width, int height,
                                  const SampleBuffer &pred_buffer,
                                  const ResidualBuffer &err_buffer) {
  static const int kNbrParams = 4;
  const ptrdiff_t pred_stride = pred_buffer.GetStride();
  const Sample *pred = pred_buffer.GetDataPtr();
  pred += pred_stride;
  for (int y = 1; y < height - 1; y++) {
    for (int x = 1; x < width - 1; x++) {
      Sample a0 = pred[x - pred_stride - 1];
      Sample a1 = pred[x - pred_stride];
      Sample a2 = pred[x - pred_stride + 1];
      Sample b0 = pred[x - 1];
      Sample b2 = pred[x + 1];
      Sample c0 = pred[x + pred_stride - 1];
      Sample c1 = pred[x + pred_stride];
      Sample c2 = pred[x + pred_stride + 1];
      affine_delta_hor_[y][x] = (-a0 + a2 - 2 * b0 + 2 * b2 - c0 + c2) / 8.0f;
      affine_delta_ver_[y][x] = (-a0 - 2 * a1 - a2 + c0 + 2 * c1 + c2) / 8.0f;
    }
    affine_delta_hor_[y][0] = affine_delta_hor_[y][1];
    affine_delta_hor_[y][width - 1] = affine_delta_hor_[y][width - 2];
    affine_delta_ver_[y][0] = affine_delta_ver_[y][1];
    affine_delta_ver_[y][width - 1] = affine_delta_ver_[y][width - 2];
    pred += pred_stride;
  }
  for (int x = 0; x < width; x++) {
    affine_delta_hor_[0][x] = affine_delta_hor_[1][x];
    affine_delta_hor_[height - 1][x] = affine_delta_hor_[height - 2][x];
    affine_delta_ver_[0][x] = affine_delta_ver_[1][x];
    affine_delta_ver_[height - 1][x] = affine_delta_ver_[height - 2][x];
  }

  std::array<std::array<double, kNbrParams + 1>, kNbrParams> matrix = { { 0 } };
  const Residual *err = err_buffer.GetDataPtr();
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      const double c[4] = {
        affine_delta_hor_[y][x],
        x * affine_delta_hor_[y][x] + y * affine_delta_ver_[y][x],
        affine_delta_ver_[y][x],
        y * affine_delta_hor_[y][x] - x * affine_delta_ver_[y][x],
      };
      for (int row = 0; row < kNbrParams; row++) {
        for (int col = 0; col < kNbrParams; col++) {
          matrix[row][col] += c[row] * c[col];
        }
        matrix[row][kNbrParams] += err[x] * c[row];
      }
    }
    err += err_buffer.GetStride();
  }

  // solve linear equation system using row echelon form
  for (int i = 0; i < kNbrParams - 1; i++) {
    int best_index = i;
    double best_val = std::abs(matrix[i][i]);
    for (int j = i + 1; j < kNbrParams; j++) {
      if (std::abs(matrix[j][i]) > best_val) {
        best_index = j;
        best_val = std::abs(matrix[j][i]);
      }
    }
    if (best_index != i) {
      for (int col = 0; col < kNbrParams + 1; col++) {
        std::swap(matrix[i][col], matrix[best_index][col]);
      }
    }
    // reduce
    for (int j = i + 1; j < kNbrParams; j++) {
      for (int k = i + 1; k < kNbrParams + 1; k++) {
        if (matrix[i][i]) {
          matrix[j][k] -= matrix[i][k] * matrix[j][i] / matrix[i][i];
        }
      }
    }
  }

  std::array<double, kNbrParams> params = { 0 };
  if (matrix[kNbrParams - 1][kNbrParams - 1]) {
    params[kNbrParams - 1] = matrix[kNbrParams - 1][kNbrParams] /
      matrix[kNbrParams - 1][kNbrParams - 1];
  }
  for (int row = kNbrParams - 2; row >= 0; row--) {
    double sum = 0;
    for (int col = row + 1; col < kNbrParams; col++) {
      sum += matrix[row][col] * params[col];
    }
    if (matrix[row][row]) {
      params[row] = (matrix[row][kNbrParams] - sum) / matrix[row][row];
    }
  }
  static const int kMvdScale = 1 << MvDelta::kPrecisionShift;
  MvDelta2 mvd;
  mvd[0].x = ::lround(kMvdScale * params[0]);
  mvd[0].y = ::lround(kMvdScale * params[2]);
  mvd[1].x = ::lround(kMvdScale * (params[1] * width + params[0]));
  mvd[1].y = ::lround(kMvdScale * (-params[3] * width + params[2]));
  return mvd;
}

MvFullpel InterSearch::FullSearch(const CodingUnit &cu, const Qp &qp,
                                  const SampleMetric &metric,
                                  const MotionVector &mvp,
                                  const YuvPicture &ref_pic,
                                  const MvFullpel &mv_min,
                                  const MvFullpel &mv_max) {
  const YuvComponent comp = YuvComponent::kY;
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const int mvd_precision =
    cu.GetFullpelMv() ? MvDelta::kPrecisionShift : 0;
  const uint32_t lambda =
    static_cast<uint32_t>(std::floor(65536.0 * qp.GetLambdaSqrt()));
  const Sample *ref_cu = ref_pic.GetSamplePtr(comp, cu.GetPosX(comp),
                                              cu.GetPosY(comp));
  intptr_t ref_stride = ref_pic.GetStride(comp);
  Distortion cost_best = std::numeric_limits<Distortion>::max();
  MvFullpel mv_best;
  for (int mv_y = mv_min.y; mv_y <= mv_max.y; mv_y++) {
    for (int mv_x = mv_min.x; mv_x <= mv_max.x; mv_x++) {
      const Sample *ref_mv = ref_cu + mv_y * ref_stride + mv_x;
      Distortion dist = metric.CompareSample(qp, comp, width, height,
                                             bipred_orig_buffer_.GetDataPtr(),
                                             bipred_orig_buffer_.GetStride(),
                                             ref_mv, ref_stride);
      if (dist >= cost_best) {
        continue;
      }
      Bits bits = GetMvdBitsFullpel(mvp, mv_x, mv_y, mvd_precision);
      Distortion cost = dist + ((lambda * bits) >> 16);
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
                          const SampleMetric &metric,
                          const YuvPicture &ref_pic, const MotionVector &mvp,
                          const MvFullpel &mv_fullpel,
                          const DataBuffer<TOrig> &orig_buffer,
                          SampleBuffer *pred_buffer, Distortion *out_dist) {
  uint32_t lambda =
    static_cast<uint32_t>(std::floor(65536.0 * qp.GetLambdaSqrt()));
  Distortion best_cost = std::numeric_limits<Distortion>::max();
  Distortion best_dist = best_cost;
  MotionVector best_mv = MotionVector(mv_fullpel);
  assert(!cu.GetFullpelMv());

  // Half-pel
  MotionVector mv_base = best_mv;
  for (int i = 0; i < static_cast<int>(kSquareXYHalf.size()); i++) {
    const MvDelta mvd(kSquareXYHalf[i][0], kSquareXYHalf[i][1], 1);
    const MotionVector mv = mv_base + mvd;
    Distortion dist = GetSubpelDist(cu, qp, ref_pic, metric, mv,
                                    orig_buffer, pred_buffer);
    if (dist >= best_cost) {
      continue;
    }
    Bits bits = GetMvdBits(mvp, mv, 0);
    Distortion cost = dist + ((lambda * bits) >> 16);
    if (cost < best_cost) {
      best_cost = cost;
      best_dist = dist;
      best_mv = mv;
    }
  }

  // Qpel
  mv_base = best_mv;
  for (int i = 1; i < static_cast<int>(kSquareXYQpel.size()); i++) {
    const MvDelta mvd(kSquareXYQpel[i][0], kSquareXYQpel[i][1], 2);
    const MotionVector mv = mv_base + mvd;
    Distortion dist = GetSubpelDist(cu, qp, ref_pic, metric, mv,
                                    orig_buffer, pred_buffer);
    if (dist >= best_cost) {
      continue;
    }
    Bits bits = GetMvdBits(mvp, mv, 0);
    Distortion cost = dist + ((lambda * bits) >> 16);
    if (cost < best_cost) {
      best_cost = cost;
      best_dist = dist;
      best_mv = mv;
    }
  }
  if (out_dist) {
    *out_dist = best_dist;
  }
  return best_mv;
}

template<typename TOrig>
Distortion
InterSearch::GetSubpelDist(const CodingUnit &cu, const Qp &qp,
                           const YuvPicture &ref_pic,
                           const SampleMetric &metric, const MotionVector &mv,
                           const DataBuffer<TOrig> &orig_buffer,
                           SampleBuffer *pred_buffer) {
  const YuvComponent comp = YuvComponent::kY;
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  MotionCompensationMv(cu, comp, ref_pic, mv, false, pred_buffer);
  return
    metric.CompareSample(qp, comp, width, height, orig_buffer, *pred_buffer);
}

template<bool IsAffine, typename MotionVec>
int InterSearch::EvalStartMvp(const CodingUnit &cu, const Qp &qp,
                              const std::array<MotionVec, kNumMvp> &mvp_list,
                              const YuvPicture &ref_pic,
                              SampleBuffer *pred_buffer, Distortion *out_cost) {
  SampleMetric metric(simd_.sample_metric, bitdepth_, GetMvpMetricType(cu));
  uint32_t lambda =
    static_cast<uint32_t>(std::floor(65536.0 * qp.GetLambdaSqrt()));
  int best_mvp_idx = 0;
  Distortion best_cost = std::numeric_limits<Distortion>::max();
  for (int i = 0; i < static_cast<int>(mvp_list.size()); i++) {
    auto mv = mvp_list[i];
    ClipMv(cu, ref_pic, &mv);   // TODO(PH) Is clip really needed here?
    MotionCompensationMv(cu, YuvComponent::kY, ref_pic, mv, true, pred_buffer);
    Distortion dist = metric.CompareSample(cu, YuvComponent::kY, orig_pic_,
                                           *pred_buffer);
    Bits bits = GetMvpBits(i, static_cast<int>(mvp_list.size()));
    Distortion cost = dist + (static_cast<uint32_t>(bits * lambda + 0.5) >> 16);
    if (cost < best_cost) {
      best_cost = cost;
      best_mvp_idx = i;
    }
    if ((!IsAffine && Restrictions::Get().disable_inter_mvp) ||
      (IsAffine &&  Restrictions::Get().disable_ext2_inter_affine_mvp)) {
      break;
    }
  }
  if (out_cost) {
    *out_cost = best_cost;
  }
  return best_mvp_idx;
}

template<typename MotionVec>
int InterSearch::EvalFinalMvpIdx(const CodingUnit &cu,
                                 const std::array<MotionVec, kNumMvp> &mvp_list,
                                 const MotionVec &mv, int mvp_idx_start) {
  if ((!cu.GetUseAffine() && Restrictions::Get().disable_inter_mvp) ||
    (cu.GetUseAffine() && Restrictions::Get().disable_ext2_inter_affine_mvp)) {
    return 0;
  }
  const int mvd_precision =
    cu.GetFullpelMv() ? MvDelta::kPrecisionShift : 0;
  int best_mvp_idx = 0;
  Bits best_cost = std::numeric_limits<Bits>::max();
  for (int i = 0; i < static_cast<int>(mvp_list.size()); i++) {
    Bits cost = GetMvpBits(i, static_cast<int>(mvp_list.size()));
    cost += GetMvdBits(mvp_list[i], mv, mvd_precision);
    if (cost < best_cost || (cost == best_cost && i == mvp_idx_start)) {
      best_cost = cost;
      best_mvp_idx = i;
    }
  }
  return best_mvp_idx;
}

template<>
void InterSearch::SetMvd<MotionVector>(CodingUnit *cu, RefPicList ref_list,
                                       const MotionVector &mvp,
                                       const MotionVector &mv) {
  MvDelta mvd = mv - mvp;
  if (cu->GetFullpelMv()) {
    mvd.x >>= MvDelta::kPrecisionShift;
    mvd.y >>= MvDelta::kPrecisionShift;
  }
  cu->SetMvDelta(mvd, ref_list);
}

template<>
void InterSearch::SetMvd<MotionVector3>(CodingUnit *cu, RefPicList ref_list,
                                        const MotionVector3 &mvp,
                                        const MotionVector3 &mv) {
  MvDelta mvd0 = mv[0] - mvp[0];
  MvDelta mvd1 = mv[1] - mvp[1];
  if (cu->GetFullpelMv()) {
    mvd0.x >>= MvDelta::kPrecisionShift;
    mvd0.y >>= MvDelta::kPrecisionShift;
    mvd1.x >>= MvDelta::kPrecisionShift;
    mvd1.y >>= MvDelta::kPrecisionShift;
  }
  cu->SetMvdAffine(0, mvd0, ref_list);
  cu->SetMvdAffine(1, mvd1, ref_list);
}

int InterSearch::GetSearchRangeUniPred(PicNum ref_poc) const {
  const int max = encoder_settings_.inter_search_range_uni_max;
  const int min = encoder_settings_.inter_search_range_uni_min;
  const int delta_poc = static_cast<int>(poc_ - ref_poc);
  const int search_range =
    (max * std::abs(delta_poc) + (sub_gop_length_ / 2)) / sub_gop_length_;
  return util::Clip3(search_range, min, max);
}

MetricType InterSearch::GetFullpelMetric(const CodingUnit &cu) const {
  if (cu.GetUseAffine()) {
    return MetricType::kSatd;
  }
  if (cu.GetUseLic()) {
    return cu.GetHeight(YuvComponent::kY) > 8 ?
      MetricType::kSadAcOnlyFast : MetricType::kSadAcOnly;
  }
  return cu.GetHeight(YuvComponent::kY) > 8 ?
    MetricType::kSadFast : MetricType::kSad;
}

MetricType InterSearch::GetSubpelMetric(const CodingUnit &cu) const {
  if (cu.GetUseLic()) {
    return MetricType::kSatdAcOnly;
  }
  return MetricType::kSatd;
}

MetricType InterSearch::GetMvpMetricType(const CodingUnit &cu) const {
  return MetricType::kSad;
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
      if (cu.GetUseAffine()) {
        bits += GetNumExpGolombBits(cu.GetMvdAffine(0, ref_list).x);
        bits += GetNumExpGolombBits(cu.GetMvdAffine(0, ref_list).y);
        bits += GetNumExpGolombBits(cu.GetMvdAffine(1, ref_list).x);
        bits += GetNumExpGolombBits(cu.GetMvdAffine(1, ref_list).y);
      } else {
        bits += GetNumExpGolombBits(cu.GetMvDelta(ref_list).x);
        bits += GetNumExpGolombBits(cu.GetMvDelta(ref_list).y);
      }
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
        if (cu.GetForceMvdZero(ref_list)) {
          continue;
        }
        if (cu.GetUseAffine()) {
          bits += GetNumExpGolombBits(cu.GetMvdAffine(0, ref_list).x);
          bits += GetNumExpGolombBits(cu.GetMvdAffine(0, ref_list).y);
          bits += GetNumExpGolombBits(cu.GetMvdAffine(1, ref_list).x);
          bits += GetNumExpGolombBits(cu.GetMvdAffine(1, ref_list).y);
        } else {
          bits += GetNumExpGolombBits(cu.GetMvDelta(ref_list).x);
          bits += GetNumExpGolombBits(cu.GetMvDelta(ref_list).y);
        }
      }
      return bits;
    }
    // TODO(PH) Consider adding fullpel mv flag for completeness
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

Bits InterSearch::GetMvdBits(const MotionVector &mvp, const MotionVector &mv,
                             int mvd_down_shift) {
  // High precision down to mvd precision
  const int mv_to_mvd_shift =
    MotionVector::kPrecisionShift - MvDelta::kPrecisionShift;
  static_assert(mv_to_mvd_shift >= 0, "Negative mvd precision shift");
  int mvd_x = (mv.x - mvp.x) >> (mv_to_mvd_shift + mvd_down_shift);
  int mvd_y = (mv.y - mvp.y) >> (mv_to_mvd_shift + mvd_down_shift);
  return GetNumExpGolombBits(mvd_x) + GetNumExpGolombBits(mvd_y);
}

Bits InterSearch::GetMvdBits(const MotionVector3 &mvp, const MotionVector3 &mv,
                             int mvd_down_shift) {
  return GetMvdBits(mvp[0], mv[0], mvd_down_shift) +
    GetMvdBits(mvp[1], mv[1], mvd_down_shift);
}

Bits InterSearch::GetMvdBitsFullpel(const MotionVector &mvp, int fullpel_x,
                                    int fullpel_y, int mvd_down_shift) {
  // Full-pel precision to high precision
  const int mv_up_shift = MotionVector::kPrecisionShift;
  // High precision down to mvd precision
  const int mv_to_mvd_shift =
    MotionVector::kPrecisionShift - MvDelta::kPrecisionShift;
  mvd_down_shift += mv_to_mvd_shift;
  int mvd_x = ((fullpel_x * (1 << mv_up_shift)) - mvp.x) >> mvd_down_shift;
  int mvd_y = ((fullpel_y * (1 << mv_up_shift)) - mvp.y) >> mvd_down_shift;
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

template<>
std::array<MotionVector, constants::kNumInterMvPredictors>
InterSearch::GetMvpListHelper<false, MotionVector>(
  const CodingUnit &cu, RefPicList ref_list, int ref_idx, int max_num_mvp) {
  return GetMvpList(cu, ref_list, ref_idx);
}

template<>
std::array<MotionVector3, constants::kNumInterMvPredictors>
InterSearch::GetMvpListHelper<true, MotionVector3>(
  const CodingUnit &cu, RefPicList ref_list, int ref_idx, int max_num_mvp) {
  return GetMvpListAffine(cu, ref_list, ref_idx, max_num_mvp);
}

template<>
const MotionVector&
InterSearch::GetBestUniPredMv<false, MotionVector>(RefPicList ref_list,
                                                   int ref_idx) const {
  return unipred_best_mv_[static_cast<int>(ref_list)][ref_idx];
}

template<>
const MotionVector3&
InterSearch::GetBestUniPredMv<true, MotionVector3>(RefPicList ref_list,
                                                   int ref_idx) const {
  return affine_best_mv_[static_cast<int>(ref_list)][ref_idx];
}

template<>
void InterSearch::SetBestUniPredMv<false, MotionVector>(
  RefPicList ref_list, int ref_idx, const MotionVector &mv) {
  unipred_best_mv_[static_cast<int>(ref_list)][ref_idx] = mv;
}

template<>
void InterSearch::SetBestUniPredMv<true, MotionVector3>(
  RefPicList ref_list, int ref_idx, const MotionVector3 &mv) {
  affine_best_mv_[static_cast<int>(ref_list)][ref_idx] = mv;
}

}   // namespace xvc
