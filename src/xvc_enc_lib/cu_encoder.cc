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
  : TransformEncoder(rec_pic->GetBitdepth(), pic_data->GetMaxNumComponents(),
                     orig_pic),
  pic_qp_(qp),
  orig_pic_(orig_pic),
  speed_settings_(speed_settings),
  rec_pic_(*rec_pic),
  pic_data_(*pic_data),
  inter_search_(rec_pic->GetBitdepth(), pic_data->GetMaxNumComponents(),
                orig_pic, *pic_data->GetRefPicLists(), speed_settings),
  intra_search_(rec_pic->GetBitdepth(), *pic_data, orig_pic, speed_settings),
  cu_writer_(pic_data_, &intra_search_) {
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
  cu->Split(SplitType::kQuad);
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
  cu->SetSplit(SplitType::kNone);

  if (pic_data_.IsIntraPic()) {
    best_cost = CompressIntra(cu, qp, *writer);
  } else {
    // Inter pic type
    const int cu_tree = static_cast<int>(cu->GetCuTree());
    CodingUnit *temp_cu = rdo_temp_cu_[cu_tree][cu->GetDepth()];
    temp_cu->SetPosition(cu->GetPosX(YuvComponent::kY),
                         cu->GetPosY(YuvComponent::kY));
    temp_cu->SetQp(qp);
    temp_cu->SetSplit(SplitType::kNone);

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
      IntraMode best_mode =
        intra_search_.SearchIntraLuma(cu, comp, qp, writer, this, &rec_pic_);
      cu->SetIntraModeLuma(best_mode);
    } else if (util::IsFirstChroma(comp)) {
      IntraChromaMode chroma_mode =
        intra_search_.SearchIntraChroma(cu, qp, writer, this, &rec_pic_);
      cu->SetIntraModeChroma(chroma_mode);
    }
    dist += intra_search_.CompressIntra(cu, comp, qp, this, &rec_pic_);
  }
  return GetCuCostWithoutSplit(*cu, qp, writer, dist);
}

CuEncoder::RdoCost
CuEncoder::CompressInter(CodingUnit *cu, const QP &qp,
                         const SyntaxWriter &bitstream_writer) {
  bool uni_pred_only =
    pic_data_.GetPredictionType() == PicturePredictionType::kUni;
  SampleBuffer &pred_buffer = GetPredBuffer();
  inter_search_.SearchMotion(cu, qp, uni_pred_only, bitstream_writer,
                             &pred_buffer);
  Distortion dist = inter_search_.CompressAndEvalCbf(cu, qp, bitstream_writer,
                                                     this, &rec_pic_);
  return GetCuCostWithoutSplit(*cu, qp, bitstream_writer, dist);
}

CuEncoder::RdoCost
CuEncoder::CompressMerge(CodingUnit *cu, const QP &qp,
                         const SyntaxWriter &bitstream_writer) {
  std::array<bool,
    constants::kNumInterMergeCandidates> skip_evaluated = { false };
  InterMergeCandidateList merge_list = inter_search_.GetMergeCandidates(*cu);
  RdoCost best_cost(std::numeric_limits<Cost>::max());
  CodingUnit::TransformState best_transform_state;
  int best_merge_idx = -1;

  int num_merge_cand = Restrictions::Get().disable_inter_merge_candidates ?
    1 : static_cast<int>(merge_list.size());
  for (int skip_eval_idx = 0; skip_eval_idx < 2; skip_eval_idx++) {
    bool force_skip = skip_eval_idx != 0;
    for (int merge_idx = 0; merge_idx < num_merge_cand; merge_idx++) {
      if (skip_evaluated[merge_idx]) {
        continue;
      }
      Distortion dist =
        inter_search_.SearchMergeCbf(cu, qp, bitstream_writer, merge_list,
                                     merge_idx, force_skip, this, &rec_pic_);
      RdoCost cost = GetCuCostWithoutSplit(*cu, qp, bitstream_writer, dist);
      if (!cu->GetHasAnyCbf()) {
        skip_evaluated[merge_idx] = true;
      }
      if (cost.cost < best_cost.cost) {
        best_cost = cost;
        best_merge_idx = merge_idx;
        cu->SaveStateTo(&best_transform_state, rec_pic_);
        if (!cu->GetHasAnyCbf() && !force_skip) {
          // Encoder optimization, assume skip is always best
          break;
        }
      }
    }
  }
  cu->SetMergeIdx(best_merge_idx);
  inter_search_.ApplyMerge(cu, merge_list, best_merge_idx);
  cu->LoadStateFrom(best_transform_state, &rec_pic_);
  cu->SetSkipFlag(!cu->GetRootCbf());
  return best_cost;
}

CuEncoder::RdoCost
CuEncoder::GetCuCostWithoutSplit(const CodingUnit &cu, const QP &qp,
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
