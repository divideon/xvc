/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_CODING_UNIT_H_
#define XVC_COMMON_LIB_CODING_UNIT_H_

#include <array>
#include <cassert>
#include <memory>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/cu_types.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/picture_types.h"
#include "xvc_common_lib/reference_picture_lists.h"
#include "xvc_common_lib/yuv_pic.h"

namespace xvc {

class QP;

class CodingUnit {
public:
  struct ReconstructionState {
    std::array<std::array<Sample, constants::kMaxBlockSamples>,
      constants::kMaxYuvComponents> reco;
    std::array<std::array<Coeff, constants::kMaxBlockSamples>,
      constants::kMaxYuvComponents> coeff;
  };
  struct TransformState {
    ReconstructionState reco;
    std::array<bool, constants::kMaxYuvComponents> cbf;
  };
  struct InterState {
    InterDir inter_dir = InterDir::kL0;
    bool skip_flag = false;
    bool merge_flag = false;
    int merge_idx = -1;
    std::array<MotionVector, 2> mv;
    std::array<MotionVector, 2> mvd;
    std::array<int8_t, 2> ref_idx;
    std::array<int8_t, 2> mvp_idx;
  };

  CodingUnit(const PictureData &pic_data, CoeffCtuBuffer *ctu_coeff,
             CuTree cu_tree, int depth, int pic_x, int pic_y,
             int width, int height);

  // General
  CuTree GetCuTree() const { return cu_tree_; }
  int GetPosX(YuvComponent comp) const {
    return comp == YuvComponent::kY ? pos_x_ : pos_x_ >> chroma_shift_x_;
  }
  int GetPosY(YuvComponent comp) const {
    return comp == YuvComponent::kY ? pos_y_ : pos_y_ >> chroma_shift_y_;
  }
  int GetDepth() const { return depth_; }
  int GetBinaryDepth() const;
  bool IsBinarySplitValid() const;
  int GetWidth(YuvComponent comp) const {
    return comp == YuvComponent::kY ? width_ : width_ >> chroma_shift_x_;
  }
  int GetHeight(YuvComponent comp) const {
    return comp == YuvComponent::kY ? height_ : height_ >> chroma_shift_y_;
  }
  PartitionType GetPartitionType() const { return PartitionType::kSize2Nx2N; }
  void SetPartitionType(PartitionType part_type) {
    assert(part_type == PartitionType::kSize2Nx2N);
  }
  const QP& GetQp() const;
  void SetQp(const QP &qp);
  void SetQp(int qp_value);
  int GetQp(YuvComponent comp) const;
  void InitializeFrom(const CodingUnit &cu);

  // Split logic
  SplitType GetSplit() const { return split_state_; }
  void SetSplit(SplitType split_type) { split_state_ = split_type; }
  std::array<CodingUnit*, constants::kQuadSplit>& GetSubCu() {
    return sub_cu_list_;
  }
  const CodingUnit* GetSubCu(int idx) const {
    return sub_cu_list_[idx];
  }
  SplitRestriction DeriveSiblingSplitRestriction(SplitType parent_split) const;

  // Picture related data
  const PictureData* GetPicData() const;
  PicturePredictionType GetPicType() const;
  PicNum GetPoc() const;
  const ReferencePictureLists *GetRefPicLists() const;
  PicNum GetRefPoc(RefPicList list) const;

  // Neighborhood
  bool IsFullyWithinPicture() const;
  const CodingUnit *GetCodingUnitAbove() const;
  const CodingUnit *GetCodingUnitAboveIfSameCtu() const;
  const CodingUnit *GetCodingUnitAboveLeft() const;
  const CodingUnit *GetCodingUnitAboveCorner() const;
  const CodingUnit *GetCodingUnitAboveRight() const;
  const CodingUnit *GetCodingUnitLeft() const;
  const CodingUnit *GetCodingUnitLeftCorner() const;
  const CodingUnit *GetCodingUnitLeftBelow() const;
  int GetCuSizeAboveRight(YuvComponent comp) const;
  int GetCuSizeBelowLeft(YuvComponent comp) const;

  // Transform
  bool GetRootCbf() const { return root_cbf_; }
  void SetRootCbf(bool root_cbf) { root_cbf_ = root_cbf; }
  bool GetCbf(YuvComponent comp) const { return cbf_[comp]; }
  void SetCbf(YuvComponent comp, bool cbf) { cbf_[comp] = cbf; }
  CoeffBuffer GetCoeff(YuvComponent comp) {
    return ctu_coeff_.GetBuffer(comp, GetPosX(comp), GetPosY(comp));
  }
  DataBuffer<const Coeff> GetCoeff(YuvComponent comp) const {
    return ctu_coeff_.GetBuffer(comp, GetPosX(comp), GetPosY(comp));
  }
  bool GetHasAnyCbf() const {
    return cbf_[YuvComponent::kY] || cbf_[YuvComponent::kU] ||
      cbf_[YuvComponent::kV];
  }

  // Prediction
  PredictionMode GetPredMode() const { return pred_mode_; }
  void SetPredMode(PredictionMode pred_mode) { pred_mode_ = pred_mode; }
  bool IsIntra() const { return GetPredMode() == PredictionMode::kIntra; }
  bool IsInter() const { return GetPredMode() == PredictionMode::kInter; }

  // Intra
  IntraMode GetIntraMode(YuvComponent comp) const;
  IntraChromaMode GetIntraChromaMode() const { return intra_mode_chroma_; }
  void SetIntraModeLuma(IntraMode intra_mode) { intra_mode_luma_ = intra_mode; }
  void SetIntraModeChroma(IntraChromaMode intra_mode) {
    intra_mode_chroma_ = intra_mode;
  }

  // Inter
  InterDir GetInterDir() const { return inter_.inter_dir; }
  void SetInterDir(InterDir inter_dir) { inter_.inter_dir = inter_dir; }
  bool GetSkipFlag() const { return inter_.skip_flag; }
  void SetSkipFlag(bool skip) { inter_.skip_flag = skip; }
  bool GetMergeFlag() const { return inter_.merge_flag; }
  void SetMergeFlag(bool merge) { inter_.merge_flag = merge; }
  int GetMergeIdx() const { return inter_.merge_idx; }
  void SetMergeIdx(int merge_idx) { inter_.merge_idx = merge_idx; }
  bool HasMv(RefPicList ref_list) const {
    return GetInterDir() == InterDir::kBi ||
      (ref_list == RefPicList::kL0 && GetInterDir() == InterDir::kL0) ||
      (ref_list == RefPicList::kL1 && GetInterDir() == InterDir::kL1);
  }
  int GetRefIdx(RefPicList list) const {
    return inter_.ref_idx[static_cast<int>(list)];
  }
  void SetRefIdx(int ref_idx, RefPicList list) {
    inter_.ref_idx[static_cast<int>(list)] = static_cast<uint8_t>(ref_idx);
  }
  const MotionVector& GetMv(RefPicList list) const {
    return inter_.mv[static_cast<int>(list)];
  }
  void SetMv(const MotionVector &mv, RefPicList list) {
    inter_.mv[static_cast<int>(list)] = mv;
  }
  const MotionVector& GetMvDelta(RefPicList list) const {
    return inter_.mvd[static_cast<int>(list)];
  }
  void SetMvDelta(const MotionVector &mvd, RefPicList list) {
    inter_.mvd[static_cast<int>(list)] = mvd;
  }
  int GetMvpIdx(RefPicList list) const {
    return inter_.mvp_idx[static_cast<int>(list)];
  }
  void SetMvpIdx(int mvp_idx, RefPicList list) {
    inter_.mvp_idx[static_cast<int>(list)] = static_cast<uint8_t>(mvp_idx);
  }

  // State handling
  void Split(SplitType split_type);
  void UnSplit();
  void SaveStateTo(ReconstructionState *state, const YuvPicture &rec_pic) const;
  void SaveStateTo(TransformState *state, const YuvPicture &rec_pic) const;
  void SaveStateTo(InterState *state) const;
  void LoadStateFrom(const ReconstructionState &state, YuvPicture *rec_pic);
  void LoadStateFrom(const TransformState &state, YuvPicture *rec_pic);
  void LoadStateFrom(const InterState &state);

private:
  const PictureData &pic_data_;
  const int chroma_shift_x_;
  const int chroma_shift_y_;
  const CuTree cu_tree_;
  int pos_x_;
  int pos_y_;
  int width_;
  int height_;
  int depth_;
  SplitType split_state_;
  PredictionMode pred_mode_;
  std::array<bool, constants::kMaxYuvComponents> cbf_;
  std::array<CodingUnit*, constants::kQuadSplit> sub_cu_list_;
  const QP *qp_;
  CoeffCtuBuffer &ctu_coeff_;   // Coefficient storage for this CU
  bool root_cbf_;
  // Intra
  IntraMode intra_mode_luma_;
  IntraChromaMode intra_mode_chroma_;
  // Inter
  InterState inter_;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_CODING_UNIT_H_
