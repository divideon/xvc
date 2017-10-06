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

class Qp;

class CodingUnit {
public:
  struct ReconstructionState {
    std::array<std::array<Sample, constants::kMaxBlockSamples>,
      constants::kMaxYuvComponents> reco;
    std::array<std::array<Coeff, constants::kMaxBlockSamples>,
      constants::kMaxYuvComponents> coeff;
  };
  struct TransformState {
    TransformState();
    bool root_cbf;
    std::array<bool, constants::kMaxYuvComponents> cbf;
    std::array<bool, constants::kMaxYuvComponents> transform_skip;
    std::array<std::array<TransformType, 2>,
      constants::kMaxNumPlanes> transform_type;
    int transform_select_idx;
  };
  struct IntraState {
    IntraMode mode_luma = IntraMode::kInvalid;
    IntraChromaMode mode_chroma = IntraChromaMode::kInvalid;
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
  struct ResidualState {
    ReconstructionState reco;
    TransformState tx;
  };
  CodingUnit() {}
  CodingUnit(PictureData *pic_data, CoeffCtuBuffer *ctu_coeff,
             CuTree cu_tree, int depth, int pic_x, int pic_y,
             int width, int height);
  CodingUnit(const CodingUnit &) = delete;
  CodingUnit& operator=(const CodingUnit &cu);
  void CopyPositionAndSizeFrom(const CodingUnit &cu);
  void CopyPredictionDataFrom(const CodingUnit &cu);

  // General
  CuTree GetCuTree() const { return cu_tree_; }
  int GetPosX(YuvComponent comp) const {
    return comp == YuvComponent::kY ?
      pos_x_ : pos_x_ >> pic_data_->GetChromaShiftX();
  }
  int GetPosY(YuvComponent comp) const {
    return comp == YuvComponent::kY ?
      pos_y_ : pos_y_ >> pic_data_->GetChromaShiftY();
  }
  int GetDepth() const { return depth_; }
  int GetBinaryDepth() const;
  bool IsBinarySplitValid() const;
  int GetWidth(YuvComponent comp) const {
    return comp == YuvComponent::kY ?
      width_ : width_ >> pic_data_->GetChromaShiftX();
  }
  int GetHeight(YuvComponent comp) const {
    return comp == YuvComponent::kY ?
      height_ : height_ >> pic_data_->GetChromaShiftY();
  }
  PartitionType GetPartitionType() const { return PartitionType::kSize2Nx2N; }
  void SetPartitionType(PartitionType part_type) {
    assert(part_type == PartitionType::kSize2Nx2N);
  }
  const Qp& GetQp() const;
  void SetQp(const Qp &qp);
  void SetQp(int qp_value);
  int GetQp(YuvComponent comp) const;

  // Split logic
  SplitType GetSplit() const { return split_state_; }
  void SetSplit(SplitType split_type) { split_state_ = split_type; }
  std::array<CodingUnit*, constants::kQuadSplit>& GetSubCu() {
    return sub_cu_list_;
  }
  const CodingUnit* GetSubCu(int idx) const {
    return sub_cu_list_.at(idx);
  }
  bool IsFirstCuInQuad(int depth) const {
    const int size = constants::kCtuSize >> depth;
    return (pos_x_ & (size - 1)) == 0 && (pos_y_ & (size - 1)) == 0;
  }
  SplitRestriction DeriveSiblingSplitRestriction(SplitType parent_split) const;

  // Picture related data
  const PictureData* GetPicData() const {
    return pic_data_;
  }
  PicturePredictionType GetPicType() const {
    return pic_data_->GetPredictionType();
  }
  PicNum GetPoc() const {
    return pic_data_->GetPoc();
  }
  const ReferencePictureLists *GetRefPicLists() const {
    return pic_data_->GetRefPicLists();
  }
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
  bool GetRootCbf() const { return tx_.root_cbf; }
  void SetRootCbf(bool root_cbf) { tx_.root_cbf = root_cbf; }
  bool GetCbf(YuvComponent comp) const { return tx_.cbf[comp]; }
  void SetCbf(YuvComponent comp, bool cbf) { tx_.cbf[comp] = cbf; }
  void ClearCbf(YuvComponent comp);   // sets cbf=false & reset tx related vars
  bool GetTransformSkip(YuvComponent comp) const {
    return tx_.transform_skip[comp];
  }
  void SetTransformSkip(YuvComponent comp, bool tx_skip) {
    tx_.transform_skip[comp] = tx_skip;
  }
  TransformType GetTransformType(YuvComponent comp, int idx) const {
    return tx_.transform_type[comp != YuvComponent::kY][idx];
  }
  bool HasTransformSelectIdx() const { return tx_.transform_select_idx >= 0; }
  int GetTransformSelectIdx() const { return tx_.transform_select_idx; }
  void SetTransformFromSelectIdx(YuvComponent comp, int tx_select_idx);
  CoeffBuffer GetCoeff(YuvComponent comp) {
    return ctu_coeff_->GetBuffer(comp, GetPosX(comp), GetPosY(comp));
  }
  DataBuffer<const Coeff> GetCoeff(YuvComponent comp) const {
    return ctu_coeff_->GetBuffer(comp, GetPosX(comp), GetPosY(comp));
  }
  bool GetHasAnyCbf() const {
    return tx_.cbf[YuvComponent::kY] || tx_.cbf[YuvComponent::kU] ||
      tx_.cbf[YuvComponent::kV];
  }
  bool CanTransformSkip(YuvComponent comp) const {
    return GetWidth(comp) * GetHeight(comp) <= constants::kTransformSkipMaxArea;
  }

  // Prediction
  PredictionMode GetPredMode() const { return pred_mode_; }
  void SetPredMode(PredictionMode pred_mode) { pred_mode_ = pred_mode; }
  bool IsIntra() const { return GetPredMode() == PredictionMode::kIntra; }
  bool IsInter() const { return GetPredMode() == PredictionMode::kInter; }

  // Intra
  IntraMode GetIntraMode(YuvComponent comp) const;
  IntraChromaMode GetIntraChromaMode() const { return intra_.mode_chroma; }
  void SetIntraModeLuma(IntraMode intra_mode) { intra_.mode_luma = intra_mode; }
  void SetIntraModeChroma(IntraChromaMode intra_mode) {
    intra_.mode_chroma = intra_mode;
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
  void SaveStateTo(ReconstructionState *state, const YuvPicture &rec_pic,
                   YuvComponent comp) const;
  void SaveStateTo(ReconstructionState *state, const YuvPicture &rec_pic) const;
  void SaveStateTo(ResidualState *state, const YuvPicture &rec_pic,
                   YuvComponent comp) const;
  void SaveStateTo(ResidualState *state, const YuvPicture &rec_pic) const;
  void SaveStateTo(InterState *state) const;
  void LoadStateFrom(const ReconstructionState &state, YuvPicture *rec_pic,
                     YuvComponent comp);
  void LoadStateFrom(const ReconstructionState &state, YuvPicture *rec_pic);
  void LoadStateFrom(const ResidualState &state, YuvPicture *rec_pic,
                     YuvComponent comp);
  void LoadStateFrom(const ResidualState &state, YuvPicture *rec_pic);
  void LoadStateFrom(const InterState &state);

private:
  PictureData *pic_data_ = nullptr;
  CoeffCtuBuffer *ctu_coeff_ = nullptr;   // Coefficient storage for this CU
  CuTree cu_tree_;
  int pos_x_;
  int pos_y_;
  int width_;
  int height_;
  int depth_;
  SplitType split_state_;
  PredictionMode pred_mode_;
  std::array<CodingUnit*, constants::kQuadSplit> sub_cu_list_;
  const Qp *qp_;
  TransformState tx_;
  IntraState intra_;
  InterState inter_;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_CODING_UNIT_H_
