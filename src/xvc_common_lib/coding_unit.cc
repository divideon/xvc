/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/coding_unit.h"

#include <algorithm>
#include <cassert>
#include <cstring>

#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/sample_buffer.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

CodingUnit::CodingUnit(const PictureData &pic_data, CuTree cu_tree, int depth,
                       int pic_x, int pic_y, int width, int height)
  : pic_data_(pic_data),
  chroma_shift_x_(pic_data.GetChromaShiftX()),
  chroma_shift_y_(pic_data.GetChromaShiftY()),
  cu_tree_(cu_tree),
  pos_x_(pic_x),
  pos_y_(pic_y),
  width_(width),
  height_(height),
  depth_(depth),
  cbf_({ { false, false, false } }),
  sub_cu_list_({ { nullptr, nullptr, nullptr, nullptr } }),
  qp_(nullptr),
  split_state_(SplitType::kNone),
  pred_mode_(PredictionMode::kIntra),
  root_cbf_(false),
  intra_mode_luma_(IntraMode::kInvalid),
  intra_mode_chroma_(IntraChromaMode::kInvalid),
  inter_() {
  ptrdiff_t coeff_stride = GetCoeffStride();
  for (int c = 0; c < constants::kMaxYuvComponents; c++) {
    YuvComponent comp = YuvComponent(c);
    int comp_height = util::ScaleSizeY(constants::kMaxBlockSize,
                                       pic_data_.GetChromaFormat(), comp);
    coeff_[static_cast<int>(comp)].resize(comp_height * coeff_stride);
  }
}

int CodingUnit::GetBinaryDepth() const {
  int quad_size_log2 = util::SizeToLog2(constants::kCtuSize >> depth_);
  return (quad_size_log2 - util::SizeToLog2(width_)) +
    (quad_size_log2 - util::SizeToLog2(height_));
}

bool CodingUnit::IsBinarySplitValid() const {
  int max_split_depth = pic_data_.GetMaxBinarySplitDepth(cu_tree_);
  int max_split_size = pic_data_.GetMaxBinarySplitSize(cu_tree_);
  return GetBinaryDepth() < max_split_depth &&
    width_ <= max_split_size &&
    height_ <= max_split_size &&
    (width_ > constants::kMinBinarySplitSize ||
     height_ > constants::kMinBinarySplitSize);
}

const QP& CodingUnit::GetQp() const {
  return *qp_;
}

void CodingUnit::SetQp(const QP &qp) {
  qp_ = &qp;
}

int CodingUnit::GetQp(YuvComponent comp) const {
  return qp_->GetQpRaw(comp);
}

void CodingUnit::InitializeFrom(const CodingUnit &cu) {
  assert(cu_tree_ == cu.cu_tree_);
  pos_x_ = cu.pos_x_;
  pos_y_ = cu.pos_y_;
  width_ = cu.width_;
  height_ = cu.height_;
  depth_ = cu.depth_;
  qp_ = cu.qp_;
}

SplitRestriction
CodingUnit::DeriveSiblingSplitRestriction(SplitType parent_split) const {
  if (pic_data_.GetPredictionType() == PicturePredictionType::kIntra) {
    return SplitRestriction::kNone;
  }
  if (parent_split == SplitType::kVertical &&
      split_state_ == SplitType::kHorizontal) {
    // This case is like quad split although with different coding order
    return width_ > constants::kMinBinarySplitSize && GetBinaryDepth() == 1 ?
      SplitRestriction::kNoHorizontal : SplitRestriction::kNone;
  } else if (parent_split == SplitType::kHorizontal &&
             split_state_ == SplitType::kVertical) {
    // This case is almost identical to quad split
    return SplitRestriction::kNoVertical;
  }
  return SplitRestriction::kNone;
}

const PictureData* CodingUnit::GetPicData() const {
  return &pic_data_;
}

PicturePredictionType CodingUnit::GetPicType() const {
  return pic_data_.GetPredictionType();
}

PicNum CodingUnit::GetPoc() const {
  return pic_data_.GetPoc();
}

const ReferencePictureLists *CodingUnit::GetRefPicLists() const {
  return pic_data_.GetRefPicLists();
}

PicNum CodingUnit::GetRefPoc(RefPicList ref_list) const {
  if (!HasMv(ref_list)) {
    return static_cast<PicNum>(-1);
  }
  int ref_idx = GetRefIdx(ref_list);
  return pic_data_.GetRefPicLists()->GetRefPoc(ref_list, ref_idx);
}

bool CodingUnit::IsFullyWithinPicture() const {
  return pos_x_ + width_ <= pic_data_.GetPictureWidth(YuvComponent::kY) &&
    pos_y_ + height_ <= pic_data_.GetPictureHeight(YuvComponent::kY);
}

const CodingUnit* CodingUnit::GetCodingUnitAbove() const {
  int posx = pos_x_;
  int posy = pos_y_;
  if (posy == 0) {
    return nullptr;
  }
  return pic_data_.GetCuAt(cu_tree_, posx, posy - constants::kMinBlockSize);
}

const CodingUnit* CodingUnit::GetCodingUnitAboveIfSameCtu() const {
  int posx = pos_x_;
  int posy = pos_y_;
  if ((posy % constants::kCtuSize) == 0) {
    return nullptr;
  }
  return pic_data_.GetCuAt(cu_tree_, posx, posy - constants::kMinBlockSize);
}

const CodingUnit* CodingUnit::GetCodingUnitAboveLeft() const {
  int posx = pos_x_;
  int posy = pos_y_;
  if (posx == 0 || posy == 0) {
    return nullptr;
  }
  return pic_data_.GetCuAt(cu_tree_, posx - constants::kMinBlockSize,
                           posy - constants::kMinBlockSize);
}

const CodingUnit* CodingUnit::GetCodingUnitAboveCorner() const {
  int right = pos_x_ + width_;
  int posy = pos_y_;
  if (posy == 0) {
    return nullptr;
  }
  return pic_data_.GetCuAt(cu_tree_, right - constants::kMinBlockSize,
                           posy - constants::kMinBlockSize);
}

const CodingUnit* CodingUnit::GetCodingUnitAboveRight() const {
  int right = pos_x_ + width_;
  int posy = pos_y_;
  if (posy == 0) {
    return nullptr;
  }
  // Padding in table will guard for y going out-of-bounds
  return pic_data_.GetCuAt(cu_tree_, right, posy - constants::kMinBlockSize);
}

const CodingUnit* CodingUnit::GetCodingUnitLeft() const {
  int posx = pos_x_;
  int posy = pos_y_;
  if (posx == 0) {
    return nullptr;
  }
  return pic_data_.GetCuAt(cu_tree_, posx - constants::kMinBlockSize, posy);
}

const CodingUnit* CodingUnit::GetCodingUnitLeftCorner() const {
  int posx = pos_x_;
  int bottom = pos_y_ + height_;
  if (posx == 0) {
    return nullptr;
  }
  return pic_data_.GetCuAt(cu_tree_, posx - constants::kMinBlockSize,
                           bottom - constants::kMinBlockSize);
}

const CodingUnit* CodingUnit::GetCodingUnitLeftBelow() const {
  int posx = pos_x_;
  int bottom = pos_y_ + height_;
  if (posx == 0) {
    return nullptr;
  }
  // Padding in table will guard for y going out-of-bounds
  return pic_data_.GetCuAt(cu_tree_, posx - constants::kMinBlockSize, bottom);
}

int CodingUnit::GetCuSizeAboveRight(YuvComponent comp) const {
  int posx = pos_x_ + width_;
  int posy = pos_y_ - constants::kMinBlockSize;
  if (posy < 0) {
    return 0;
  }
  posx -= constants::kMinBlockSize;
  for (int i = height_; i >= 0; i -= constants::kMinBlockSize) {
    if (pic_data_.GetCuAt(cu_tree_, posx + i, posy)) {
      return util::IsLuma(comp) ? i : (i >> chroma_shift_y_);
    }
  }
  return 0;
}

int CodingUnit::GetCuSizeBelowLeft(YuvComponent comp) const {
  int posx = pos_x_ - constants::kMinBlockSize;
  int posy = pos_y_ + height_;
  if (posx < 0) {
    return 0;
  }
  posy -= constants::kMinBlockSize;
  for (int i = width_; i >= 0; i -= constants::kMinBlockSize) {
    if (pic_data_.GetCuAt(cu_tree_, posx, posy + i)) {
      return util::IsLuma(comp) ? i : (i >> chroma_shift_x_);
    }
  }
  return 0;
}

IntraMode CodingUnit::GetIntraMode(YuvComponent comp) const {
  if (util::IsLuma(comp)) {
    assert(cu_tree_ == CuTree::Primary);
    return intra_mode_luma_;
  }
  if (intra_mode_chroma_ == IntraChromaMode::kDMChroma) {
    if (cu_tree_ == CuTree::Primary) {
      return intra_mode_luma_;
    }
    const CodingUnit *luma_cu = pic_data_.GetLumaCu(this);
    return luma_cu->intra_mode_luma_;
  }
  assert(static_cast<int>(intra_mode_chroma_) < IntraMode::kTotalNumber);
  return static_cast<IntraMode>(intra_mode_chroma_);
}

void CodingUnit::Split(SplitType split_type) {
  assert(split_type != SplitType::kNone);
  assert(split_state_ == SplitType::kNone);
  assert(!sub_cu_list_[0]);
  split_state_ = split_type;
  int sub_width = width_ >> 1;
  int sub_height = height_ >> 1;
  int quad_subdepth = depth_ + 1;
  switch (split_type) {
    case SplitType::kQuad:
      sub_cu_list_[0] =
        pic_data_.CreateCu(cu_tree_, quad_subdepth, pos_x_ + sub_width * 0,
                           pos_y_ + sub_height * 0, sub_width, sub_height);
      sub_cu_list_[1] =
        pic_data_.CreateCu(cu_tree_, quad_subdepth, pos_x_ + sub_width * 1,
                           pos_y_ + sub_height * 0, sub_width, sub_height);
      sub_cu_list_[2] =
        pic_data_.CreateCu(cu_tree_, quad_subdepth, pos_x_ + sub_width * 0,
                           pos_y_ + sub_height * 1, sub_width, sub_height);
      sub_cu_list_[3] =
        pic_data_.CreateCu(cu_tree_, quad_subdepth, pos_x_ + sub_width * 1,
                           pos_y_ + sub_height * 1, sub_width, sub_height);
      break;

    case SplitType::kHorizontal:
      sub_cu_list_[0] =
        pic_data_.CreateCu(cu_tree_, depth_, pos_x_,
                           pos_y_ + sub_height * 0, width_, sub_height);
      sub_cu_list_[1] =
        pic_data_.CreateCu(cu_tree_, depth_, pos_x_,
                           pos_y_ + sub_height * 1, width_, sub_height);
      sub_cu_list_[2] = nullptr;
      sub_cu_list_[3] = nullptr;
      break;

    case SplitType::kVertical:
      sub_cu_list_[0] =
        pic_data_.CreateCu(cu_tree_, depth_, pos_x_ + sub_width * 0,
                           pos_y_, sub_width, height_);
      sub_cu_list_[1] =
        pic_data_.CreateCu(cu_tree_, depth_, pos_x_ + sub_width * 1,
                           pos_y_, sub_width, height_);
      sub_cu_list_[2] = nullptr;
      sub_cu_list_[3] = nullptr;
      break;

    case SplitType::kNone:
    default:
      assert(0);
      break;
  }
}

void CodingUnit::UnSplit() {
  assert(split_state_ != SplitType::kNone);
  assert(sub_cu_list_[0]);
  for (int i = 0; i < static_cast<int>(sub_cu_list_.size()); i++) {
    if (sub_cu_list_[i]) {
      pic_data_.ReleaseCu(sub_cu_list_[i]);
    }
  }
  sub_cu_list_.fill(nullptr);
  split_state_ = SplitType::kNone;
}

void CodingUnit::SaveStateTo(ReconstructionState *dst_state,
                             const YuvPicture &rec_pic) {
  for (int c = 0; c < pic_data_.GetMaxNumComponents(); c++) {
    const YuvComponent comp = YuvComponent(c);
    int posx = GetPosX(comp);
    int posy = GetPosY(comp);
    // Reco
    dst_state->reco[c].resize(GetWidth(comp) * GetHeight(comp));
    auto reco_src = rec_pic.GetSampleBuffer(comp, posx, posy);
    SampleBuffer reco_dst(&dst_state->reco[c][0], GetWidth(comp));
    reco_dst.CopyFrom(GetWidth(comp), GetHeight(comp), reco_src);
  }
}

void CodingUnit::SaveStateTo(TransformState *dst_state,
                             const YuvPicture &rec_pic) {
  dst_state->cbf = cbf_;
  SaveStateTo(static_cast<ReconstructionState*>(dst_state), rec_pic);
  for (int c = 0; c < pic_data_.GetMaxNumComponents(); c++) {
    const YuvComponent comp = YuvComponent(c);
    // Coeff
    if (cbf_[c]) {
      dst_state->coeff[c].resize(GetWidth(comp) * GetHeight(comp));
      DataBuffer<const Coeff> coeff_src(&coeff_[c][0], GetCoeffStride());
      CoeffBuffer coeff_dst(&dst_state->coeff[c][0], GetWidth(comp));
      coeff_dst.CopyFrom(GetWidth(comp), GetHeight(comp), coeff_src);
    }
  }
}

void CodingUnit::SaveStateTo(InterState *state) {
  *state = inter_;
}

void CodingUnit::LoadStateFrom(const ReconstructionState &src_state,
                               YuvPicture *rec_pic) {
  for (int c = 0; c < pic_data_.GetMaxNumComponents(); c++) {
    const YuvComponent comp = YuvComponent(c);
    int posx = GetPosX(comp);
    int posy = GetPosY(comp);
    DataBuffer<const Sample> reco_src(&src_state.reco[c][0], GetWidth(comp));
    SampleBuffer reco_dst = rec_pic->GetSampleBuffer(comp, posx, posy);
    reco_dst.CopyFrom(GetWidth(comp), GetHeight(comp), reco_src);
  }
}

void CodingUnit::LoadStateFrom(const TransformState &src_state,
                               YuvPicture *rec_pic) {
  cbf_ = src_state.cbf;
  LoadStateFrom(static_cast<const ReconstructionState&>(src_state), rec_pic);
  for (int c = 0; c < pic_data_.GetMaxNumComponents(); c++) {
    const YuvComponent comp = YuvComponent(c);
    if (src_state.cbf[c]) {
      coeff_[c].resize(GetCoeffStride() * GetHeight(comp));
      DataBuffer<const Coeff> coeff_src(&src_state.coeff[c][0], GetWidth(comp));
      CoeffBuffer coeff_dst(&coeff_[c][0], GetCoeffStride());
      coeff_dst.CopyFrom(GetWidth(comp), GetHeight(comp), coeff_src);
    }
  }
  SetRootCbf(GetHasAnyCbf());
}

void CodingUnit::LoadStateFrom(const InterState &state) {
  inter_ = state;
}

}   // namespace xvc
