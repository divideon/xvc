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
  : cu_tree_(cu_tree),
  pos_x_(pic_x),
  pos_y_(pic_y),
  width_(width),
  height_(height),
  chroma_shift_x_(pic_data.GetChromaShiftX()),
  chroma_shift_y_(pic_data.GetChromaShiftY()),
  cbf_({ { false, false, false } }),
  sub_cu_list_({ { nullptr, nullptr, nullptr, nullptr } }),
  pic_data_(pic_data),
  qp_(nullptr),
  split_(false),
  depth_(depth),
  pred_mode_(PredictionMode::kIntra),
  root_cbf_(false),
  intra_mode_luma_(IntraMode::kInvalid),
  intra_mode_chroma_(IntraChromaMode::kInvalid),
  inter_() {
  ptrdiff_t coeff_stride = GetCoeffStride();
  for (int c = 0; c < constants::kMaxYuvComponents; c++) {
    int comp_height = GetHeight(YuvComponent(c));
    coeff_[c].resize(coeff_stride * comp_height);
  }
}

void CodingUnit::SetPosition(int posx, int posy) {
  pos_x_ = posx;
  pos_y_ = posy;
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
  const CodingUnit *above_right = GetCodingUnitAboveRight();
  if (!above_right) {
    return 0;
  }
  int pic_remain_x = pic_data_.GetPictureWidth(comp) -
    (GetPosX(comp) + GetWidth(comp));
  return std::min(GetWidth(comp), pic_remain_x);
}

int CodingUnit::GetCuSizeBelowLeft(YuvComponent comp) const {
  const CodingUnit *below_left = GetCodingUnitLeftBelow();
  if (!below_left) {
    return 0;
  }
  int pic_remain_y = pic_data_.GetPictureHeight(comp) -
    (GetPosY(comp) + GetHeight(comp));
  return std::min(GetHeight(comp), pic_remain_y);
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

void CodingUnit::SplitQuad() {
  assert(!split_);
  assert(!sub_cu_list_[0]);
  int sub_width = GetWidth(YuvComponent::kY) >> 1;
  int sub_height = GetHeight(YuvComponent::kY) >> 1;
  int sub_depth = GetDepth() + 1;
  sub_cu_list_[0] =
    pic_data_.CreateCu(cu_tree_, sub_depth, pos_x_ + sub_width * 0,
                       pos_y_ + sub_height * 0, sub_width, sub_height);
  sub_cu_list_[1] =
    pic_data_.CreateCu(cu_tree_, sub_depth, pos_x_ + sub_width * 1,
                       pos_y_ + sub_height * 0, sub_width, sub_height);
  sub_cu_list_[2] =
    pic_data_.CreateCu(cu_tree_, sub_depth, pos_x_ + sub_width * 0,
                       pos_y_ + sub_height * 1, sub_width, sub_height);
  sub_cu_list_[3] =
    pic_data_.CreateCu(cu_tree_, sub_depth, pos_x_ + sub_width * 1,
                       pos_y_ + sub_height * 1, sub_width, sub_height);
  split_ = true;
}

void CodingUnit::UnSplit() {
  assert(split_);
  assert(sub_cu_list_[0]);
  for (int i = 0; i < static_cast<int>(sub_cu_list_.size()); i++) {
    pic_data_.ReleaseCu(sub_cu_list_[i]);
  }
  sub_cu_list_.fill(nullptr);
  split_ = false;
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
