/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/picture_data.h"

#include <cassert>
#include <fstream>


#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/common.h"
#include "xvc_common_lib/resample.h"


namespace xvc {

PictureData::PictureData(ChromaFormat chroma_format, int width, int height,
                         int bitdepth)
  : rec_pic_(std::make_shared<YuvPicture>(chroma_format, width, height,
                                          bitdepth, true)),
  pic_width_(width),
  pic_height_(height),
  bitdepth_(bitdepth),
  chroma_fmt_(chroma_format),
  chroma_shift_x_(rec_pic_->GetSizeShiftX(YuvComponent::kU)),
  chroma_shift_y_(rec_pic_->GetSizeShiftY(YuvComponent::kU)),
  ctu_num_x_((pic_width_ + constants::kCtuSize - 1) /
             constants::kCtuSize),
  ctu_num_y_((pic_height_ + constants::kCtuSize - 1) /
             constants::kCtuSize) {
  int depth = 0;
  for (int y = 0; y < ctu_num_y_; y++) {
    for (int x = 0; x < ctu_num_x_; x++) {
      ctu_list_.push_back(new CodingUnit(*this, depth,
                                         x * constants::kCtuSize,
                                         y * constants::kCtuSize,
                                         constants::kCtuSize,
                                         constants::kCtuSize));
    }
  }
  int num_cu_pic_x = (pic_width_ + constants::kMaxBlockSize - 1) /
    constants::kMinBlockSize;
  int num_cu_pic_y = (pic_height_ + constants::kMaxBlockSize - 1) /
    constants::kMinBlockSize;
  cu_pic_stride_ = num_cu_pic_x + 1;
  cu_pic_table_.resize(cu_pic_stride_ * (num_cu_pic_y + 1));
  std::fill(cu_pic_table_.begin(), cu_pic_table_.end(), nullptr);
}

PictureData::~PictureData() {
  for (CodingUnit *ctu : ctu_list_) {
    ReleaseCu(ctu);
  }
}

void PictureData::Init(const QP &pic_qp) {
  pic_qp_.reset(new QP(pic_qp));
  std::fill(cu_pic_table_.begin(), cu_pic_table_.end(), nullptr);
  for (CodingUnit *ctu : ctu_list_) {
    ReleaseSubCuRecursively(ctu);
  }
  RefPicList tmvp_ref_list = DetermineTmvpRefList(&tmvp_ref_idx_);
  tmvp_ref_list_inv_ = ReferencePictureLists::Inverse(tmvp_ref_list);
  PicturePredictionType pic_type =
    ref_pic_lists_.GetRefPicType(tmvp_ref_list_inv_, tmvp_ref_idx_);
  tmvp_valid_ = pic_type == PicturePredictionType::kUni ||
    pic_type == PicturePredictionType::kBi;
}

int PictureData::DerivePictureQp(int segment_qp) const {
  if (GetPredictionType() == PicturePredictionType::kIntra) {
    return segment_qp;
  }
  return segment_qp + tid_ + 1;
}

std::shared_ptr<YuvPicture>
PictureData::GetAlternativeRecPic(ChromaFormat chroma_format, int width,
                                  int height, int bitdepth) const {
  if (alt_rec_pic_)
    return alt_rec_pic_;
  auto alt_rec_pic =
    std::make_shared<YuvPicture>(chroma_format, width, height, bitdepth, true);
  for (int c = 0; c < util::GetNumComponents(chroma_format); c++) {
    YuvComponent comp = YuvComponent(c);
    uint8_t* dst =
      reinterpret_cast<uint8_t*>(alt_rec_pic->GetSamplePtr(comp, 0, 0));
    if (rec_pic_->GetChromaFormat() == ChromaFormat::kMonochrome &&
        comp != YuvComponent::kY) {
      std::memset(dst, 1 << (alt_rec_pic->GetBitdepth() - 1),
                  alt_rec_pic->GetStride(comp) *
                  alt_rec_pic->GetHeight(comp) * sizeof(Sample));
      continue;
    }
    uint8_t* src =
      reinterpret_cast<uint8_t*>(rec_pic_->GetSamplePtr(comp, 0, 0));
    resample::Resample<Sample, Sample>
      (dst, alt_rec_pic->GetWidth(comp), alt_rec_pic->GetHeight(comp),
       alt_rec_pic->GetStride(comp), alt_rec_pic->GetBitdepth(),
       src, rec_pic_->GetWidth(comp), rec_pic_->GetHeight(comp),
       rec_pic_->GetStride(comp), rec_pic_->GetBitdepth());
  }
  alt_rec_pic->PadBorder();
  // TODO(PH) Revise const_cast by making this function a pure getter?
  // In the future alternate pictures should probably be created
  // beforehand in a separate thread as this can be quite time consuming
  const_cast<PictureData*>(this)->alt_rec_pic_ = alt_rec_pic;
  return alt_rec_pic;
}

CodingUnit* PictureData::SetCtu(int rsaddr, CodingUnit *cu) {
  if (ctu_list_[rsaddr] == cu) {
    return nullptr;
  }
  CodingUnit *old = ctu_list_[rsaddr];
  ctu_list_[rsaddr] = cu;
  return old;
}

CodingUnit *PictureData::CreateCu(int depth, int posx, int posy, int width,
                                  int height) const {
  if (posx >= pic_width_ || posy >= pic_height_) {
    return nullptr;
  }
  return new CodingUnit(*this, depth, posx, posy, width, height);
}

void PictureData::ReleaseCu(CodingUnit *cu) const {
  ReleaseSubCuRecursively(cu);
  delete cu;
}

void PictureData::MarkUsedInPic(CodingUnit *cu) {
  int index_x = cu->GetPosX(YuvComponent::kY) / constants::kMinBlockSize;
  int index_y = cu->GetPosY(YuvComponent::kY) / constants::kMinBlockSize;
  int num_x = cu->GetWidth(YuvComponent::kY) / constants::kMinBlockSize;
  int num_y = cu->GetHeight(YuvComponent::kY) / constants::kMinBlockSize;
  for (int y = 0; y < num_y; y++) {
    CodingUnit **ptr =
      &cu_pic_table_[(index_y + y) * cu_pic_stride_ + index_x];
    std::fill(ptr, ptr + num_x, cu);
  }
}

void PictureData::ClearMarkCuInPic(CodingUnit *cu) {
  int index_x = cu->GetPosX(YuvComponent::kY) / constants::kMinBlockSize;
  int index_y = cu->GetPosY(YuvComponent::kY) / constants::kMinBlockSize;
  int num_x = cu->GetWidth(YuvComponent::kY) / constants::kMinBlockSize;
  int num_y = cu->GetHeight(YuvComponent::kY) / constants::kMinBlockSize;
  for (int y = 0; y < num_y; y++) {
    CodingUnit **ptr =
      &cu_pic_table_[(index_y + y) * cu_pic_stride_ + index_x];
    std::fill(ptr, ptr + num_x, nullptr);
  }
}

PicturePredictionType PictureData::GetPredictionType() const {
  switch (nal_type_) {
    case NalUnitType::kIntraAccessPicture:
    case NalUnitType::kIntraPicture:
      return PicturePredictionType::kIntra;

    case NalUnitType::kPredictedAccessPicture:
    case NalUnitType::kPredictedPicture:
      return PicturePredictionType::kUni;

    case NalUnitType::kBipredictedAccessPicture:
    case NalUnitType::kBipredictedPicture:
      return PicturePredictionType::kBi;

    default:
      assert(0);
      return PicturePredictionType::kInvalid;
  }
}

RefPicList PictureData::DetermineTmvpRefList(int *tmvp_ref_idx) {
  const int ref_idx = 0;
  *tmvp_ref_idx = ref_idx;
  if (GetPredictionType() != PicturePredictionType::kBi) {
    return RefPicList::kL0;
  }
#if HM_STRICT
  int tid_l0 = ref_pic_lists_.GetRefPicTid(RefPicList::kL0, ref_idx);
  int tid_l1 = ref_pic_lists_.GetRefPicTid(RefPicList::kL1, ref_idx);
  return (tid_l0 <= tid_l1) ? RefPicList::kL0 : RefPicList::kL1;
#else
  if (ref_pic_lists_.GetRefPicType(RefPicList::kL0, 0) ==
      PicturePredictionType::kIntra) {
    return RefPicList::kL1;
  }
  if (ref_pic_lists_.GetRefPicType(RefPicList::kL1, 0) ==
      PicturePredictionType::kIntra) {
    return RefPicList::kL0;
  }
  const QP *qp_l0 = ref_pic_lists_.GetRefPicQp(RefPicList::kL0, ref_idx);
  const QP *qp_l1 = ref_pic_lists_.GetRefPicQp(RefPicList::kL1, ref_idx);
  return (*qp_l0 <= *qp_l1) ? RefPicList::kL0 : RefPicList::kL1;
#endif
}

void PictureData::ReleaseSubCuRecursively(CodingUnit *cu) const {
  for (CodingUnit *sub_cu : cu->GetSubCu()) {
    if (sub_cu) {
      ReleaseSubCuRecursively(sub_cu);
      delete sub_cu;
    }
  }
  cu->GetSubCu().fill(nullptr);
  cu->SetSplit(false);
}

}   // namespace xvc
