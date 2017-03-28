/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/picture_data.h"

#include <cassert>

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/common.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

PictureData::PictureData(ChromaFormat chroma_format, int width, int height,
                         int bitdepth)
  : pic_width_(width),
  pic_height_(height),
  bitdepth_(bitdepth),
  chroma_fmt_(chroma_format),
  chroma_shift_x_(util::GetChromaShiftX(chroma_format)),
  chroma_shift_y_(util::GetChromaShiftY(chroma_format)),
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
  tmvp_ref_list_ = DetermineTmvpRefList(&tmvp_ref_idx_);
  PicturePredictionType pic_type =
    ref_pic_lists_.GetRefPicType(tmvp_ref_list_, tmvp_ref_idx_);
  tmvp_valid_ = pic_type == PicturePredictionType::kUni ||
    pic_type == PicturePredictionType::kBi;
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
  if (GetPredictionType() != PicturePredictionType::kBi ||
      Restrictions::Get().disable_inter_tmvp_ref_list_derivation) {
    return RefPicList::kL0;
  }
  int tid_l0 = ref_pic_lists_.GetRefPicTid(RefPicList::kL0, ref_idx);
  int tid_l1 = ref_pic_lists_.GetRefPicTid(RefPicList::kL1, ref_idx);
  if (!Restrictions::Get().disable_ext_tmvp_exclude_intra_from_ref_list) {
    if (ref_pic_lists_.GetRefPicType(RefPicList::kL0, ref_idx) ==
        PicturePredictionType::kIntra) {
      return RefPicList::kL1;
    }
    if (ref_pic_lists_.GetRefPicType(RefPicList::kL1, ref_idx) ==
        PicturePredictionType::kIntra) {
      return RefPicList::kL0;
    }
  }
  return (tid_l1 >= tid_l0) ? RefPicList::kL1 : RefPicList::kL0;
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
