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
  : ctu_coeff_(new CoeffCtuBuffer(util::GetChromaShiftX(chroma_format),
                                  util::GetChromaShiftY(chroma_format))),
  pic_width_(width),
  pic_height_(height),
  bitdepth_(bitdepth),
  chroma_fmt_(chroma_format),
  max_num_components_(util::GetNumComponents(chroma_format)),
  chroma_shift_x_(util::GetChromaShiftX(chroma_format)),
  chroma_shift_y_(util::GetChromaShiftY(chroma_format)),
  num_cu_trees_(1),
  ctu_num_x_((pic_width_ + constants::kCtuSize - 1) /
             constants::kCtuSize),
  ctu_num_y_((pic_height_ + constants::kCtuSize - 1) /
             constants::kCtuSize) {
  int num_cu_pic_x = (pic_width_ + constants::kMaxBlockSize - 1) /
    constants::kMinBlockSize;
  int num_cu_pic_y = (pic_height_ + constants::kMaxBlockSize - 1) /
    constants::kMinBlockSize;
  cu_pic_stride_ = num_cu_pic_x + 1;
  for (int tree_idx = 0; tree_idx < constants::kMaxNumCuTrees; tree_idx++) {
    cu_pic_table_[tree_idx].resize(cu_pic_stride_ * (num_cu_pic_y + 1));
    std::fill(cu_pic_table_[tree_idx].begin(),
              cu_pic_table_[tree_idx].end(), nullptr);
  }
}

PictureData::~PictureData() {
  for (int tree_idx = 0; tree_idx < constants::kMaxNumCuTrees; tree_idx++) {
    for (CodingUnit *ctu : ctu_rs_list_[tree_idx]) {
      ReleaseCu(ctu);
    }
  }
}

void PictureData::Init(const QP &pic_qp) {
  if (!Restrictions::Get().disable_ext_two_cu_trees && IsIntraPic() &&
      max_num_components_ > 1) {
    num_cu_trees_ = 2;
    cu_tree_components_[0] = { YuvComponent::kY };
    cu_tree_components_[1] = { YuvComponent::kU, YuvComponent::kV };
  } else if (max_num_components_ > 1) {
    num_cu_trees_ = 1;
    cu_tree_components_[0] = { YuvComponent::kY, YuvComponent::kU,
      YuvComponent::kV };
    cu_tree_components_[1] = {};
  } else {
    num_cu_trees_ = 1;
    cu_tree_components_[0] = { YuvComponent::kY };
    cu_tree_components_[1] = {};
  }
  pic_qp_.reset(new QP(pic_qp));
  for (int tree_idx = 0; tree_idx < constants::kMaxNumCuTrees; tree_idx++) {
    std::fill(cu_pic_table_[tree_idx].begin(),
              cu_pic_table_[tree_idx].end(), nullptr);
    for (CodingUnit *ctu : ctu_rs_list_[tree_idx]) {
      ReleaseSubCuRecursively(ctu);
    }
  }
  if (ctu_rs_list_[static_cast<int>(CuTree::Primary)].empty()) {
    AllocateAllCtu(CuTree::Primary);
  }
  if (num_cu_trees_ > 1 &&
      ctu_rs_list_[static_cast<int>(CuTree::Secondary)].empty()) {
    AllocateAllCtu(CuTree::Secondary);
  }
  tmvp_ref_list_ = DetermineTmvpRefList(&tmvp_ref_idx_);
  PicturePredictionType pic_type =
    ref_pic_lists_.GetRefPicType(tmvp_ref_list_, tmvp_ref_idx_);
  tmvp_valid_ = pic_type == PicturePredictionType::kUni ||
    pic_type == PicturePredictionType::kBi;
}

CodingUnit* PictureData::SetCtu(CuTree cu_tree, int rsaddr, CodingUnit *cu) {
  if (ctu_rs_list_[static_cast<int>(cu_tree)][rsaddr] == cu) {
    return nullptr;
  }
  CodingUnit *old = ctu_rs_list_[static_cast<int>(cu_tree)][rsaddr];
  ctu_rs_list_[static_cast<int>(cu_tree)][rsaddr] = cu;
  return old;
}

const CodingUnit* PictureData::GetLumaCu(const CodingUnit *cu) const {
  if (cu->GetCuTree() == CuTree::Primary) {
    return cu;
  }
  return GetCuAt(CuTree::Primary, cu->GetPosX(YuvComponent::kY),
                 cu->GetPosY(YuvComponent::kY));
}

CodingUnit *PictureData::CreateCu(CuTree cu_tree, int depth, int posx,
                                  int posy, int width, int height) const {
  if (posx >= pic_width_ || posy >= pic_height_) {
    return nullptr;
  }
  return new CodingUnit(*this, ctu_coeff_.get(), cu_tree, depth, posx, posy,
                        width, height);
}

void PictureData::ReleaseCu(CodingUnit *cu) const {
  ReleaseSubCuRecursively(cu);
  delete cu;
}

void PictureData::MarkUsedInPic(CodingUnit *cu) {
  if (cu->GetSplit() != SplitType::kNone) {
    for (CodingUnit *sub_cu : cu->GetSubCu()) {
      if (sub_cu) {
        MarkUsedInPic(sub_cu);
      }
    }
    return;
  }
  const int cu_tree = static_cast<int>(cu->GetCuTree());
  const int index_x = cu->GetPosX(YuvComponent::kY) / constants::kMinBlockSize;
  const int index_y = cu->GetPosY(YuvComponent::kY) / constants::kMinBlockSize;
  const int num_x = cu->GetWidth(YuvComponent::kY) / constants::kMinBlockSize;
  const int num_y = cu->GetHeight(YuvComponent::kY) / constants::kMinBlockSize;
  for (int y = 0; y < num_y; y++) {
    CodingUnit **ptr =
      &cu_pic_table_[cu_tree][(index_y + y) * cu_pic_stride_ + index_x];
    std::fill(ptr, ptr + num_x, cu);
  }
}

void PictureData::ClearMarkCuInPic(CodingUnit *cu) {
  const int cu_tree = static_cast<int>(cu->GetCuTree());
  const int index_x = cu->GetPosX(YuvComponent::kY) / constants::kMinBlockSize;
  const int index_y = cu->GetPosY(YuvComponent::kY) / constants::kMinBlockSize;
  const int num_x = cu->GetWidth(YuvComponent::kY) / constants::kMinBlockSize;
  const int num_y = cu->GetHeight(YuvComponent::kY) / constants::kMinBlockSize;
  for (int y = 0; y < num_y; y++) {
    CodingUnit **ptr =
      &cu_pic_table_[cu_tree][(index_y + y) * cu_pic_stride_ + index_x];
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

void PictureData::AllocateAllCtu(CuTree cu_tree) {
  const int depth = 0;
  int tree_idx = static_cast<int>(cu_tree);
  assert(ctu_rs_list_[tree_idx].empty());
  for (int y = 0; y < ctu_num_y_; y++) {
    for (int x = 0; x < ctu_num_x_; x++) {
      ctu_rs_list_[tree_idx].push_back(
        new CodingUnit(*this, ctu_coeff_.get(), cu_tree, depth,
                       x * constants::kCtuSize,
                       y * constants::kCtuSize,
                       constants::kCtuSize,
                       constants::kCtuSize));
    }
  }
}

void PictureData::ReleaseSubCuRecursively(CodingUnit *cu) const {
  for (CodingUnit *sub_cu : cu->GetSubCu()) {
    if (sub_cu) {
      ReleaseSubCuRecursively(sub_cu);
      delete sub_cu;
    }
  }
  cu->GetSubCu().fill(nullptr);
  cu->SetSplit(SplitType::kNone);
}

}   // namespace xvc
