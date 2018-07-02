/******************************************************************************
* Copyright (C) 2018, Divideon.
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
* This library is also available under a commercial license.
* Please visit https://xvc.io/license/ for more information.
******************************************************************************/

#include "xvc_common_lib/picture_data.h"

#include <algorithm>
#include <cassert>
#include <cmath>

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
             constants::kCtuSize),
  cu_alloc_batch_size_(std::max(1, ctu_num_x_ * ctu_num_y_) * 4) {
  int num_cu_pic_x = (pic_width_ + constants::kMaxBlockSize - 1) /
    constants::kMinBlockSize;
  int num_cu_pic_y = (pic_height_ + constants::kMaxBlockSize - 1) /
    constants::kMinBlockSize;
  cu_pic_stride_ = num_cu_pic_x + 1;
  // Initial CU buffer allocation, includes majority of allocated CUs
  cu_alloc_buffers_.emplace_back(cu_alloc_batch_size_ * 4);
  for (int tree_idx = 0; tree_idx < constants::kMaxNumCuTrees; tree_idx++) {
    cu_pic_table_[tree_idx].resize(cu_pic_stride_ * (num_cu_pic_y + 1));
    std::fill(cu_pic_table_[tree_idx].begin(),
              cu_pic_table_[tree_idx].end(), nullptr);
  }
}

PictureData::~PictureData() {
}

void PictureData::Init(const SegmentHeader &segment, const Qp &pic_qp,
                       bool recalculate_lambda) {
  // Determine number of CU trees
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

  // CU structure
  max_binary_split_depth_ = segment.max_binary_split_depth;

  // Setup Qp
  pic_qp_.reset(new Qp(pic_qp));
  qps_.clear();
  for (int i = 0; i <= constants::kMaxAllowedQp; i++) {
    int qp_tmp = i;
    double lambda_tmp;
    if (recalculate_lambda) {
      lambda_tmp = 0.57 * pow(2.0, (i - 12) / 3.0);
    } else {
      lambda_tmp = pic_qp.GetLambda() *
        pow(2.0, (i - pic_qp.GetQpRaw(YuvComponent::kY)) / 3.0);
    }
    qps_.emplace_back(qp_tmp, GetChromaFormat(), GetBitdepth(), lambda_tmp,
                      segment.chroma_qp_offset_table,
                      segment.chroma_qp_offset_u, segment.chroma_qp_offset_v);
  }

  // Initialize CU allocator / object pool
  // Buffer pointers are reset to first entry without any deconstruction
  // this requires that no object are reused across pictures
  cu_alloc_free_list_.clear();
  cu_alloc_list_index_ = 0;
  cu_alloc_item_index_ = 0;

  // CTU initialization
  for (int tree_idx = 0; tree_idx < constants::kMaxNumCuTrees; tree_idx++) {
    std::fill(cu_pic_table_[tree_idx].begin(),
              cu_pic_table_[tree_idx].end(), nullptr);
    // Clear all CTU objects and re-assign again below for every picture
    ctu_rs_list_[tree_idx].clear();
  }
  AllocateAllCtu(CuTree::Primary);
  if (num_cu_trees_ > 1) {
    AllocateAllCtu(CuTree::Secondary);
  }

  force_bipred_l1_mvd_zero_ = DetermineForceBipredL1MvdZero();

  // Temporal mv prediction
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
                                  int posy, int width, int height) {
  if (posx >= pic_width_ || posy >= pic_height_) {
    return nullptr;
  }
  CodingUnit *cu;
  if (!cu_alloc_free_list_.empty()) {
    cu = cu_alloc_free_list_.back();
    cu_alloc_free_list_.pop_back();
  } else {
    assert(!cu_alloc_buffers_.empty());
    if (cu_alloc_item_index_ ==
        cu_alloc_buffers_[cu_alloc_list_index_].size()) {
      cu_alloc_list_index_++;
      cu_alloc_item_index_ = 0;
    }
    if (cu_alloc_list_index_ == cu_alloc_buffers_.size()) {
      // Allocate an extra buffer (typically needed for intra pictures)
      cu_alloc_buffers_.emplace_back(cu_alloc_batch_size_);
    }
    cu = &cu_alloc_buffers_[cu_alloc_list_index_][cu_alloc_item_index_];
    cu_alloc_item_index_++;
  }
  // Reinitialize memory to a known state
  return new (cu) CodingUnit(this, ctu_coeff_.get(), cu_tree, depth,
                             posx, posy, width, height);
}

void PictureData::ReleaseCu(CodingUnit *cu) {
  for (CodingUnit *sub_cu : cu->GetSubCu()) {
    if (sub_cu) {
      ReleaseCu(sub_cu);
    }
  }
  cu_alloc_free_list_.push_back(cu);
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

bool PictureData::DetermineForceBipredL1MvdZero() {
  if (IsIntraPic() ||
      Restrictions::Get().disable_ext2_inter_bipred_l1_mvd_zero) {
    return false;
  }
  return ref_pic_lists_.HasOnlyBackReferences();
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
      auto *cu = CreateCu(cu_tree, depth,
                          x * constants::kCtuSize, y * constants::kCtuSize,
                          constants::kCtuSize, constants::kCtuSize);
      ctu_rs_list_[tree_idx].push_back(cu);
    }
  }
}

}   // namespace xvc
