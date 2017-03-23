/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/reference_picture_lists.h"

#include <cassert>

#include "xvc_common_lib/picture_data.h"

namespace xvc {

bool ReferencePictureLists::IsRefPicListUsed(RefPicList ref_pic_list,
                                             InterDir inter_dir) {
  switch (inter_dir) {
    case InterDir::kL0:
      return ref_pic_list == RefPicList::kL0;
      break;
    case InterDir::kL1:
      return ref_pic_list == RefPicList::kL1;
      break;
    case InterDir::kBi:
      return true;
      break;
    default:
      return false;
  }
}

PicturePredictionType
ReferencePictureLists::GetRefPicType(RefPicList ref_list, int ref_idx) const {
  const std::vector<RefEntry> *entry_list =
    ref_list == RefPicList::kL0 ? &l0_ : &l1_;
  if (static_cast<int>(entry_list->size()) <= ref_idx) {
    return PicturePredictionType::kInvalid;
  }
  return (*entry_list)[ref_idx].data->GetPredictionType();
}

const QP*
ReferencePictureLists::GetRefPicQp(RefPicList ref_list, int ref_idx) const {
  const std::vector<RefEntry> *entry_list =
    ref_list == RefPicList::kL0 ? &l0_ : &l1_;
  if (static_cast<int>(entry_list->size()) <= ref_idx) {
    return nullptr;
  }
  return (*entry_list)[ref_idx].data->GetPicQp();
}

int
ReferencePictureLists::GetRefPicTid(RefPicList ref_list, int ref_idx) const {
  const std::vector<RefEntry> *entry_list =
    ref_list == RefPicList::kL0 ? &l0_ : &l1_;
  if (static_cast<int>(entry_list->size()) <= ref_idx) {
    return -1;
  }
  return (*entry_list)[ref_idx].data->GetTid();
}

const CodingUnit*
ReferencePictureLists::GetCodingUnitAt(RefPicList ref_list, int index,
                                       CuTree cu_tree, int posx,
                                       int posy) const {
  std::shared_ptr<const PictureData> pic_data;
  if (ref_list == RefPicList::kL0) {
    pic_data = l0_[index].data;
  } else {
    pic_data = l1_[index].data;
  }
  return pic_data->GetCuAt(cu_tree, posx, posy);
}

void
ReferencePictureLists::SetRefPic(RefPicList list, int index, PicNum ref_poc,
                                 const std::shared_ptr<const PictureData>
                                 &pic_data,
                                 const std::shared_ptr<const YuvPicture>
                                 &ref_pic) {
  std::vector<RefEntry> *entry_list = list == RefPicList::kL0 ? &l0_ : &l1_;
  if (index >= static_cast<int>(entry_list->size())) {
    entry_list->resize(index + 1);
  }
  (*entry_list)[index].pic = ref_pic;
  (*entry_list)[index].data = pic_data;
  (*entry_list)[index].poc = ref_poc;
  if (ref_poc > current_poc_) {
    only_back_references_ = false;
  }
}

void
ReferencePictureLists::GetSamePocMappingFor(RefPicList ref_list,
                                            std::vector<int> *mapping) const {
  RefPicList other_list = ReferencePictureLists::Inverse(ref_list);
  int num_ref_pics = GetNumRefPics(ref_list);
  int other_ref_pics = GetNumRefPics(other_list);
  mapping->resize(num_ref_pics);
  for (int i = 0; i < num_ref_pics; i++) {
    (*mapping)[i] = -1;
    PicNum ref_poc = GetRefPoc(ref_list, i);
    for (int j = 0; j < other_ref_pics; j++) {
      if (ref_poc == GetRefPoc(other_list, j)) {
        (*mapping)[i] = j;
        break;
      }
    }
  }
}

void ReferencePictureLists::ZeroOutReferences() {
  for (auto &ref : l0_) {
    ref.pic.reset();
    ref.data.reset();
  }
  for (auto &ref : l1_) {
    ref.pic.reset();
    ref.data.reset();
  }
}

void ReferencePictureLists::Reset(PicNum current_poc) {
  l0_.clear();
  l1_.clear();
  current_poc_ = current_poc;
  only_back_references_ = true;
}

}   // namespace xvc
