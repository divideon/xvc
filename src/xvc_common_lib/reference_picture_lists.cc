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

void
ReferencePictureLists::SetRefPic(RefPicList list, int index, PicNum ref_poc,
                                 const std::shared_ptr<PictureData> &pic_data,
                                 const std::shared_ptr<YuvPicture> &ref_pic) {
  std::vector<RefEntry> *entry_list = list == RefPicList::kL0 ? &l0_ : &l1_;
  if (index >= static_cast<int>(entry_list->size())) {
    entry_list->resize(index + 1);
  }
  (*entry_list)[index].pic = ref_pic;
  (*entry_list)[index].data = pic_data;
  (*entry_list)[index].poc = ref_poc;
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

void ReferencePictureLists::Clear() {
  l0_.clear();
  l1_.clear();
}

}   // namespace xvc
