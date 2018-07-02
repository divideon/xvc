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

#include "xvc_common_lib/reference_picture_lists.h"

#include <algorithm>
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

bool ReferencePictureLists::HasRefPoc(RefPicList ref_list, PicNum poc) const {
  const std::vector<RefEntry> &entry_list =
    ref_list == RefPicList::kL0 ? l0_ : l1_;
  return std::find_if(entry_list.begin(), entry_list.end(),
                      [poc](const RefEntry &entry) {
    return entry.poc == poc;
  }) != entry_list.end();
}

PicturePredictionType
ReferencePictureLists::GetRefPicType(RefPicList ref_list, int ref_idx) const {
  const std::vector<RefEntry> &entry_list =
    ref_list == RefPicList::kL0 ? l0_ : l1_;
  if (static_cast<int>(entry_list.size()) <= ref_idx) {
    return PicturePredictionType::kInvalid;
  }
  return entry_list[ref_idx].data->GetPredictionType();
}

int
ReferencePictureLists::GetRefPicTid(RefPicList ref_list, int ref_idx) const {
  const std::vector<RefEntry> &entry_list =
    ref_list == RefPicList::kL0 ? l0_ : l1_;
  if (static_cast<int>(entry_list.size()) <= ref_idx) {
    return -1;
  }
  return entry_list[ref_idx].data->GetTid();
}

const CodingUnit*
ReferencePictureLists::GetCodingUnitAt(RefPicList ref_list, int index,
                                       CuTree cu_tree, int posx,
                                       int posy) const {
  const std::vector<RefEntry> &entry_list =
    ref_list == RefPicList::kL0 ? l0_ : l1_;
  std::shared_ptr<const PictureData> pic_data = entry_list[index].data;
  return pic_data->GetCuAt(cu_tree, posx, posy);
}

void ReferencePictureLists::SetRefPic(
  RefPicList list, int index, PicNum ref_poc,
  const std::shared_ptr<const PictureData> &pic_data,
  const std::shared_ptr<const YuvPicture> &ref_pic,
  const std::shared_ptr<const YuvPicture> &orig_pic) {
  std::vector<RefEntry> *entry_list = list == RefPicList::kL0 ? &l0_ : &l1_;
  if (index >= static_cast<int>(entry_list->size())) {
    entry_list->resize(index + 1);
  }
  (*entry_list)[index].ref_pic = ref_pic;
  (*entry_list)[index].orig_pic = orig_pic;
  (*entry_list)[index].data = pic_data;
  (*entry_list)[index].poc = ref_poc;
  if (ref_poc > current_poc_) {
    only_back_references_ = false;
  }
}

std::vector<int>
ReferencePictureLists::GetSamePocMappingFor(RefPicList ref_list) const {
  RefPicList other_list = ReferencePictureLists::Inverse(ref_list);
  int num_ref_pics = GetNumRefPics(ref_list);
  int other_ref_pics = GetNumRefPics(other_list);
  std::vector<int> mapping(num_ref_pics);
  for (int i = 0; i < num_ref_pics; i++) {
    mapping[i] = -1;
    PicNum ref_poc = GetRefPoc(ref_list, i);
    for (int j = 0; j < other_ref_pics; j++) {
      if (ref_poc == GetRefPoc(other_list, j)) {
        mapping[i] = j;
        break;
      }
    }
  }
  return mapping;
}

void ReferencePictureLists::ZeroOutReferences() {
  for (auto &ref : l0_) {
    ref.ref_pic.reset();
    ref.orig_pic.reset();
    ref.data.reset();
  }
  for (auto &ref : l1_) {
    ref.ref_pic.reset();
    ref.orig_pic.reset();
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
