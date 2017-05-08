/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_REFERENCE_PICTURE_LISTS_H_
#define XVC_COMMON_LIB_REFERENCE_PICTURE_LISTS_H_

#include <memory>
#include <vector>

#include "xvc_common_lib/cu_types.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/yuv_pic.h"

namespace xvc {

enum class RefPicList {
  kL0 = 0,
  kL1 = 1,
  kTotalNumber = 2
};

class CodingUnit;
class PictureData;

class ReferencePictureLists {
public:
  static bool IsRefPicListUsed(RefPicList ref_pic_list, InterDir inter_dir);
  static RefPicList Inverse(RefPicList ref_list) {
    return ref_list == RefPicList::kL0 ? RefPicList::kL1 : RefPicList::kL0;
  }

  int GetNumRefPics(RefPicList list) const {
    return (list == RefPicList::kL0) ?
      static_cast<int>(l0_.size()) : static_cast<int>(l1_.size());
  }
  const YuvPicture* GetRefPic(RefPicList ref_list, int ref_idx) const {
    return ref_list == RefPicList::kL0 ?
      l0_[ref_idx].pic.get() : l1_[ref_idx].pic.get();
  }
  PicNum GetRefPoc(RefPicList ref_list, int ref_idx) const {
    return (ref_list == RefPicList::kL0) ? l0_[ref_idx].poc : l1_[ref_idx].poc;
  }
  bool HasOnlyBackReferences() const { return only_back_references_; }
  PicturePredictionType GetRefPicType(RefPicList ref_list, int ref_idx) const;
  int GetRefPicTid(RefPicList ref_list, int ref_idx) const;
  const CodingUnit* GetCodingUnitAt(RefPicList ref_list, int ref_idx,
                                    CuTree cu_tree, int posx, int posy) const;
  void SetRefPic(RefPicList ref_list, int index, PicNum ref_poc,
                 const std::shared_ptr<const PictureData> &pic_data,
                 const std::shared_ptr<const YuvPicture> &ref_pic);
  void GetSamePocMappingFor(RefPicList ref_list,
                            std::vector<int> *mapping) const;
  void ZeroOutReferences();
  void Reset(PicNum current_poc);

private:
  struct RefEntry {
    PicNum poc;
    std::shared_ptr<const YuvPicture> pic;
    std::shared_ptr<const PictureData> data;
  };
  std::vector<RefEntry> l0_;
  std::vector<RefEntry> l1_;
  PicNum current_poc_ = static_cast<PicNum>(-1);
  bool only_back_references_ = true;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_REFERENCE_PICTURE_LISTS_H_
