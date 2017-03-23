/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_REFERENCE_LIST_SORTER_H_
#define XVC_COMMON_LIB_REFERENCE_LIST_SORTER_H_

#include <limits>
#include <memory>
#include <vector>

#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/reference_picture_lists.h"

namespace xvc {

template<class T>
class ReferenceListSorter {
public:
  explicit ReferenceListSorter(bool prev_segment_open_gop, int num_ref_pics)
    : prev_segment_open_gop_(prev_segment_open_gop),
    num_ref_pics_(num_ref_pics) {
  }

  void PrepareRefPicLists(std::shared_ptr<PictureData> curr_pic,
                          const std::vector<std::shared_ptr<T>> &pic_buffer,
                          ReferencePictureLists* rpl) {
    rpl->Reset(curr_pic->GetPoc());
    if (curr_pic->GetPredictionType() == PicturePredictionType::kIntra) {
      return;
    }
    int num_l0 = FillLowerPoc(RefPicList::kL0, 0, curr_pic, pic_buffer, rpl);
    FillHigherPoc(RefPicList::kL0, num_l0, curr_pic, pic_buffer, rpl);
    int num_l1 = FillHigherPoc(RefPicList::kL1, 0, curr_pic, pic_buffer, rpl);
    FillLowerPoc(RefPicList::kL1, num_l1, curr_pic, pic_buffer, rpl);
  }

private:
  int FillLowerPoc(RefPicList ref_pic_list, int start_idx,
                   std::shared_ptr<PictureData> curr_pic,
                   const std::vector<std::shared_ptr<T>> &pic_buffer,
                   ReferencePictureLists* rpl) {
    PicNum last_added_poc = curr_pic->GetPoc();
    int last_added_tid = curr_pic->GetTid();
    int ref_idx = start_idx;

    while (ref_idx < num_ref_pics_) {
      PicNum highest_poc_plus1 = 0;
      std::shared_ptr<const T> pic_enc_dec;
      for (auto &pic : pic_buffer) {
        auto pic_data = pic->GetPicData();
        if (pic_data->GetSoc() == curr_pic->GetSoc() &&
            pic_data->GetPoc() < last_added_poc &&
            pic_data->GetPoc() + 1 > highest_poc_plus1 &&
            (pic_data->GetTid() < last_added_tid || pic_data->GetTid() == 0)) {
          pic_enc_dec = pic;
          highest_poc_plus1 = pic_data->GetPoc() + 1;
        }
      }
      if (!pic_enc_dec) {
        break;
      }
      last_added_tid = pic_enc_dec->GetPicData()->GetTid();
      last_added_poc = highest_poc_plus1 - 1;
      rpl->SetRefPic(ref_pic_list, ref_idx, pic_enc_dec->GetPicData()->GetPoc(),
                     pic_enc_dec->GetPicData(),
                     pic_enc_dec->GetRecPic());
      ref_idx++;
    }
    return ref_idx;
  }

  int FillHigherPoc(RefPicList ref_pic_list, int start_idx,
                    std::shared_ptr<PictureData> curr_pic,
                    const std::vector<std::shared_ptr<T>> &pic_buffer,
                    ReferencePictureLists* rpl) {
    PicNum last_added_poc = curr_pic->GetPoc();
    int last_added_tid = curr_pic->GetTid();
    int ref_idx = start_idx;

    while (ref_idx < num_ref_pics_) {
      PicNum lowest_poc = std::numeric_limits<PicNum>::max();
      std::shared_ptr<const T> pic_enc_dec;
      for (auto &pic : pic_buffer) {
        auto pic_data = pic->GetPicData();
        SegmentNum curr_soc = curr_pic->GetSoc();
        bool same_or_previous_segment = pic_data->GetSoc() == curr_soc ||
          (pic_data->GetSoc() == static_cast<SegmentNum>(curr_soc + 1) &&
           prev_segment_open_gop_);
        if (same_or_previous_segment &&
            pic_data->GetPoc() > last_added_poc &&
            pic_data->GetPoc() < lowest_poc &&
            (pic_data->GetTid() < last_added_tid || pic_data->GetTid() == 0)) {
          pic_enc_dec = pic;
          lowest_poc = pic_data->GetPoc();
        }
      }
      if (!pic_enc_dec) {
        break;
      }
      last_added_tid = pic_enc_dec->GetPicData()->GetTid();
      last_added_poc = lowest_poc;
      auto ref_data = pic_enc_dec->GetPicData();
      auto ref_pic = pic_enc_dec->GetRecPic();
      if (curr_pic->GetSoc() != ref_data->GetSoc() &&
          !PictureData::IsSameDimension(*curr_pic, *ref_data)) {
        ref_pic = pic_enc_dec->GetAlternativeRecPic(
          curr_pic->GetChromaFormat(),
          curr_pic->GetPictureWidth(YuvComponent::kY),
          curr_pic->GetPictureHeight(YuvComponent::kY),
          curr_pic->GetBitdepth());
      }
      rpl->SetRefPic(ref_pic_list, ref_idx, pic_enc_dec->GetPicData()->GetPoc(),
                     pic_enc_dec->GetPicData(),
                     pic_enc_dec->GetRecPic());
      ref_idx++;
    }
    return ref_idx;
  }

  bool prev_segment_open_gop_;
  int num_ref_pics_;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_REFERENCE_LIST_SORTER_H_
