/******************************************************************************
* Copyright (C) 2017, Divideon.
*
* Redistribution and use in source and binary form, with or without
* modifications is permitted only under the terms and conditions set forward
* in the xvc License Agreement. For commercial redistribution and use, you are
* required to send a signed copy of the xvc License Agreement to Divideon.
*
* Redistribution and use in source and binary form is permitted free of charge
* for non-commercial purposes. See definition of non-commercial in the xvc
* License Agreement.
*
* All redistribution of source code must retain this copyright notice
* unmodified.
*
* The xvc License Agreement is available at https://xvc.io/license/.
******************************************************************************/

#ifndef XVC_COMMON_LIB_REFERENCE_LIST_SORTER_H_
#define XVC_COMMON_LIB_REFERENCE_LIST_SORTER_H_

#include <limits>
#include <memory>
#include <vector>

#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/reference_picture_lists.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/segment_header.h"

namespace xvc {

template<class T>
class ReferenceListSorter {
public:
  ReferenceListSorter(const SegmentHeader &segment_header,
                      bool prev_segment_open_gop)
    : segment_header_(segment_header),
    prev_segment_open_gop_(prev_segment_open_gop) {
  }

  std::vector<std::shared_ptr<const T>>
    Prepare(PicNum curr_poc, int curr_tid, bool is_intra_pic,
            const std::vector<std::shared_ptr<T>> &pic_buffer,
            ReferencePictureLists* rpl, int leading_pictures) {
    std::vector<std::shared_ptr<const T>> dependencies;
    if (rpl) {
      rpl->Reset(curr_poc);
    }
    if (is_intra_pic) {
      return dependencies;
    }
    if (Restrictions::Get().disable_inter_bipred) {
      FillClosestPoc(RefPicList::kL0, 0, curr_poc, curr_tid, pic_buffer,
                     &dependencies, rpl);
      return dependencies;
    }
    int num_l0 = FillLowerPoc(RefPicList::kL0, 0, curr_poc, curr_tid,
                              pic_buffer, &dependencies, rpl);
    if (Restrictions::Get().disable_ext_ref_list_l0_trim
        || num_l0 == 0) {
      FillHigherPoc(RefPicList::kL0, num_l0, curr_poc, curr_tid, pic_buffer,
                    &dependencies, rpl);
    }
    int num_l1 = FillHigherPoc(RefPicList::kL1, 0, curr_poc, curr_tid,
                               pic_buffer, &dependencies, rpl);
    FillLowerPoc(RefPicList::kL1, num_l1, curr_poc, curr_tid, pic_buffer,
                 &dependencies, rpl);
    return dependencies;
  }

private:
  int FillLowerPoc(RefPicList ref_pic_list, int start_idx,
                   PicNum curr_poc, int curr_tid,
                   const std::vector<std::shared_ptr<T>> &pic_buffer,
                   std::vector<std::shared_ptr<const T>> *dependencies,
                   ReferencePictureLists* rpl) {
    PicNum last_added_poc = curr_poc;
    int last_added_tid = curr_tid;
    int ref_idx = start_idx;

    while (ref_idx < segment_header_.num_ref_pics) {
      PicNum highest_poc_plus1 = 0;
      std::shared_ptr<const T> pic_enc_dec;
      for (auto &pic : pic_buffer) {
        auto pic_data = pic->GetPicData();
        if (pic_data->GetSoc() == segment_header_.soc &&
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
      if (rpl) {
        rpl->SetRefPic(ref_pic_list, ref_idx,
                       pic_enc_dec->GetPicData()->GetPoc(),
                       pic_enc_dec->GetPicData(), pic_enc_dec->GetRecPic(),
                       pic_enc_dec->GetOrigPic());
      }
      dependencies->push_back(pic_enc_dec);
      ref_idx++;
    }
    return ref_idx;
  }

  int FillHigherPoc(RefPicList ref_pic_list, int start_idx,
                    PicNum curr_poc, int curr_tid,
                    const std::vector<std::shared_ptr<T>> &pic_buffer,
                    std::vector<std::shared_ptr<const T>> *dependencies,
                    ReferencePictureLists* rpl) {
    PicNum last_added_poc = curr_poc;
    int last_added_tid = curr_tid;
    int ref_idx = start_idx;

    while (ref_idx < segment_header_.num_ref_pics) {
      PicNum lowest_poc = std::numeric_limits<PicNum>::max();
      std::shared_ptr<const T> pic_enc_dec;
      for (auto &pic : pic_buffer) {
        auto pic_data = pic->GetPicData();
        SegmentNum curr_soc = segment_header_.soc;
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
      if (segment_header_.soc != ref_data->GetSoc() &&
          !IsSameDimension(segment_header_, *ref_data)) {
        ref_pic = pic_enc_dec->GetAlternativeRecPic(
          segment_header_.GetInternalPicFormat(),
          segment_header_.GetCropWidth(),
          segment_header_.GetCropHeight());
      }
      if (rpl) {
        rpl->SetRefPic(ref_pic_list, ref_idx, ref_data->GetPoc(),
                       ref_data, ref_pic, pic_enc_dec->GetOrigPic());
      }
      dependencies->push_back(pic_enc_dec);
      ref_idx++;
    }
    return ref_idx;
  }

  int FillClosestPoc(RefPicList ref_pic_list, int start_idx,
                     PicNum curr_poc, int curr_tid,
                     const std::vector<std::shared_ptr<T>> &pic_buffer,
                     std::vector<std::shared_ptr<const T>> *dependencies,
                     ReferencePictureLists* rpl) {
    PicNum last_added_poc0 = curr_poc;
    int last_added_tid0 = curr_tid;
    PicNum last_added_poc1 = curr_poc;
    int last_added_tid1 = curr_tid;
    int ref_idx = start_idx;

    while (ref_idx < segment_header_.num_ref_pics) {
      PicNum lowest_poc = std::numeric_limits<PicNum>::max();
      PicNum highest_poc_plus1 = 0;
      std::shared_ptr<const T> pic_enc_dec0;
      std::shared_ptr<const T> pic_enc_dec1;
      for (auto &pic : pic_buffer) {
        auto pic_data = pic->GetPicData();
        SegmentNum curr_soc = segment_header_.soc;
        bool same_or_previous_segment = pic_data->GetSoc() == curr_soc ||
          (pic_data->GetSoc() == static_cast<SegmentNum>(curr_soc + 1) &&
           prev_segment_open_gop_);
        if (same_or_previous_segment &&
            pic_data->GetPoc() > last_added_poc1 &&
            pic_data->GetPoc() < lowest_poc &&
            (pic_data->GetTid() < last_added_tid1 || pic_data->GetTid() == 0)) {
          pic_enc_dec1 = pic;
          lowest_poc = pic_data->GetPoc();
        } else if (pic_data->GetSoc() == segment_header_.soc &&
                   pic_data->GetPoc() < last_added_poc0 &&
                   pic_data->GetPoc() + 1 > highest_poc_plus1 &&
                   (pic_data->GetTid() < last_added_tid0 ||
                    pic_data->GetTid() == 0)) {
          pic_enc_dec0 = pic;
          highest_poc_plus1 = pic_data->GetPoc() + 1;
        }
      }
      if (!(pic_enc_dec0 || pic_enc_dec1)) {
        break;
      }
      if (highest_poc_plus1 == 0 ||
          lowest_poc - curr_poc <= curr_poc - highest_poc_plus1) {
        last_added_tid1 = pic_enc_dec1->GetPicData()->GetTid();
        last_added_poc1 = lowest_poc;
        auto ref_data = pic_enc_dec1->GetPicData();
        auto ref_pic = pic_enc_dec1->GetRecPic();
        if (segment_header_.soc != ref_data->GetSoc() &&
            !IsSameDimension(segment_header_, *ref_data)) {
          ref_pic = pic_enc_dec1->GetAlternativeRecPic(
            segment_header_.GetInternalPicFormat(),
            segment_header_.GetCropWidth(),
            segment_header_.GetCropHeight());
        }
        if (rpl) {
          rpl->SetRefPic(ref_pic_list, ref_idx,
                         pic_enc_dec1->GetPicData()->GetPoc(),
                         pic_enc_dec1->GetPicData(), ref_pic,
                         pic_enc_dec1->GetOrigPic());
        }
        dependencies->push_back(pic_enc_dec1);
      } else {
        last_added_tid0 = pic_enc_dec0->GetPicData()->GetTid();
        last_added_poc0 = highest_poc_plus1 - 1;
        if (rpl) {
          rpl->SetRefPic(ref_pic_list, ref_idx,
                         pic_enc_dec0->GetPicData()->GetPoc(),
                         pic_enc_dec0->GetPicData(),
                         pic_enc_dec0->GetRecPic(),
                         pic_enc_dec0->GetOrigPic());
        }
        dependencies->push_back(pic_enc_dec0);
      }
      ref_idx++;
    }
    return ref_idx;
  }

  static bool IsSameDimension(const SegmentHeader &segment_header,
                              const PictureData &pic) {
    const YuvComponent luma = YuvComponent::kY;
    return segment_header.GetInternalWidth() == pic.GetPictureWidth(luma) &&
      segment_header.GetInternalHeight() == pic.GetPictureHeight(luma) &&
      segment_header.chroma_format == pic.GetChromaFormat() &&
      segment_header.internal_bitdepth == pic.GetBitdepth();
  }

  const SegmentHeader &segment_header_;
  bool prev_segment_open_gop_;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_REFERENCE_LIST_SORTER_H_
