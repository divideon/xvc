/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/encoder.h"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <utility>

#include "xvc_common_lib/reference_list_sorter.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/segment_header.h"
#include "xvc_enc_lib/segment_header_writer.h"

namespace xvc {

Encoder::Encoder(int internal_bitdepth, int base_qp)
  : segment_qp_(base_qp) {
  segment_header_.codec_identifier = constants::kXvcCodecIdentifier;
  segment_header_.major_version = constants::kXvcMajorVersion;
  segment_header_.minor_version = constants::kXvcMinorVersion;
  segment_header_.internal_bitdepth = internal_bitdepth;
}

int Encoder::Encode(const uint8_t *pic_bytes, xvc_enc_nal_unit **nal_units,
                    bool output_rec, xvc_enc_pic_buffer *rec_pic) {
  nal_units_.clear();

  // Set picture parameters and get bytes for original picture
  auto pic_enc = GetNewPictureEncoder();
  auto pic_data = pic_enc->GetPicData();
  pic_data->SetOutputStatus(OutputStatus::kHasNotBeenOutput);
  pic_data->SetPoc(poc_);
  if (all_intra_) {
    pic_data->SetNalType(NalUnitType::kIntraPicture);
  } else {
    pic_data->SetNalType(NalUnitType::kBipredictedPicture);
  }
  PicNum doc =
    SegmentHeader::CalcDocFromPoc(poc_, segment_header_.max_sub_gop_length,
                                  sub_gop_start_poc_);
  int tid =
    SegmentHeader::CalcTidFromDoc(doc, segment_header_.max_sub_gop_length,
                                  sub_gop_start_poc_);
  pic_data->SetDoc(doc);
  pic_data->SetTid(tid);
  pic_data->SetDeblock(segment_header_.deblock > 0);
  pic_data->SetBetaOffset(segment_header_.beta_offset);
  pic_data->SetTcOffset(segment_header_.tc_offset);

  pic_enc->GetOrigPic()->CopyFrom(pic_bytes, input_bitdepth_);

  // Check if it is time to encode a new segment header.
  if ((poc_ % segment_length_) == 0) {
    prev_segment_open_gop_ = curr_segment_open_gop_;
    if (((poc_ + segment_length_) % closed_gop_interval_) == 0) {
      curr_segment_open_gop_ = false;
    } else {
      curr_segment_open_gop_ = true;
    }
    SegmentHeaderWriter::Write(&segment_header_, &bit_writer_, framerate_,
                               curr_segment_open_gop_);
    soc_++;
    xvc_enc_nal_unit nal;
    std::vector<uint8_t> *nal_bytes = bit_writer_.GetBytes();
    nal.bytes = &(*nal_bytes)[0];
    nal.size = nal_bytes->size();
    nal.stats.nal_unit_type = 16;
    nal.buffer_flag = 0;
    nal_units_.push_back(nal);
    pic_data->SetNalType(NalUnitType::kIntraAccessPicture);
    buffer_flag_ = 1;
  } else {
    buffer_flag_ = 0;
  }
  pic_data->SetSoc(soc_);


  if (poc_ == 0) {
    EncodeOnePicture(pic_enc);
    doc_ = 0;
  }

  // Check if there are enough pictures buffered to encode a new Sub Gop
  if (poc_ == doc_ + segment_header_.max_sub_gop_length) {
    for (PicNum i = 0; i < segment_header_.max_sub_gop_length; i++) {
      // Find next picture to encode by searching for
      // the one that has doc = this->doc_ + 1.
      for (auto &pic : pic_encoders_) {
        if (pic->GetPicData()->GetDoc() == doc_ + 1) {
          EncodeOnePicture(pic);
        }
      }
    }
    sub_gop_start_poc_ = doc_;
  }

  // Put tail pictures before the segment header in network order.
  std::stable_sort(nal_units_.begin(), nal_units_.end(),
                   [](xvc_enc_nal_unit n1, xvc_enc_nal_unit n2) {
    return n1.buffer_flag > n2.buffer_flag;
  });

  rec_pic->pic = nullptr;
  rec_pic->size = 0;
  // If enough pictures have been encoded, the reconstructed picture
  // with lowest poc can be output.
  if (poc_ >= segment_header_.max_sub_gop_length) {
    ReconstructOnePicture(output_rec, rec_pic);
  }
  // Increase encoder poc counter by one for each call to Encode.
  // poc_ is initialized to 0.
  poc_++;

  if (nal_units_.size() > 0) {
    *nal_units = &nal_units_[0];
  }
  return static_cast<int>(nal_units_.size());
}

int Encoder::Flush(xvc_enc_nal_unit **nal_units, bool output_rec,
                   xvc_enc_pic_buffer *rec_pic) {
  nal_units_.clear();
  // Since poc is increased at the end of each call to Encode
  // it is reduced by one here to get the poc of the last picture.
  poc_--;
  // Check if there are pictures left to encode.
  if (doc_ < poc_) {
    // Use a smaller Sub Gop for the pictures that have not been encoded yet.
    segment_header_.max_sub_gop_length = poc_ - doc_;
    buffer_flag_ = 0;
    // Recount doc and tid for the pictures to be encoded.
    for (auto &pic : pic_encoders_) {
      auto pd = pic->GetPicData();
      if (pd->GetPoc() > doc_) {
        PicNum doc =
          SegmentHeader::CalcDocFromPoc(pd->GetPoc(),
                                        segment_header_.max_sub_gop_length,
                                        sub_gop_start_poc_);
        int tid =
          SegmentHeader::CalcTidFromDoc(doc, segment_header_.max_sub_gop_length,
                                        sub_gop_start_poc_);
        pd->SetDoc(doc);
        pd->SetTid(tid);
      }
    }
    for (PicNum i = 0; i < segment_header_.max_sub_gop_length; i++) {
      // Find next picture to encode by searching for
      // the one that has doc = this->doc_ + 1.
      for (auto &pic : pic_encoders_) {
        if (pic->GetPicData()->GetDoc() == doc_ + 1) {
          EncodeOnePicture(pic);
        }
      }
    }
  }

  // Increase poc by one for each call to Flush.
  poc_++;

  rec_pic->pic = nullptr;
  rec_pic->size = 0;
  // Check if reconstruction should be performed.
  ReconstructOnePicture(output_rec, rec_pic);
  if (nal_units_.size() > 0) {
    *nal_units = &nal_units_[0];
  }
  return static_cast<int>(nal_units_.size());
}

void Encoder::EncodeOnePicture(std::shared_ptr<PictureEncoder> pic) {
  // Check if current picture is a tail picture.
  // This means that it will be sent before the key picture of the
  // next segment and then be buffered in the decoder to be decoded after
  // the key picture.
  int bflag = (buffer_flag_ &&
    (pic->GetPicData()->GetNalType() != NalUnitType::kIntraAccessPicture));

  ReferenceListSorter<PictureEncoder> ref_list_sorter(prev_segment_open_gop_);
  ref_list_sorter.PrepareRefPicLists(pic->GetPicData(), pic_encoders_,
                                     pic->GetPicData()->GetRefPicLists());

  // Bitstream reference valid until next picture is coded
  std::vector<uint8_t> *pic_bytes =
    pic->Encode(segment_qp_, segment_header_.max_sub_gop_length, bflag,
                flat_lambda_);

  // When a picture has been encoded the picture data is put into
  // the xvc_enc_nal_unit struct to be delivered through the API.
  xvc_enc_nal_unit nal;
  nal.bytes = &(*pic_bytes)[0];
  nal.size = pic_bytes->size();
  nal.buffer_flag = bflag;
  SetNalStats(&nal, pic);
  nal_units_.push_back(nal);

  // Decoding order counter is increased each time a picture has been encoded.
  doc_++;
}

void Encoder::ReconstructOnePicture(bool output_rec,
                                    xvc_enc_pic_buffer *rec_pic) {
  // Find the picture with the lowest poc that has not been output.
  std::shared_ptr<PictureEncoder> pic_enc;
  PicNum lowest_poc = std::numeric_limits<PicNum>::max();
  for (auto &pic : pic_encoders_) {
    auto pd = pic->GetPicData();
    if ((pd->GetOutputStatus() == OutputStatus::kHasNotBeenOutput) &&
      (pd->GetPoc() < lowest_poc)) {
      pic_enc = pic;
      lowest_poc = pd->GetPoc();
    }
  }
  if (!pic_enc) {
    return;
  }
  pic_enc->GetPicData()->SetOutputStatus(OutputStatus::kHasBeenOutput);
  auto rec_pic_out = pic_enc->GetPicData()->GetRecPic();

  // Only perform reconstruction if it is requested and a picture was found.
  if (output_rec && rec_pic_out) {
    rec_pic_out->CopyToSameBitdepth(&output_pic_bytes_);
    rec_pic->size = output_pic_bytes_.size();
    rec_pic->pic = &output_pic_bytes_[0];
  }
}

std::shared_ptr<PictureEncoder> Encoder::GetNewPictureEncoder() {
  // Allocate a new PictureEncoder if the number of buffered pictures
  // is lower than the maximum that will be used.
  if (pic_encoders_.size() < pic_buffering_num_) {
    auto pic = std::make_shared<PictureEncoder>(
      segment_header_.chroma_format, segment_header_.pic_width,
      segment_header_.pic_height, segment_header_.internal_bitdepth);
    pic_encoders_.push_back(pic);
    return pic;
  }

  // Reuse a PictureEncoder if the number of buffered pictures
  // is equal to the maximum that will be used.
  // Pick any that has been output and has tid higher than 0.
  // If no pictures with tid higher than 0 is available, reuse the
  // picture with the lowest poc.
  PicNum lowest_poc = std::numeric_limits<PicNum>::max();
  std::shared_ptr<PictureEncoder> pic_enc;
  for (auto &pic : pic_encoders_) {
    auto pic_data = pic->GetPicData();
    if ((pic_data->GetOutputStatus() == OutputStatus::kHasBeenOutput) &&
        pic_data->GetTid() > 0) {
      return pic;
    } else if (pic_data->GetPoc() < lowest_poc) {
      pic_enc = pic;
      lowest_poc = pic_data->GetPoc();
    }
  }
  assert(pic_enc);
  return pic_enc;
}

void Encoder::SetNalStats(xvc_enc_nal_unit *nal,
                          std::shared_ptr<PictureEncoder> pic) {
  auto pic_data = pic->GetPicData();
  nal->stats.nal_unit_type =
    static_cast<uint32_t>(pic_data->GetNalType());

  // Expose the 32 least significant bits of poc and doc.
  nal->stats.poc = static_cast<uint32_t>(pic_data->GetPoc());
  nal->stats.doc = static_cast<uint32_t>(pic_data->GetDoc());
  nal->stats.soc = static_cast<uint32_t>(pic_data->GetSoc());
  nal->stats.tid = pic_data->GetTid();
  nal->stats.qp = pic_data->GetPicQp()->GetQpRaw(YuvComponent::kY);

  // Expose the first reference pictures in L0 and L1.
  ReferencePictureLists* rpl = pic_data->GetRefPicLists();
  int length = sizeof(nal->stats.l0) / sizeof(nal->stats.l0[0]);
  for (int i = 0; i < length; i++) {
    if (i < rpl->GetNumRefPics(RefPicList::kL0)) {
      nal->stats.l0[i] =
        static_cast<int32_t>(rpl->GetRefPoc(RefPicList::kL0, i));
    } else {
      nal->stats.l0[i] = -1;
    }
    if (i < rpl->GetNumRefPics(RefPicList::kL1)) {
      nal->stats.l1[i] =
        static_cast<int32_t>(rpl->GetRefPoc(RefPicList::kL1, i));
    } else {
      nal->stats.l1[i] = -1;
    }
  }
}

}   // namespace xvc
