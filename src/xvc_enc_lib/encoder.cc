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

#include "xvc_common_lib/restrictions.h"

namespace xvc {

int Encoder::Encode(const uint8_t *pic_bytes, xvc_enc_nal_unit **nal_units,
                    bool output_rec, xvc_enc_pic_buffer *rec_pic) {
  nal_units_.clear();

  // Set picture parameters and get bytes for original picture
  auto pic_enc = GetNewPictureEncoder();
  auto pic_data = pic_enc->GetPicData();
  pic_data->SetOutputStatus(OutputStatus::kHasNotBeenOutput);
  pic_data->SetPoc(poc_);
  if (all_intra_) {
    pic_data->SetPicType(NalUnitType::kIntraPicture);
  } else {
    pic_data->SetPicType(NalUnitType::kBipredictedPicture);
  }
  pic_data->CalcDocFromPoc(sub_gop_length_, sub_gop_start_poc_);
  pic_data->CalcTidFromDoc(sub_gop_length_, sub_gop_start_poc_);
  pic_data->SetDeblock(deblock_ > 0);
  pic_data->SetBetaOffset(beta_offset_);
  pic_data->SetTcOffset(tc_offset_);

  pic_enc->GetOrigPic()->CopyFrom(pic_bytes, input_bitdepth_);

  // Check if it is time to encode a new segment header.
  if ((poc_ % segment_length_) == 0) {
    EncodeSegmentHeader();
    pic_data->SetPicType(NalUnitType::kIntraAccessPicture);
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
  if (poc_ == doc_ + sub_gop_length_) {
    for (PicNum i = 0; i < sub_gop_length_; i++) {
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
  if (poc_ >= sub_gop_length_) {
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
    sub_gop_length_ = poc_ - doc_;
    buffer_flag_ = 0;
    // Recount doc and tid for the pictures to be encoded.
    for (auto &pic : pic_encoders_) {
      auto pd = pic->GetPicData();
      if (pd->GetPoc() > doc_) {
        pd->CalcDocFromPoc(sub_gop_length_, sub_gop_start_poc_);
        pd->CalcTidFromDoc(sub_gop_length_, sub_gop_start_poc_);
      }
    }
    for (PicNum i = 0; i < sub_gop_length_; i++) {
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
    (pic->GetPicData()->GetPicType() != NalUnitType::kIntraAccessPicture));

  PrepareRefPicLists(pic);

  // Bitstream reference valid until next picture is coded
  std::vector<uint8_t> &pic_bytes =
    pic->Encode(base_qp_, sub_gop_length_, bflag, flat_lambda_);

  // When a picture has been encoded the picture data is put into
  // the xvc_enc_nal_unit struct to be delivered through the API.
  xvc_enc_nal_unit nal;
  nal.bytes = &pic_bytes[0];
  nal.size = pic_bytes.size();
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

void Encoder::EncodeSegmentHeader() {
  bit_writer_.Clear();
  bit_writer_.WriteBits(33, 8);  // Nal Unit header with nal_unit_type == 16
  bit_writer_.WriteBits(constants::kXvcMajorVersion, 16);
  bit_writer_.WriteBits(constants::kXvcMinorVersion, 16);
  bit_writer_.WriteBits(pic_width_, 16);
  bit_writer_.WriteBits(pic_height_, 16);
  bit_writer_.WriteBits(static_cast<uint8_t>(chroma_format_), 4);
  bit_writer_.WriteBits(internal_bitdepth_ - 8, 4);
  bit_writer_.WriteBits(static_cast<uint32_t>(constants::kTimeScale
                                              / framerate_), 24);
  bit_writer_.WriteBits(base_qp_ + 64, 7);
  bit_writer_.WriteBits(static_cast<uint32_t>(sub_gop_length_), 8);
  bit_writer_.WriteBit(1);  // shorter_sub_gops_allowed
  // The open gop flag is set to 0 if the tail pictures
  // do not predict from the Intra picture in the next segment.
  prev_segment_open_gop_ = curr_segment_open_gop_;
  if (((poc_ + segment_length_) % closed_gop_interval_) == 0) {
    bit_writer_.WriteBit(0);  // open_gop = 0 (closed Gop)
    curr_segment_open_gop_ = false;
  } else {
    bit_writer_.WriteBit(1);  // open_gop = 1 (open Gop)
    curr_segment_open_gop_ = true;
  }
  bit_writer_.WriteBits(deblock_, 2);
  if (deblock_ == 3) {
    int d = constants::kDeblockOffsetBits;
    bit_writer_.WriteBits(beta_offset_ + (1 << (d - 1)), d);
    bit_writer_.WriteBits(tc_offset_ + (1 << (d - 1)), d);
  }
  bit_writer_.WriteBits(0, 4);  // num_ref_pic_r0
  bit_writer_.WriteBits(0, 4);  // num_ref_pic_r1

  auto &restr = Restrictions::Get();
  if (Restrictions::GetIntraRestrictions()) {
    bit_writer_.WriteBit(1);  // intra_restrictions
    bit_writer_.WriteBit(restr.disable_intra_ref_padding);
    bit_writer_.WriteBit(restr.disable_intra_ref_sample_filter);
    bit_writer_.WriteBit(restr.disable_intra_dc_post_filter);
    bit_writer_.WriteBit(restr.disable_intra_ver_hor_post_filter);
    bit_writer_.WriteBit(restr.disable_intra_planar);
    bit_writer_.WriteBit(restr.disable_intra_mpm_prediction);
    bit_writer_.WriteBit(restr.disable_intra_chroma_predictor);
  } else {
    bit_writer_.WriteBit(0);  // intra_restrictions
  }
  if (Restrictions::GetInterRestrictions()) {
    bit_writer_.WriteBit(1);  // inter_restrictions
    bit_writer_.WriteBit(restr.disable_inter_mvp);
    bit_writer_.WriteBit(restr.disable_inter_scaling_mvp);
    bit_writer_.WriteBit(restr.disable_inter_merge_candidates);
    bit_writer_.WriteBit(restr.disable_inter_merge_mode);
    bit_writer_.WriteBit(restr.disable_inter_skip_mode);
    bit_writer_.WriteBit(restr.disable_inter_chroma_subpel);
    bit_writer_.WriteBit(restr.disable_inter_mvd_greater_than_flags);
  } else {
    bit_writer_.WriteBit(0);  // inter_restrictions
  }
  if (Restrictions::GetTransformRestrictions()) {
    bit_writer_.WriteBit(1);  // transform_restrictions
    bit_writer_.WriteBit(restr.disable_transform_adaptive_scan_order);
    bit_writer_.WriteBit(restr.disable_transform_residual_greater_than_flags);
    bit_writer_.WriteBit(restr.disable_transform_last_position);
    bit_writer_.WriteBit(restr.disable_transform_cbf);
    bit_writer_.WriteBit(restr.disable_transform_subblock_csbf);
    bit_writer_.WriteBit(restr.disable_transform_adaptive_exp_golomb);
  } else {
    bit_writer_.WriteBit(0);  // transform_restrictions
  }
  if (Restrictions::GetCabacRestrictions()) {
    bit_writer_.WriteBit(1);  // cabac_restrictions
    bit_writer_.WriteBit(restr.disable_cabac_ctx_update);
    bit_writer_.WriteBit(restr.disable_cabac_split_flag_ctx);
    bit_writer_.WriteBit(restr.disable_cabac_skip_flag_ctx);
    bit_writer_.WriteBit(restr.disable_cabac_subblock_csbf_ctx);
    bit_writer_.WriteBit(restr.disable_cabac_coeff_sig_ctx);
    bit_writer_.WriteBit(restr.disable_cabac_coeff_greater1_ctx);
    bit_writer_.WriteBit(restr.disable_cabac_coeff_greater2_ctx);
    bit_writer_.WriteBit(restr.disable_cabac_coeff_last_pos_ctx);
  } else {
    bit_writer_.WriteBit(0);  // cabac_restrictions
  }
  if (Restrictions::GetDeblockRestrictions()) {
    bit_writer_.WriteBit(1);  // deblock_restrictions
    bit_writer_.WriteBit(restr.disable_deblock_strong_filter);
    bit_writer_.WriteBit(restr.disable_deblock_weak_filter);
    bit_writer_.WriteBit(restr.disable_deblock_chroma_filter);
    bit_writer_.WriteBit(restr.disable_deblock_boundary_strength_zero);
    bit_writer_.WriteBit(restr.disable_deblock_boundary_strength_one);
    bit_writer_.WriteBit(restr.disable_deblock_initial_sample_decision);
    bit_writer_.WriteBit(restr.disable_deblock_two_samples_weak_filter);
    bit_writer_.WriteBit(restr.disable_deblock_depending_on_qp);
  } else {
    bit_writer_.WriteBit(0);  // deblock_restrictions
  }
  bit_writer_.PadZeroBits();

  // Increase segment order count
  soc_++;
  xvc_enc_nal_unit nal;
  nal.bytes = &bit_writer_.GetBytes()[0];
  nal.size = bit_writer_.GetBytes().size();
  nal.stats.nal_unit_type = 16;
  nal.buffer_flag = 0;
  nal_units_.push_back(nal);
}

std::shared_ptr<PictureEncoder> Encoder::GetNewPictureEncoder() {
  // Allocate a new PictureEncoder if the number of buffered pictures
  // is lower than the maximum that will be used.
  if (pic_encoders_.size() < pic_buffering_num_) {
    auto pic = std::make_shared<PictureEncoder>(chroma_format_,
                                                pic_width_,
                                                pic_height_,
                                                internal_bitdepth_);
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

void Encoder::PrepareRefPicLists(std::shared_ptr<PictureEncoder> curr_pic) {
  ReferencePictureLists* rpl = curr_pic->GetPicData()->GetRefPicLists();
  rpl->Reset(curr_pic->GetPicData()->GetPoc());
  if (curr_pic->GetPicData()->GetPredictionType() ==
      PicturePredictionType::kIntra) {
    return;
  }
  int num_pics_in_l0 =
    FillRefPicsLowerPoc(0, curr_pic, RefPicList::kL0);
  FillRefPicsHigherPoc(num_pics_in_l0, curr_pic, RefPicList::kL0);
  int num_pics_in_l1 =
    FillRefPicsHigherPoc(0, curr_pic, RefPicList::kL1);
  FillRefPicsLowerPoc(num_pics_in_l1, curr_pic, RefPicList::kL1);
}

int Encoder::FillRefPicsLowerPoc(int start_idx,
                                 std::shared_ptr<PictureEncoder> curr_pic,
                                 RefPicList ref_pic_list) {
  int idx = start_idx;
  PicNum curr_poc = curr_pic->GetPicData()->GetPoc();
  SegmentNum curr_soc = curr_pic->GetPicData()->GetSoc();
  PicNum last_added_poc = curr_poc;
  std::shared_ptr<PictureEncoder> pic_enc;
  ReferencePictureLists* rpl = curr_pic->GetPicData()->GetRefPicLists();
  int last_added_tid = curr_pic->GetPicData()->GetTid();
  bool found_pic = true;
  while (found_pic) {
    PicNum highest_poc_plus1 = 0;
    found_pic = false;
    for (auto &pic : pic_encoders_) {
      auto pic_data = pic->GetPicData();
      if ((pic_data->GetSoc() == curr_soc ||
        (pic_data->GetSoc() == static_cast<SegmentNum>(curr_soc + 1) &&
         prev_segment_open_gop_)) &&
          pic_data->GetPoc() < last_added_poc &&
          pic_data->GetPoc() + 1 > highest_poc_plus1 &&
          (pic_data->GetTid() < last_added_tid ||
           pic_data->GetTid() == 0)) {
        pic_enc = pic;
        highest_poc_plus1 = pic_data->GetPoc() + 1;
        found_pic = true;
      }
    }
    if (found_pic) {
      last_added_tid = pic_enc->GetPicData()->GetTid();
      last_added_poc = highest_poc_plus1 - 1;
      if (idx < constants::kNumPicsInRefPicLists) {
        rpl->SetRefPic(ref_pic_list, idx++, pic_enc->GetPicData()->GetPoc(),
                       pic_enc->GetPicData(),
                       pic_enc->GetPicData()->GetRecPic());
      }
    }
  }
  return idx;
}

int Encoder::FillRefPicsHigherPoc(int start_idx,
                                  std::shared_ptr<PictureEncoder> curr_pic,
                                  RefPicList ref_pic_list) {
  int idx = start_idx;
  PicNum curr_poc = curr_pic->GetPicData()->GetPoc();
  SegmentNum curr_soc = curr_pic->GetPicData()->GetSoc();
  PicNum last_added_poc = curr_poc;
  std::shared_ptr<PictureEncoder> pic_enc;
  ReferencePictureLists* rpl = curr_pic->GetPicData()->GetRefPicLists();
  int last_added_tid = curr_pic->GetPicData()->GetTid();
  bool found_pic = true;
  while (found_pic) {
    PicNum lowest_poc = std::numeric_limits<PicNum>::max();
    found_pic = false;
    for (auto &pic : pic_encoders_) {
      auto pic_data = pic->GetPicData();
      if ((pic_data->GetSoc() == curr_soc ||
        (pic_data->GetSoc() == static_cast<SegmentNum>(curr_soc + 1) &&
         prev_segment_open_gop_)) &&
          pic_data->GetPoc() > last_added_poc &&
          pic_data->GetPoc() < lowest_poc &&
          (pic_data->GetTid() < last_added_tid ||
           pic_data->GetTid() == 0)) {
        pic_enc = pic;
        lowest_poc = pic_data->GetPoc();
        found_pic = true;
      }
    }
    if (found_pic) {
      last_added_tid = pic_enc->GetPicData()->GetTid();
      last_added_poc = lowest_poc;
      if (idx < constants::kNumPicsInRefPicLists) {
        rpl->SetRefPic(ref_pic_list, idx++, pic_enc->GetPicData()->GetPoc(),
                       pic_enc->GetPicData(),
                       pic_enc->GetPicData()->GetRecPic());
      }
    }
  }
  return idx;
}

void Encoder::SetNalStats(xvc_enc_nal_unit *nal,
                          std::shared_ptr<PictureEncoder> pic) {
  auto pic_data = pic->GetPicData();
  nal->stats.nal_unit_type =
    static_cast<uint32_t>(pic_data->GetPicType());

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
