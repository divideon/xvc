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

Encoder::Encoder(int internal_bitdepth)
  : segment_header_(new SegmentHeader()),
  simd_(SimdCpu::GetRuntimeCapabilities(), internal_bitdepth),
  encoder_settings_() {
  assert(internal_bitdepth >= 8);
#if XVC_HIGH_BITDEPTH
  assert(internal_bitdepth <= 16);
#else
  assert(internal_bitdepth == 8);
#endif
  segment_header_->codec_identifier = constants::kXvcCodecIdentifier;
  segment_header_->major_version = constants::kXvcMajorVersion;
  segment_header_->minor_version = constants::kXvcMinorVersion;
  segment_header_->internal_bitdepth = internal_bitdepth;
}

int Encoder::Encode(const uint8_t *pic_bytes, xvc_enc_nal_unit **nal_units,
                    bool output_rec, xvc_enc_pic_buffer *rec_pic) {
  nal_units_.clear();

  // Set picture parameters and get bytes for original picture
  auto pic_enc = GetNewPictureEncoder();
  auto pic_data = pic_enc->GetPicData();
  pic_enc->SetOutputStatus(OutputStatus::kHasNotBeenOutput);
  pic_data->SetPoc(poc_);
  if (segment_header_->num_ref_pics == 0) {
    pic_data->SetNalType(NalUnitType::kIntraPicture);
  } else if (Restrictions::Get().disable_inter_bipred) {
    pic_data->SetNalType(NalUnitType::kPredictedPicture);
  } else {
    pic_data->SetNalType(NalUnitType::kBipredictedPicture);
  }
  PicNum doc =
    SegmentHeader::CalcDocFromPoc(poc_, segment_header_->max_sub_gop_length,
                                  sub_gop_start_poc_);
  int tid =
    SegmentHeader::CalcTidFromDoc(doc, segment_header_->max_sub_gop_length,
                                  sub_gop_start_poc_);
  int max_tid = SegmentHeader::GetMaxTid(segment_header_->max_sub_gop_length);
  pic_data->SetDoc(doc);
  pic_data->SetTid(tid);
  pic_data->SetSubGopLength(segment_header_->max_sub_gop_length);
  pic_data->SetHighestLayer(tid == max_tid);
  pic_data->SetAdaptiveQp(segment_header_->adaptive_qp > 0);
  pic_data->SetDeblock(segment_header_->deblock > 0);
  pic_data->SetBetaOffset(segment_header_->beta_offset);
  pic_data->SetTcOffset(segment_header_->tc_offset);

  if (segment_header_->GetOutputWidth() != segment_header_->GetInternalWidth()
      || segment_header_->GetOutputHeight() !=
      segment_header_->GetInternalHeight()) {
    pic_enc->GetOrigPic()->CopyFromWithResampling(
      pic_bytes, input_bitdepth_, segment_header_->GetOutputWidth(),
      segment_header_->GetOutputHeight());
  } else {
    pic_enc->GetOrigPic()->CopyFrom(pic_bytes, input_bitdepth_);
  }

  // Check if it is time to encode a new segment header.
  if ((poc_ % segment_length_) == 0) {
    prev_segment_open_gop_ = curr_segment_open_gop_;
    if (((poc_ + segment_length_) % closed_gop_interval_) == 0) {
      curr_segment_open_gop_ = false;
    } else {
      curr_segment_open_gop_ = true;
    }
    prev_segment_header_ = std::move(segment_header_);
    segment_header_.reset(new SegmentHeader(*prev_segment_header_));
    bit_writer_.Clear();
    if (encoder_settings_.encapsulation_mode != 0) {
      bit_writer_.WriteBits(constants::kEncapsulationCode1, 8);
      bit_writer_.WriteBits(1, 8);
    }
    SegmentHeaderWriter::Write(segment_header_.get(), &bit_writer_, framerate_,
                               curr_segment_open_gop_);
    soc_++;
    segment_header_->soc = soc_;
    xvc_enc_nal_unit nal;
    std::vector<uint8_t> *nal_bytes = bit_writer_.GetBytes();
    nal.bytes = &(*nal_bytes)[0];
    nal.size = nal_bytes->size();
    nal.buffer_flag = 0;
    SetNalStats(*pic_data, &nal);
    nal.stats.nal_unit_type = static_cast<int>(NalUnitType::kSegmentHeader);
    nal_units_.push_back(nal);
    pic_data->SetNalType(NalUnitType::kIntraAccessPicture);
    encode_with_buffer_flag_ = true;
  } else {
    encode_with_buffer_flag_ = false;
  }
  pic_data->SetSoc(soc_);


  if (poc_ == 0) {
    EncodeOnePicture(pic_enc, segment_header_->max_sub_gop_length);
    doc_ = 0;
  }

  // Check if there are enough pictures buffered to encode a new Sub Gop
  if (poc_ == doc_ + segment_header_->max_sub_gop_length) {
    for (PicNum i = 0; i < segment_header_->max_sub_gop_length; i++) {
      // Find next picture to encode by searching for
      // the one that has doc = this->doc_ + 1.
      for (auto &pic : pic_encoders_) {
        if (pic->GetPicData()->GetDoc() == doc_ + 1) {
          EncodeOnePicture(pic, segment_header_->max_sub_gop_length);
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

  // Increase encoder poc counter by one for each call to Encode.
  // poc_ is initialized to 0.
  poc_++;

  if (rec_pic) {
    rec_pic->pic = nullptr;
    rec_pic->size = 0;
  }
  // If enough pictures have been encoded, the reconstructed picture
  // with lowest poc can be output.
  if (poc_ >= segment_header_->max_sub_gop_length) {
    ReconstructOnePicture(output_rec, rec_pic);
  }
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
  if (poc_ > 0) {
    poc_--;
  }
  // Check if there are pictures left to encode.
  if (doc_ < poc_) {
    encode_with_buffer_flag_ = false;
    PicNum pics_to_encode = poc_ - doc_;
    PicNum num_encoded = 0;
    while (num_encoded < pics_to_encode) {
      // Find next picture to encode by searching for
      // the one that has doc = this->doc_ + 1.
      bool found = false;
      for (auto &pic : pic_encoders_) {
        if (pic->GetPicData()->GetDoc() == doc_ + 1) {
          EncodeOnePicture(pic, segment_header_->max_sub_gop_length);
          found = true;
          num_encoded++;
        }
      }
      if (!found) {
        doc_++;
      }
    }
  }

  // Increase poc by one for each call to Flush.
  poc_++;

  if (rec_pic) {
    rec_pic->pic = nullptr;
    rec_pic->size = 0;
  }
  // Check if reconstruction should be performed.
  ReconstructOnePicture(output_rec, rec_pic);
  if (nal_units_.size() > 0) {
    *nal_units = &nal_units_[0];
  }
  return static_cast<int>(nal_units_.size());
}

void Encoder::SetEncoderSettings(const EncoderSettings &settings) {
  assert(poc_ == 0);
  encoder_settings_ = settings;
  segment_header_->num_ref_pics = settings.default_num_ref_pics;
  segment_header_->max_binary_split_depth = settings.max_binary_split_depth;
  segment_header_->chroma_qp_offset_table = settings.chroma_qp_offset_table;
  segment_header_->chroma_qp_offset_u = settings.chroma_qp_offset_u;
  segment_header_->chroma_qp_offset_v = settings.chroma_qp_offset_v;
  segment_header_->adaptive_qp = settings.adaptive_qp;
  // Load restriction flags
  Restrictions restrictions = Restrictions();
  restrictions.EnableRestrictedMode(settings.restricted_mode);
  // Apply speed settings that correspond directly to restriction flags
  if (settings.fast_transform_size_64) {
    restrictions.disable_ext_transform_size_64 = 1;
  }
  if (settings.fast_transform_select) {
    restrictions.disable_ext2_transform_select = 1;
  }
  if (settings.fast_inter_local_illumination_comp) {
    restrictions.disable_ext2_inter_local_illumination_comp = 1;
  }
  if (settings.fast_inter_adaptive_fullpel_mv) {
    restrictions.disable_ext2_inter_adaptive_fullpel_mv = 1;
  }
  Restrictions::GetRW() = std::move(restrictions);
}

void Encoder::EncodeOnePicture(std::shared_ptr<PictureEncoder> pic,
                               PicNum sub_gop_length) {
  // Check if current picture is a tail picture.
  // This means that it will be sent before the key picture of the
  // next segment and then be buffered in the decoder to be decoded after
  // the key picture.
  int buffer_flag = encode_with_buffer_flag_ &&
    pic->GetPicData()->GetNalType() != NalUnitType::kIntraAccessPicture;
  SegmentHeader &segment_header =
    !buffer_flag ? *segment_header_ : *prev_segment_header_;

  ReferenceListSorter<PictureEncoder>
    ref_list_sorter(segment_header, prev_segment_open_gop_);
  ref_list_sorter.Prepare(pic->GetPicData()->GetPoc(),
                          pic->GetPicData()->GetTid(),
                          pic->GetPicData()->IsIntraPic(),
                          pic_encoders_, pic->GetPicData()->GetRefPicLists());

  // Bitstream reference valid until next picture is coded
  std::vector<uint8_t> *pic_bytes =
    pic->Encode(segment_header, segment_qp_, sub_gop_length, buffer_flag,
                flat_lambda_, encoder_settings_);

  // When a picture has been encoded, the picture data is put into
  // the xvc_enc_nal_unit struct to be delivered through the API.
  xvc_enc_nal_unit nal;
  nal.bytes = &(*pic_bytes)[0];
  nal.size = pic_bytes->size();
  nal.buffer_flag = buffer_flag;
  SetNalStats(*pic->GetPicData(), &nal);
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
    if (pic->GetOutputStatus() == OutputStatus::kHasNotBeenOutput &&
        pd->GetPoc() < lowest_poc) {
      pic_enc = pic;
      lowest_poc = pd->GetPoc();
    }
  }
  if (!pic_enc) {
    return;
  }
  pic_enc->SetOutputStatus(OutputStatus::kHasBeenOutput);
  auto rec_pic_out = pic_enc->GetRecPic();

  // Only perform reconstruction if it is requested and a picture was found.
  if (output_rec && rec_pic_out && rec_pic) {
    rec_pic_out->CopyToSameBitdepth(&output_pic_bytes_);
    rec_pic->size = output_pic_bytes_.size();
    rec_pic->pic = &output_pic_bytes_[0];
  }
}

std::shared_ptr<PictureEncoder> Encoder::GetNewPictureEncoder() {
  // Allocate a new PictureEncoder if the number of buffered pictures
  // is lower than the maximum that will be used.
  if (pic_encoders_.size() < pic_buffering_num_) {
    auto pic =
      std::make_shared<PictureEncoder>(simd_,
                                       segment_header_->GetInternalPicFormat());
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
    if (pic->GetOutputStatus() == OutputStatus::kHasBeenOutput &&
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

void Encoder::SetNalStats(const PictureData &pic_data, xvc_enc_nal_unit *nal) {
  nal->stats.nal_unit_type =
    static_cast<uint32_t>(pic_data.GetNalType());

  // Expose the 32 least significant bits of poc and doc.
  nal->stats.poc = static_cast<uint32_t>(pic_data.GetPoc());
  nal->stats.doc = static_cast<uint32_t>(pic_data.GetDoc());
  nal->stats.soc = static_cast<uint32_t>(pic_data.GetSoc());
  nal->stats.tid = pic_data.GetTid();
  if (pic_data.GetPicQp()) {
    nal->stats.qp = pic_data.GetPicQp()->GetQpRaw(YuvComponent::kY);
  } else {
    nal->stats.qp = constants::kMaxAllowedQp + 1;
  }

  // Expose the first reference pictures in L0 and L1.
  const ReferencePictureLists* rpl = pic_data.GetRefPicLists();
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
