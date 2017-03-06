/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_dec_lib/decoder.h"

#include <cassert>
#include <iostream>
#include <limits>

#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"
#include "xvc_dec_lib/segment_header_reader.h"

namespace xvc {

bool Decoder::DecodeNal(const uint8_t *nal_unit, size_t nal_unit_size) {
  // First check the Nal Unit Type
  BitReader bit_reader(nal_unit, nal_unit_size);
  uint8_t header = bit_reader.ReadByte();
  NalUnitType nal_unit_type = NalUnitType((header >> 1) & 31);

  // Perform different decoding processes depending on the Nal Unit Type
  if (nal_unit_type == NalUnitType::kSegmentHeader) {
    // If there are old nal units buffered that are not tail pictures,
    // they are discarded before decoding the new segment.
    if (nal_buffer_.size() > num_tail_pics_) {
      num_pics_in_buffer_ -= static_cast<uint32_t>(nal_buffer_.size());
      nal_buffer_.clear();
      num_tail_pics_ = 0;
    }
    prev_segment_header_ = curr_segment_header_;
    state_ = SegmentHeaderReader::Read(&curr_segment_header_, &bit_reader,
                                       soc_);
    if (state_ != State::kSegmentHeaderDecoded) {
      return false;
    }
    soc_++;
    sub_gop_length_ = curr_segment_header_.max_sub_gop_length;
    if (sub_gop_length_ + 1 > sliding_window_length_) {
      sliding_window_length_ = sub_gop_length_ + 1;
    }
    pic_buffering_num_ =
      sliding_window_length_ + constants::kMaxNumLongTermPics;

    if (output_width_ == 0) {
      output_width_ = curr_segment_header_.pic_width;
    }
    if (output_height_ == 0) {
      output_height_ = curr_segment_header_.pic_height;
    }
    if (output_chroma_format_ == ChromaFormat::kUndefinedChromaFormat) {
      output_chroma_format_ = curr_segment_header_.chroma_format;
    }
    if (output_bitdepth_ == 0) {
      output_bitdepth_ = curr_segment_header_.internal_bitdepth;
    }
    max_tid_ = PictureData::GetMaxTid(decoder_ticks_,
                                      curr_segment_header_.bitstream_ticks,
                                      sub_gop_length_);

  } else if (state_ == State::kNoSegmentHeader ||
             state_ == State::kDecoderVersionTooLow ||
             state_ == State::kBitstreamBitdepthTooHigh) {
    // Do not decode anything else than a segment header if
    // no segment header has been decoded or if the xvc version
    // of the decoder is identified to be too low or if the
    // bitstream bitdepth is too high.
    return false;
  } else if (nal_unit_type >= NalUnitType::kIntraPicture &&
             nal_unit_type <= NalUnitType::kReservedPictureType10) {
    // All picture types are decoded using the same process.
    // First, the buffer flag is checked to see if the picture
    // should be decoded or buffered.
    int buffer_flag = bit_reader.ReadBit();

    int tid = bit_reader.ReadBits(3);
    if (tid > max_tid_) {
      // Ignore (drop) picture if it belongs to a temporal layer that
      // should not be decoded.
      return true;
    }
    num_pics_in_buffer_++;
    bit_reader.Rewind(4);

    if (buffer_flag == 0 && num_tail_pics_ > 0) {
      nal_buffer_.emplace_front(&nal_unit[0], &nal_unit[0] + nal_unit_size);
    } else {
      nal_buffer_.emplace_back(&nal_unit[0], &nal_unit[0] + nal_unit_size);
    }
    if (buffer_flag) {
      num_tail_pics_++;
    } else {
      while (nal_buffer_.size() > 0 && num_pics_in_buffer_ - nal_buffer_.size()
             + 1 < pic_buffering_num_) {
        auto nal = nal_buffer_.front();
        DecodeOneBufferedNal(nal);
        nal_buffer_.pop_front();
      }
    }
  }
  return true;
}

void Decoder::DecodeOneBufferedNal(const std::vector<uint8_t> &nal) {
  BitReader pic_bit_reader(&nal[0], nal.size());
  // Configure picture parameters before decoding
  pic_bit_reader.ReadBits(8);
  int buffer_flag = pic_bit_reader.ReadBits(1);
  pic_bit_reader.Rewind(9);
  std::shared_ptr<PictureDecoder> pic_dec;
  SegmentHeader *segment_header;
  if (buffer_flag) {
    segment_header = &prev_segment_header_;
    num_tail_pics_--;
  } else {
    segment_header = &curr_segment_header_;
  }
  pic_dec = GetNewPictureDecoder(segment_header->chroma_format,
                                 segment_header->pic_width,
                                 segment_header->pic_height,
                                 segment_header->internal_bitdepth);
  auto pic_data = pic_dec->GetPicData();
  pic_data->SetOutputStatus(OutputStatus::kHasNotBeenOutput);
  pic_data->SetDoc(doc_);
  pic_data->CalcPocFromDoc(sub_gop_length_, sub_gop_start_poc_);

  pic_data->SetDeblock(segment_header->deblock > 0);
  pic_data->SetBetaOffset(segment_header->beta_offset);
  pic_data->SetTcOffset(segment_header->tc_offset);

  // Decode the picture header
  pic_dec->DecodeHeader(&pic_bit_reader, &sub_gop_end_poc_,
                        &sub_gop_start_poc_, &sub_gop_length_, soc_,
                        num_tail_pics_);

  PrepareRefPicLists(pic_dec);

  // Decode the picture.
  if (!pic_dec->Decode(&pic_bit_reader, segment_header->base_qp,
                       sub_gop_length_)) {
    num_corrupted_pics_++;
  }
  if (num_corrupted_pics_ == 1) {
    std::cerr << "Detected checksum mismatch!" << std::endl;
  }
  state_ = State::kPicDecoded;
  // Increase global decode order counter.
  doc_ = pic_data->GetDoc() + 1;
}

void Decoder::DecodeAllBufferedNals() {
  for (auto &nal : nal_buffer_) {
    DecodeOneBufferedNal(nal);
  }
  nal_buffer_.clear();
}

void Decoder::FlushBufferedTailPics() {
  // Return if there are still Nal Units waiting
  // to be decoded.
  if (nal_buffer_.size() > num_tail_pics_) {
    return;
  }
  // Preparing to start a new segment
  soc_++;
  prev_segment_header_ = curr_segment_header_;
  // Check if there are buffered nal units.
  if (nal_buffer_.size() > 0) {
    if (curr_segment_header_.open_gop) {
      // Throw away buffered Nal Units.
      num_pics_in_buffer_ -= static_cast<uint32_t>(nal_buffer_.size());
      nal_buffer_.clear();
    } else {
      // Step over the missing key picture and then decode the buffered
      // Nal Units.
      doc_++;
      sub_gop_start_poc_ = sub_gop_end_poc_;
      sub_gop_end_poc_ += sub_gop_length_;
      DecodeAllBufferedNals();
    }
  }
}

void Decoder::GetDecodedPicture(xvc_decoded_picture *output_pic) {
  // Find the picture with lowest Poc that has not been output.
  std::shared_ptr<YuvPicture> decoded_pic;
  std::shared_ptr<PictureDecoder> pic_dec;
  PicNum lowest_poc = std::numeric_limits<PicNum>::max();
  for (auto &pic : pic_decoders_) {
    auto pd = pic->GetPicData();
    if ((pd->GetOutputStatus() == OutputStatus::kHasNotBeenOutput) &&
      (pd->GetPoc() < lowest_poc)) {
      pic_dec = pic;
      lowest_poc = pd->GetPoc();
    }
  }
  if (!pic_dec) {
    output_pic->size = 0;
    output_pic->bytes = nullptr;
    return;
  }
  pic_dec->GetPicData()->SetOutputStatus(OutputStatus::kHasBeenOutput);
  SetOutputStats(pic_dec, output_pic);
  decoded_pic = pic_dec->GetPicData()->GetRecPic();

  decoded_pic->CopyTo(&output_pic_bytes_, output_width_, output_height_,
                      output_chroma_format_, output_bitdepth_);
  output_pic->size = output_pic_bytes_.size();
  output_pic->bytes = output_pic_bytes_.empty() ? nullptr :
    reinterpret_cast<char *>(&output_pic_bytes_[0]);
  // Decrease counter for how many decoded pictures are buffered.
  num_pics_in_buffer_--;
  if (nal_buffer_.size() > num_tail_pics_ &&
      num_pics_in_buffer_ - nal_buffer_.size() < pic_buffering_num_) {
    auto nal = nal_buffer_.front();
    DecodeOneBufferedNal(nal);
    nal_buffer_.pop_front();
  }
}

std::shared_ptr<PictureDecoder> Decoder::GetNewPictureDecoder(
  ChromaFormat chroma_format, int width, int height, int bitdepth) {
  // Allocate a new PictureDecoder if the number of buffered pictures
  // is lower than the maximum that will be used.
  if (pic_decoders_.size() < pic_buffering_num_) {
    auto pic =
      std::make_shared<PictureDecoder>(chroma_format, width, height, bitdepth);
    pic_decoders_.push_back(pic);
    return pic;
  }

  // Reuse a PictureDecoder if the number of buffered pictures
  // is equal to the maximum that will be used.
  // Pick any that has been output and has tid higher than 0.
  // If no pictures with tid higher than 0 is available, reuse the
  // picture with the lowest poc.
  PicNum lowest_poc = std::numeric_limits<PicNum>::max();
  auto it = pic_decoders_.begin();
  auto pic_it = it;
  for (; it != pic_decoders_.end(); ++it) {
    auto pic = *it;
    auto pic_data = pic->GetPicData();
    if ((pic_data->GetOutputStatus() == OutputStatus::kHasBeenOutput) &&
        pic_data->GetTid() > 0) {
      pic_it = it;
      break;
    } else if (pic_data->GetPoc() < lowest_poc) {
      lowest_poc = pic_data->GetPoc();
      pic_it = it;
    }
  }
  // Replace the PictureDecoder if the picture format has changed.
  auto pic_data = (*pic_it)->GetPicData();
  if (width != pic_data->GetPictureWidth(YuvComponent::kY) ||
      height != pic_data->GetPictureHeight(YuvComponent::kY) ||
      chroma_format != pic_data->GetChromaFormat() ||
      bitdepth != pic_data->GetBitdepth()) {
    *pic_it =
      std::make_shared<PictureDecoder>(chroma_format, width, height, bitdepth);
  }
  assert(*pic_it);
  return *pic_it;
}

void Decoder::PrepareRefPicLists(std::shared_ptr<PictureDecoder> curr_pic) {
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

int Decoder::FillRefPicsLowerPoc(int start_idx,
                                 std::shared_ptr<PictureDecoder> curr_pic,
                                 RefPicList ref_pic_list) {
  int idx = start_idx;
  PicNum curr_poc = curr_pic->GetPicData()->GetPoc();
  SegmentNum curr_soc = curr_pic->GetPicData()->GetSoc();
  PicNum last_added_poc = curr_poc;
  std::shared_ptr<PictureDecoder> pic_dec;
  ReferencePictureLists* rpl = curr_pic->GetPicData()->GetRefPicLists();
  int last_added_tid = curr_pic->GetPicData()->GetTid();
  bool found_pic = true;
  while (found_pic) {
    PicNum highest_poc_plus1 = 0;
    found_pic = false;
    for (auto &pic : pic_decoders_) {
      auto pic_data = pic->GetPicData();
      if (pic_data->GetSoc() == curr_soc &&
          pic_data->GetPoc() < last_added_poc &&
          pic_data->GetPoc() + 1 > highest_poc_plus1 &&
          (pic_data->GetTid() < last_added_tid ||
           pic_data->GetTid() == 0)) {
        pic_dec = pic;
        highest_poc_plus1 = pic_data->GetPoc() + 1;
        found_pic = true;
      }
    }
    if (found_pic) {
      last_added_tid = pic_dec->GetPicData()->GetTid();
      last_added_poc = highest_poc_plus1 - 1;
      if (idx < constants::kNumPicsInRefPicLists) {
        rpl->SetRefPic(ref_pic_list, idx++, pic_dec->GetPicData()->GetPoc(),
                       pic_dec->GetPicData(),
                       pic_dec->GetPicData()->GetRecPic());
      }
    }
  }
  return idx;
}

int Decoder::FillRefPicsHigherPoc(int start_idx,
                                  std::shared_ptr<PictureDecoder> curr_pic,
                                  RefPicList ref_pic_list) {
  int idx = start_idx;
  PicNum curr_poc = curr_pic->GetPicData()->GetPoc();
  SegmentNum curr_soc = curr_pic->GetPicData()->GetSoc();
  PicNum last_added_poc = curr_poc;
  std::shared_ptr<PictureDecoder> pic_dec;
  ReferencePictureLists* rpl = curr_pic->GetPicData()->GetRefPicLists();
  int last_added_tid = curr_pic->GetPicData()->GetTid();
  bool found_pic = true;
  while (found_pic) {
    PicNum lowest_poc = std::numeric_limits<PicNum>::max();
    found_pic = false;
    for (auto &pic : pic_decoders_) {
      auto pic_data = pic->GetPicData();
      if ((pic_data->GetSoc() == curr_soc ||
        (pic_data->GetSoc() == static_cast<SegmentNum>(curr_soc + 1) &&
         prev_segment_header_.open_gop)) &&
          pic_data->GetPoc() > last_added_poc &&
          pic_data->GetPoc() < lowest_poc &&
          (pic_data->GetTid() < last_added_tid ||
           pic_data->GetTid() == 0)) {
        pic_dec = pic;
        lowest_poc = pic_data->GetPoc();
        found_pic = true;
      }
    }
    if (found_pic) {
      last_added_tid = pic_dec->GetPicData()->GetTid();
      last_added_poc = lowest_poc;
      if (idx < constants::kNumPicsInRefPicLists) {
        auto ref_data = pic_dec->GetPicData();
        auto curr_data = curr_pic->GetPicData();
        YuvComponent c = YuvComponent::kY;
        if (curr_data->GetSoc() != ref_data->GetSoc() &&
          (curr_data->GetPictureWidth(c) != ref_data->GetPictureWidth(c) ||
           curr_data->GetPictureHeight(c) != ref_data->GetPictureHeight(c) ||
           curr_data->GetChromaFormat() != ref_data->GetChromaFormat() ||
           curr_data->GetBitdepth() != ref_data->GetBitdepth())) {
          rpl->SetRefPic(ref_pic_list, idx++, pic_dec->GetPicData()->GetPoc(),
                         pic_dec->GetPicData(),
                         pic_dec->GetPicData()->GetAlternativeRecPic(
                           curr_data->GetChromaFormat(),
                           curr_data->GetPictureWidth(c),
                           curr_data->GetPictureHeight(c),
                           curr_data->GetBitdepth()));
        } else {
          rpl->SetRefPic(ref_pic_list, idx++, pic_dec->GetPicData()->GetPoc(),
                         pic_dec->GetPicData(),
                         pic_dec->GetPicData()->GetRecPic());
        }
      }
    }
  }
  return idx;
}


void Decoder::SetOutputStats(std::shared_ptr<PictureDecoder> pic_dec,
                             xvc_decoded_picture *output_pic) {
  auto pic_data = pic_dec->GetPicData();
  auto decoded_pic = pic_data->GetRecPic();
  output_pic->stats.height = decoded_pic->GetHeight(YuvComponent(0));
  output_pic->stats.width = decoded_pic->GetWidth(YuvComponent(0));
  output_pic->stats.bitdepth = output_bitdepth_;
  output_pic->stats.bitstream_bitdepth = decoded_pic->GetBitdepth();
  output_pic->stats.chroma_format = getChromaFormatApiStyle();
  output_pic->stats.framerate =
    PictureData::GetFramerate(max_tid_, curr_segment_header_.bitstream_ticks,
                              sliding_window_length_ - 1);
  output_pic->stats.bitstream_framerate =
    PictureData::GetFramerate(0, curr_segment_header_.bitstream_ticks, 1);
  output_pic->stats.nal_unit_type =
    static_cast<uint32_t>(pic_data->GetPicType());

  // Expose the 32 least significant bits of poc and doc.
  output_pic->stats.poc = static_cast<uint32_t>(pic_data->GetPoc());
  output_pic->stats.doc = static_cast<uint32_t>(pic_data->GetDoc());
  output_pic->stats.soc = static_cast<uint32_t>(pic_data->GetSoc());
  output_pic->stats.tid = pic_data->GetTid();
  output_pic->stats.qp = pic_data->GetPicQp()->GetQpRaw(YuvComponent::kY);

  // Expose the first five reference pictures in L0 and L1.
  ReferencePictureLists* rpl = pic_data->GetRefPicLists();
  int length = sizeof(output_pic->stats.l0) / sizeof(output_pic->stats.l0[0]);
  for (int i = 0; i < length; i++) {
    if (i < rpl->GetNumRefPics(RefPicList::kL0)) {
      output_pic->stats.l0[i] =
        static_cast<int32_t>(rpl->GetRefPoc(RefPicList::kL0, i));
    } else {
      output_pic->stats.l0[i] = -1;
    }
    if (i < rpl->GetNumRefPics(RefPicList::kL1)) {
      output_pic->stats.l1[i] =
        static_cast<int32_t>(rpl->GetRefPoc(RefPicList::kL1, i));
    } else {
      output_pic->stats.l1[i] = -1;
    }
  }
}

}   // namespace xvc
