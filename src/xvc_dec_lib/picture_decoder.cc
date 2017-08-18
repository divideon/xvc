/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_dec_lib/picture_decoder.h"

#include <cassert>
#include <cstring>
#include <memory>
#include <utility>

#include "xvc_common_lib/deblocking_filter.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/resample.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/segment_header.h"
#include "xvc_dec_lib/cu_decoder.h"
#include "xvc_dec_lib/entropy_decoder.h"

namespace xvc {

PictureDecoder::PictureDecoder(const SimdFunctions &simd,
                               ChromaFormat chroma_format, int width,
                               int height, int bitdepth)
  : simd_(simd),
  pic_data_(std::make_shared<PictureData>(chroma_format, width, height,
                                          bitdepth)),
  rec_pic_(std::make_shared<YuvPicture>(chroma_format, width, height,
                                        bitdepth, true)) {
}

PictureDecoder::PicNalHeader
PictureDecoder::DecodeHeader(BitReader *bit_reader, PicNum *sub_gop_end_poc,
                             PicNum *sub_gop_start_poc, PicNum *sub_gop_length,
                             PicNum max_sub_gop_length,
                             PicNum prev_sub_gop_length, PicNum doc,
                             SegmentNum soc_counter, int num_buffered_nals) {
  // Start by reading the picture header data
  uint32_t header_byte = bit_reader->ReadBits(8);
  NalUnitType nal_unit_type = NalUnitType((header_byte >> 1) & 31);
  int buffer_flag = bit_reader->ReadBits(1);
  SegmentNum soc =  (buffer_flag) ? soc_counter - 1 : soc_counter;
  int tid = bit_reader->ReadBits(3);
  if (tid == 0) {
    PicNum length = max_sub_gop_length;
    if (num_buffered_nals) {
      *sub_gop_length = prev_sub_gop_length;
    } else if (nal_unit_type == NalUnitType::kIntraAccessPicture) {
      *sub_gop_length = 1;
    } else if (length > 0) {
      *sub_gop_length = length;
    } else if (doc > 0) {
      *sub_gop_length = 1;
    }
    *sub_gop_start_poc = *sub_gop_end_poc;
  } else if (max_sub_gop_length > *sub_gop_length) {
    *sub_gop_length = max_sub_gop_length;
  }
  int pic_qp_ = bit_reader->ReadBits(7) - constants::kQpSignalBase;
  bit_reader->SkipBits();

  // Ensure that Sub Gop start is updated to include the current doc.
  if (doc > *sub_gop_end_poc) {
    *sub_gop_start_poc = *sub_gop_end_poc;
  }
  while (doc > *sub_gop_start_poc + *sub_gop_length) {
    *sub_gop_start_poc += *sub_gop_length;
  }
  if (doc > 0 && doc <= *sub_gop_start_poc) {
    doc = *sub_gop_start_poc + 1;
  }

  // The tid in the segment header can differ from the expected tid if:
  //  1. Temporal layers have been removed, or
  //  2. The Sub Gop is incomplete (i.e. at the end of a stream)
  // Figure out the correct doc to use by stepping until the tids are equal.
  while (SegmentHeader::CalcTidFromDoc(doc, *sub_gop_length, *sub_gop_start_poc)
         != tid) {
    doc++;
    if (doc > *sub_gop_end_poc) {
      *sub_gop_start_poc = *sub_gop_end_poc;
    }
  }

  if (tid == 0) {
    *sub_gop_end_poc =
      SegmentHeader::CalcPocFromDoc(doc, *sub_gop_length, *sub_gop_start_poc);
  }

  // Set High-level syntax parameters of the current picture
  PictureDecoder::PicNalHeader header;
  header.nal_unit_type = nal_unit_type;
  header.soc = soc;
  header.poc =
    SegmentHeader::CalcPocFromDoc(doc, *sub_gop_length, *sub_gop_start_poc);
  header.doc = doc;
  header.tid = tid;
  header.pic_qp = pic_qp_;
  header.highest_layer = tid == SegmentHeader::GetMaxTid(*sub_gop_length);
  return header;
}

void PictureDecoder::Init(const SegmentHeader &segment,
                          const PicNalHeader &header,
                          ReferencePictureLists &&ref_pic_list,
                          int64_t user_data) {
  assert(output_status_ == OutputStatus::kHasBeenOutput);
  pic_qp_ = header.pic_qp;
  user_data_ = user_data;
  output_status_ = OutputStatus::kProcessing;
  ref_count = 0;
  pic_data_->SetNalType(header.nal_unit_type);
  pic_data_->SetSoc(header.soc);
  pic_data_->SetPoc(header.poc);
  pic_data_->SetDoc(header.doc);
  pic_data_->SetTid(header.tid);
  pic_data_->SetHighestLayer(header.highest_layer);
  pic_data_->SetAdaptiveQp(segment.adaptive_qp > 0);
  pic_data_->SetDeblock(segment.deblock > 0);
  pic_data_->SetBetaOffset(segment.beta_offset);
  pic_data_->SetTcOffset(segment.tc_offset);
  *pic_data_->GetRefPicLists() = std::move(ref_pic_list);
}

bool PictureDecoder::Decode(const SegmentHeader &segment,
                            BitReader *bit_reader) {
  assert(output_status_ == OutputStatus::kProcessing);
  bool success = true;
  double lambda = 0;
  Qp qp(pic_qp_, pic_data_->GetChromaFormat(), pic_data_->GetBitdepth(),
        lambda);

  pic_data_->Init(segment, qp, true);

  EntropyDecoder entropy_decoder(bit_reader);
  entropy_decoder.Start();
  SyntaxReader syntax_reader(qp, pic_data_->GetPredictionType(),
                             &entropy_decoder);
  std::unique_ptr<CuDecoder> cu_decoder(
    new CuDecoder(simd_, qp, rec_pic_.get(), pic_data_.get()));
  int num_ctus = pic_data_->GetNumberOfCtu();
  for (int rsaddr = 0; rsaddr < num_ctus; rsaddr++) {
    cu_decoder->DecodeCtu(rsaddr, &syntax_reader);
  }
  if (pic_data_->GetDeblock()) {
    DeblockingFilter deblocker(pic_data_.get(), rec_pic_.get(),
                               pic_data_->GetBetaOffset(),
                               pic_data_->GetTcOffset());
    deblocker.DeblockPicture();
  }
  if (!entropy_decoder.DecodeBinTrm()) {
    assert(0);
    success = false;
  }
  entropy_decoder.Finish();
  int pic_tid = pic_data_->GetTid();
  rec_pic_->PadBorder();
  pic_data_->GetRefPicLists()->ZeroOutReferences();
  if (pic_tid == 0 || segment.checksum_mode == Checksum::Mode::kMaxRobust) {
    success &= ValidateChecksum(bit_reader, segment.checksum_mode);
  }
  return success;
}

std::shared_ptr<YuvPicture>
PictureDecoder::GetAlternativeRecPic(ChromaFormat chroma_format, int width,
                                     int height, int bitdepth) const {
  if (alt_rec_pic_)
    return alt_rec_pic_;
  auto alt_rec_pic =
    std::make_shared<YuvPicture>(chroma_format, width, height, bitdepth, true);
  for (int c = 0; c < util::GetNumComponents(chroma_format); c++) {
    YuvComponent comp = YuvComponent(c);
    uint8_t* dst =
      reinterpret_cast<uint8_t*>(alt_rec_pic->GetSamplePtr(comp, 0, 0));
    if (rec_pic_->GetChromaFormat() == ChromaFormat::kMonochrome &&
        comp != YuvComponent::kY) {
      std::memset(dst, 1 << (alt_rec_pic->GetBitdepth() - 1),
                  alt_rec_pic->GetStride(comp) *
                  alt_rec_pic->GetHeight(comp) * sizeof(Sample));
      continue;
    }
    uint8_t* src =
      reinterpret_cast<uint8_t*>(rec_pic_->GetSamplePtr(comp, 0, 0));
    resample::Resample<Sample, Sample>
      (dst, alt_rec_pic->GetWidth(comp), alt_rec_pic->GetHeight(comp),
       alt_rec_pic->GetStride(comp), alt_rec_pic->GetBitdepth(),
       src, rec_pic_->GetWidth(comp), rec_pic_->GetHeight(comp),
       rec_pic_->GetStride(comp), rec_pic_->GetBitdepth());
  }
  alt_rec_pic->PadBorder();
  // TODO(PH) Revise const_cast by making this function a pure getter?
  // In the future alternate pictures should probably be created
  // beforehand in a separate thread as this can be quite time consuming
  const_cast<PictureDecoder*>(this)->alt_rec_pic_ = alt_rec_pic;
  return alt_rec_pic;
}

bool PictureDecoder::ValidateChecksum(BitReader *bit_reader,
                                      Checksum::Mode checksum_mode) {
  size_t checksum_len = bit_reader->ReadByte();
  std::vector<uint8_t> checksum_bytes;
  checksum_bytes.resize(checksum_len);
  bit_reader->ReadBytes(&checksum_bytes[0], checksum_len);

  Checksum::Method checksum_method =
    Restrictions::Get().disable_high_level_default_checksum_method ?
    Checksum::kFallbackMethod : Checksum::kDefaultMethod;

  checksum_.Clear();
  checksum_.HashPicture(*rec_pic_, checksum_method, checksum_mode);
  return checksum_.GetHash() == checksum_bytes;
}

}   // namespace xvc
