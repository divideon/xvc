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

PictureDecoder::PictureDecoder(ChromaFormat chroma_format, int width,
                               int height, int bitdepth)
  : pic_data_(std::make_shared<PictureData>(chroma_format, width, height,
                                            bitdepth)),
  rec_pic_(std::make_shared<YuvPicture>(chroma_format, width, height,
                                        bitdepth, true)),
  checksum_(Restrictions::Get().disable_high_level_default_checksum_method ?
            Checksum::kFallbackMethod : Checksum::kDefaultMethod),
  first_peek_(1) {
}

void PictureDecoder::DecodeHeader(BitReader *bit_reader,
                                  PicNum *sub_gop_end_poc,
                                  PicNum *sub_gop_start_poc,
                                  PicNum *sub_gop_length,
                                  PicNum max_sub_gop_length,
                                  PicNum prev_sub_gop_length,
                                  PicNum doc,
                                  SegmentNum soc,
                                  int num_buffered_nals) {
  // Start by reading the picture header data
  uint32_t header = bit_reader->ReadBits(8);
  NalUnitType nal_unit_type = NalUnitType((header >> 1) & 31);
  pic_data_->SetNalType(nal_unit_type);
  int buffer_flag = bit_reader->ReadBits(1);
  if (buffer_flag) {
    pic_data_->SetSoc(soc - 1);
  } else {
    pic_data_->SetSoc(soc);
  }
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
  pic_qp_ = bit_reader->ReadBits(7) - constants::kQpSignalBase;
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
  int max_tid = SegmentHeader::GetMaxTid(*sub_gop_length);

  // Set High-level syntax parameters of the current picture
  pic_data_->SetOutputStatus(OutputStatus::kHasNotBeenOutput);
  pic_data_->SetDoc(doc);
  pic_data_->SetPoc(
    SegmentHeader::CalcPocFromDoc(doc, *sub_gop_length, *sub_gop_start_poc));
  pic_data_->SetTid(tid);
  pic_data_->SetHighestLayer(tid == max_tid);
}

bool PictureDecoder::Decode(const SegmentHeader &segment,
                            BitReader *bit_reader) {
  double lambda = 0;
  Qp qp(pic_qp_, pic_data_->GetChromaFormat(), pic_data_->GetBitdepth(),
        lambda);
  pic_data_->Init(segment, qp, true);

  EntropyDecoder entropy_decoder(bit_reader);
  entropy_decoder.Start();
  SyntaxReader syntax_reader(qp, pic_data_->GetPredictionType(),
                             &entropy_decoder);
  std::unique_ptr<CuDecoder> cu_decoder(
    new CuDecoder(qp, rec_pic_.get(), pic_data_.get()));
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
  }
  entropy_decoder.Finish();
  int pic_tid = pic_data_->GetTid();
  rec_pic_->PadBorder();
  pic_data_->GetRefPicLists()->ZeroOutReferences();
  if (pic_tid == 0 || segment.checksum_mode == Checksum::Mode::kMaxRobust) {
    return ValidateChecksum(bit_reader, segment.checksum_mode);
  } else {
    return true;
  }
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

  checksum_.Clear();
  checksum_.HashPicture(*rec_pic_, checksum_mode);
  return checksum_ == Checksum(checksum_.GetMethod(), checksum_bytes);
}

}   // namespace xvc
