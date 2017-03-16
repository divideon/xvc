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
#include "xvc_common_lib/segment_header.h"
#include "xvc_dec_lib/cu_decoder.h"
#include "xvc_dec_lib/entropy_decoder.h"

namespace xvc {

PictureDecoder::PictureDecoder(ChromaFormat chroma_format, int width,
                               int height, int bitdepth)
  : pic_data_(std::make_shared<PictureData>(chroma_format, width, height,
                                            bitdepth)),
  checksum_(Checksum::kDefaultMethod),
  first_peek_(1) {
}

void PictureDecoder::DecodeHeader(BitReader *bit_reader,
                                  PicNum *sub_gop_end_poc,
                                  PicNum *sub_gop_start_poc,
                                  PicNum *sub_gop_length,
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
  pic_data_->SetTid(tid);
  if (tid == 0) {
    PicNum length = bit_reader->ReadBits(7);
    if (length > 0 && num_buffered_nals == 0) {
      *sub_gop_length = length;
      PicNum poc = *sub_gop_end_poc + *sub_gop_length;
      pic_data_->SetPoc(poc);
      pic_data_->SetDoc(SegmentHeader::CalcDocFromPoc(poc, *sub_gop_length,
                                                      *sub_gop_end_poc));
    } else if (soc > 0) {
      *sub_gop_length = num_buffered_nals + 1;
      PicNum poc = *sub_gop_end_poc + *sub_gop_length;
      pic_data_->SetPoc(poc);
      pic_data_->SetDoc(SegmentHeader::CalcDocFromPoc(poc, *sub_gop_length,
                                                      *sub_gop_end_poc));
    }
    *sub_gop_start_poc = *sub_gop_end_poc;
    *sub_gop_end_poc = pic_data_->GetPoc();
  }
  pic_qp_ = bit_reader->ReadBits(7) - constants::kQpSignalBase;
  bit_reader->SkipBits();
}

bool PictureDecoder::Decode(BitReader *bit_reader, PicNum sub_gop_length) {
  double lambda = 0;
  QP qp(pic_qp_, pic_data_->GetChromaFormat(),  pic_data_->GetBitdepth(),
        lambda);
  pic_data_->Init(qp);

  EntropyDecoder entropy_decoder(bit_reader);
  entropy_decoder.Start();
  SyntaxReader syntax_reader(qp, pic_data_->GetPredictionType(),
                             &entropy_decoder);
  std::unique_ptr<CuDecoder> cu_decoder(
    new CuDecoder(qp, pic_data_->GetRecPic().get(), pic_data_.get()));
  int num_ctus = pic_data_->GetNumberOfCtu();
  for (int rsaddr = 0; rsaddr < num_ctus; rsaddr++) {
    cu_decoder->DecodeCtu(rsaddr, &syntax_reader);
  }
  if (pic_data_->GetDeblock()) {
    DeblockingFilter deblocker(pic_data_.get(), pic_data_->GetBetaOffset(),
                               pic_data_->GetTcOffset());
    deblocker.DeblockPicture();
  }
  if (!entropy_decoder.DecodeBinTrm()) {
    assert(0);
  }
  entropy_decoder.Finish();

  pic_data_->GetRecPic()->PadBorder();
  pic_data_->GetRefPicLists()->ZeroOutReferences();
  return ValidateChecksum(bit_reader);
}

bool PictureDecoder::ValidateChecksum(BitReader *bit_reader) {
  size_t checksum_len = bit_reader->ReadByte();
  std::vector<uint8_t> checksum_bytes;
  checksum_bytes.resize(checksum_len);
  bit_reader->ReadBytes(&checksum_bytes[0], checksum_len);

  checksum_.Clear();
  checksum_.HashPicture(*(pic_data_->GetRecPic()));
  return checksum_ == Checksum(checksum_.GetMethod(), checksum_bytes);
}

}   // namespace xvc
