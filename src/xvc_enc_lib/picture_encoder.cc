/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/picture_encoder.h"

#include <cassert>
#include <cstring>
#include <memory>
#include <utility>

#include "xvc_common_lib/deblocking_filter.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_enc_lib/cu_encoder.h"
#include "xvc_enc_lib/entropy_encoder.h"

namespace xvc {

PictureEncoder::PictureEncoder(ChromaFormat chroma_format, int width,
                               int height, int bitdepth)
  : checksum_(Checksum::kDefaultMethod),
  orig_pic_(std::make_shared<YuvPicture>(chroma_format, width, height,
                                         bitdepth, false)),
  pic_data_(std::make_shared<PictureData>(chroma_format, width, height,
                                          bitdepth)) {
}

std::vector<uint8_t>&
PictureEncoder::Encode(int base_qp, PicNum sub_gop_length, int buffer_flag,
                       bool flat_lambda) {
  int lambda_sub_gop_length =
    !flat_lambda ? static_cast<int>(sub_gop_length) : 1;
  int lambda_pic_tid = !flat_lambda ? pic_data_->GetTid() : 0;
  QP qp(base_qp, pic_data_->GetChromaFormat(), pic_data_->GetPredictionType(),
        pic_data_->GetBitdepth(), lambda_sub_gop_length, lambda_pic_tid);
  EntropyEncoder entropy_encoder(&bit_writer_);
  SyntaxWriter writer(qp, pic_data_->GetPredictionType(), &entropy_encoder);
  bit_writer_.Clear();
  bit_writer_.WriteBits(static_cast<uint8_t>(pic_data_->GetPicType()) << 1,
                        8);  // Nal Unit header
  bit_writer_.WriteBits(buffer_flag, 1);
  bit_writer_.WriteBits(pic_data_->GetTid(), 3);
  if (pic_data_->GetTid() == 0) {
    if (pic_data_->GetPoc() == 0) {
      bit_writer_.WriteBits(0, 7);
    } else {
      bit_writer_.WriteBits(static_cast<uint32_t>(sub_gop_length), 7);
    }
  }
  bit_writer_.PadZeroBits();

  pic_data_->Init(qp);
  entropy_encoder.Start();
  std::unique_ptr<CuEncoder> cu_encoder(
    new CuEncoder(qp, *orig_pic_, pic_data_->GetRecPic().get(),
                  pic_data_.get()));
  int num_ctus = pic_data_->GetNumberOfCtu();
  for (int rsaddr = 0; rsaddr < num_ctus; rsaddr++) {
    cu_encoder->EncodeCtu(rsaddr, &writer);
  }
  if (pic_data_->GetDeblock()) {
    DeblockingFilter deblocker(pic_data_.get(), pic_data_->GetBetaOffset(),
                               pic_data_->GetTcOffset());
    deblocker.DeblockPicture();
  }

  entropy_encoder.EncodeBinTrm(1);
  entropy_encoder.Finish();
  WriteChecksum(&bit_writer_);
  pic_data_->GetRecPic()->PadBorder();
  pic_data_->GetRefPicLists()->ZeroOutReferences();
  return bit_writer_.GetBytes();
}

void PictureEncoder::WriteChecksum(BitWriter *bit_writer) {
  checksum_.Clear();
  checksum_.HashPicture(*(pic_data_->GetRecPic()));
  std::vector<uint8_t> hash = checksum_.GetHash();
  assert(hash.size() < UINT8_MAX);
  bit_writer->WriteByte(static_cast<uint8_t>(hash.size()));
  bit_writer->WriteBytes(&hash[0], hash.size());
}

}   // namespace xvc
