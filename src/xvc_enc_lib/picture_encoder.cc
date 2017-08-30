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

#include "xvc_enc_lib/picture_encoder.h"

#include <cassert>
#include <cstring>
#include <memory>
#include <utility>

#include "xvc_common_lib/deblocking_filter.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"
#include "xvc_enc_lib/cu_encoder.h"
#include "xvc_enc_lib/entropy_encoder.h"

namespace xvc {

PictureEncoder::PictureEncoder(const SimdFunctions &simd,
                               ChromaFormat chroma_format, int width,
                               int height, int bitdepth)
  : simd_(simd),
  orig_pic_(std::make_shared<YuvPicture>(chroma_format, width, height,
                                         bitdepth, false)),
  pic_data_(std::make_shared<PictureData>(chroma_format, width, height,
                                          bitdepth)),
  rec_pic_(std::make_shared<YuvPicture>(chroma_format, width, height,
                                        bitdepth, true)) {
}

std::vector<uint8_t>*
PictureEncoder::Encode(const SegmentHeader &segment, int segment_qp,
                       PicNum sub_gop_length, int buffer_flag,
                       bool flat_lambda,
                       const EncoderSettings &encoder_settings) {
  int lambda_sub_gop_length =
    !flat_lambda ? static_cast<int>(segment.max_sub_gop_length) : 1;
  int lambda_max_tid = SegmentHeader::GetMaxTid(lambda_sub_gop_length);
  int lambda_pic_tid = !flat_lambda ? pic_data_->GetTid() : 0;
  int pic_qp = DerivePictureQp(*pic_data_, segment_qp);
  double lambda =
    Qp::CalculateLambda(pic_qp, pic_data_->GetPredictionType(),
                        lambda_sub_gop_length, lambda_pic_tid, lambda_max_tid,
                        encoder_settings.smooth_lambda_scaling);
  int scaled_qp = Qp::GetQpFromLambda(pic_data_->GetBitdepth(), lambda);
  Qp base_qp(scaled_qp, pic_data_->GetChromaFormat(), pic_data_->GetBitdepth(),
             lambda, encoder_settings.chroma_qp_offset_table,
             encoder_settings.chroma_qp_offset_u,
             encoder_settings.chroma_qp_offset_v);

  pic_data_->Init(segment, base_qp, encoder_settings.adaptive_qp > 0);

  bit_writer_.Clear();
  if (encoder_settings.encapsulation_mode != 0) {
    bit_writer_.WriteBits(constants::kEncapsulationCode1, 8);
    bit_writer_.WriteBits(1, 8);
  }
  WriteHeader(*pic_data_, sub_gop_length, buffer_flag, &bit_writer_);

  EntropyEncoder entropy_encoder(&bit_writer_);
  entropy_encoder.Start();
  SyntaxWriter writer(base_qp, pic_data_->GetPredictionType(),
                      &entropy_encoder);
  std::unique_ptr<CuEncoder>
    cu_encoder(new CuEncoder(simd_, *orig_pic_, rec_pic_.get(), pic_data_.get(),
                             encoder_settings));
  int num_ctus = pic_data_->GetNumberOfCtu();
  for (int rsaddr = 0; rsaddr < num_ctus; rsaddr++) {
    cu_encoder->EncodeCtu(rsaddr, &writer);
  }
  if (pic_data_->GetDeblock()) {
    DeblockingFilter deblocker(pic_data_.get(), rec_pic_.get(),
                               pic_data_->GetBetaOffset(),
                               pic_data_->GetTcOffset());
    deblocker.DeblockPicture();
  }
  entropy_encoder.EncodeBinTrm(1);
  entropy_encoder.Finish();

  int pic_tid = pic_data_->GetTid();
  if (pic_tid == 0 || !pic_data_->IsHighestLayer()) {
    rec_pic_->PadBorder();
  }
  pic_data_->GetRefPicLists()->ZeroOutReferences();
  if (pic_tid == 0 || segment.checksum_mode == Checksum::Mode::kMaxRobust) {
    WriteChecksum(&bit_writer_, segment.checksum_mode);
  }
  return bit_writer_.GetBytes();
}

std::shared_ptr<YuvPicture>
PictureEncoder::GetAlternativeRecPic(ChromaFormat chroma_format, int width,
                                     int height, int bitdepth) const {
  assert(0);
  return std::shared_ptr<YuvPicture>();
}

void PictureEncoder::WriteHeader(const PictureData &pic_data,
                                 PicNum sub_gop_length, int buffer_flag,
                                 BitWriter *bit_writer) {
  bit_writer->WriteBits(0, 2);  // nal_rfe
  bit_writer->WriteBits(static_cast<uint8_t>(pic_data.GetNalType()), 5);
  bit_writer->WriteBits(1, 1);  // nal_rfl
  bit_writer->WriteBits(buffer_flag, 1);
  assert(pic_data.GetTid() >= 0);
  bit_writer->WriteBits(pic_data.GetTid(), 3);
  int pic_qp = pic_data.GetPicQp()->GetQpRaw(YuvComponent::kY);
  assert(pic_qp + constants::kQpSignalBase < (1 << 7));
  bit_writer->WriteBits(pic_qp + constants::kQpSignalBase, 7);
  bit_writer->PadZeroBits();
}

void PictureEncoder::WriteChecksum(BitWriter *bit_writer,
                                   Checksum::Mode checksum_mode) {
  checksum_.Clear();
  Checksum::Method checksum_method =
    Restrictions::Get().disable_high_level_default_checksum_method ?
    Checksum::kFallbackMethod : Checksum::kDefaultMethod;
  checksum_.HashPicture(*rec_pic_, checksum_method, checksum_mode);
  std::vector<uint8_t> hash = checksum_.GetHash();
  assert(hash.size() < UINT8_MAX);
  bit_writer->WriteByte(static_cast<uint8_t>(hash.size()));
  bit_writer->WriteBytes(&hash[0], hash.size());
}

int PictureEncoder::DerivePictureQp(const PictureData &pic_data,
                                    int segment_qp) const {
  int pic_qp = segment_qp;
  if (pic_data.GetPredictionType() != PicturePredictionType::kIntra) {
    pic_qp = segment_qp + pic_data.GetTid() + 1;
  }
  return util::Clip3(pic_qp, constants::kMinAllowedQp,
                     constants::kMaxAllowedQp);
}

}   // namespace xvc
