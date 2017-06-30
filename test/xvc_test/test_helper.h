/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_TEST_TEST_HELPER_H_
#define XVC_TEST_TEST_HELPER_H_

#include <memory>
#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_dec_lib/decoder.h"
#include "xvc_enc_lib/encoder.h"

namespace xvc_test {

using NalUnit = std::vector<uint8_t>;

class EncoderHelper {
protected:
  void Init() {
    encoder_ = CreateEncoder(0, 0, 8, 32);
  }

  std::unique_ptr<xvc::Encoder>
    CreateEncoder(int width, int height, int internal_bitdepth, int qp) {
    std::unique_ptr<xvc::Encoder> encoder(new xvc::Encoder());
    xvc::EncoderSettings encoder_settings;
    encoder_settings.Initialize(xvc::SpeedMode::kSlow);
    encoder_settings.Tune(xvc::TuneMode::kPsnr);
    encoder->SetEncoderSettings(std::move(encoder_settings));
    encoder->SetResolution(width, height);
    encoder->SetChromaFormat(xvc::ChromaFormat::k420);
    encoder->SetInputBitdepth(8);
    encoder->SetInternalBitdepth(internal_bitdepth);
    encoder->SetSegmentLength(64);
    encoder->SetSubGopLength(1);
    encoder->SetFramerate(30);
    encoder->SetChecksumMode(xvc::Checksum::Mode::kMaxRobust);
    encoder->SetDeblock(1);
    encoder->SetQp(qp);
    return encoder;
  }

  std::vector<uint8_t> CreateSampleBuffer(xvc::Sample fill_sample,
                                          int bitdepth) {
    const xvc::SegmentHeader *segment = encoder_->GetCurrentSegment();
    size_t sample_size = bitdepth == 8 ? sizeof(uint8_t) : sizeof(uint16_t);
    size_t plane_samples =
      segment->pic_width * segment->pic_height * sample_size;
    std::vector<uint8_t> pic_bytes(plane_samples *
                                   ::xvc::constants::kMaxYuvComponents);
    if (bitdepth == 8) {
      std::fill(pic_bytes.begin(), pic_bytes.end(),
                static_cast<uint8_t>(fill_sample));
    } else {
      std::fill(reinterpret_cast<uint16_t*>(&pic_bytes[0]),
                reinterpret_cast<uint16_t*>(&pic_bytes[pic_bytes.size() - 2]),
                static_cast<uint16_t>(fill_sample));
    }
    return pic_bytes;
  }

  void EncodeFirstFrame(const std::vector<uint8_t> &pic_bytes, int bitdepth) {
    const uint8_t *in_pic = pic_bytes.empty() ? nullptr : &pic_bytes[0];
    xvc_enc_nal_unit *nal_units = nullptr;
    xvc_enc_pic_buffer rec;
    encoder_->SetInputBitdepth(bitdepth);
    int num_nals = encoder_->Encode(in_pic, &nal_units, false, &rec);
    ASSERT_EQ(2, num_nals);
    EXPECT_EQ(static_cast<uint32_t>(xvc::NalUnitType::kSegmentHeader),
              nal_units[0].stats.nal_unit_type);
    EXPECT_EQ(static_cast<uint32_t>(xvc::NalUnitType::kIntraAccessPicture),
              nal_units[1].stats.nal_unit_type);
    for (int i = 0; i < 2; i++) {
      encoded_nal_units_.push_back(
        NalUnit(nal_units[i].bytes, nal_units[i].bytes + nal_units[i].size));
    }
  }

  void EncodeOneFrame(const std::vector<uint8_t> &pic_bytes, int bitdepth) {
    const uint8_t *in_pic = pic_bytes.empty() ? nullptr : &pic_bytes[0];
    xvc_enc_nal_unit *nal_units = nullptr;
    encoder_->SetInputBitdepth(bitdepth);
    int num_nals = encoder_->Encode(in_pic, &nal_units, false, nullptr);
    for (int i = 0; i < num_nals; i++) {
      encoded_nal_units_.push_back(
        NalUnit(nal_units[i].bytes, nal_units[i].bytes + nal_units[i].size));
    }
  }

  void EncoderFlush() {
    xvc_enc_nal_unit *nal_units = nullptr;
    int num_nals = encoder_->Flush(&nal_units, false, nullptr);
    for (int i = 0; i < num_nals; i++) {
      encoded_nal_units_.push_back(
        NalUnit(nal_units[i].bytes, nal_units[i].bytes + nal_units[i].size));
    }
  }

  void ResetBitstreamPosition() {
    decoded_nals_units = 0;
  }

  NalUnit& GetNextNalToDecode() {
    return encoded_nal_units_[decoded_nals_units++];
  }

  std::unique_ptr<xvc::Encoder> encoder_;
  std::vector<NalUnit> encoded_nal_units_;
  int decoded_nals_units = 0;
};


class DecoderHelper {
public:
  void Init(int num_threads = 0) {
    decoder_ = std::unique_ptr<xvc::Decoder>(new ::xvc::Decoder(num_threads));
  }

  void DecodeSegmentHeaderSuccess(const NalUnit &nal) {
    xvc::PicNum before_num_decoded_pics = decoder_->GetNumDecodedPics();
    EXPECT_TRUE(decoder_->DecodeNal(&nal[0], nal.size()));
    EXPECT_EQ(::xvc::Decoder::State::kSegmentHeaderDecoded,
              decoder_->GetState());
    EXPECT_EQ(before_num_decoded_pics, decoder_->GetNumDecodedPics());
    EXPECT_EQ(0, decoder_->GetNumCorruptedPics());
    EXPECT_FALSE(decoder_->GetDecodedPicture(&last_decoded_picture_));
    EXPECT_EQ(nullptr, last_decoded_picture_.bytes);
    EXPECT_EQ(0, last_decoded_picture_.size);
  }

  void DecodeSegmentHeaderFailed(const NalUnit &nal) {
    xvc::PicNum before_num_decoded_pics = decoder_->GetNumDecodedPics();
    EXPECT_FALSE(decoder_->DecodeNal(&nal[0], nal.size()));
    EXPECT_EQ(before_num_decoded_pics, decoder_->GetNumDecodedPics());
    EXPECT_FALSE(decoder_->GetDecodedPicture(&last_decoded_picture_));
    EXPECT_EQ(nullptr, last_decoded_picture_.bytes);
    EXPECT_EQ(0, last_decoded_picture_.size);
  }

  bool DecodePictureSuccess(const NalUnit &nal) {
    xvc::PicNum before_num_decoded_pics = decoder_->GetNumDecodedPics();
    EXPECT_TRUE(decoder_->DecodeNal(&nal[0], nal.size()));
    EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
    EXPECT_EQ(before_num_decoded_pics + 1, decoder_->GetNumDecodedPics());
    EXPECT_EQ(0, decoder_->GetNumCorruptedPics());
    return decoder_->GetDecodedPicture(&last_decoded_picture_);
  }

  void DecodePictureFailed(const NalUnit &nal) {
    xvc::PicNum before_num_decoded_pics = decoder_->GetNumDecodedPics();
    EXPECT_FALSE(decoder_->DecodeNal(&nal[0], nal.size()));
    EXPECT_EQ(before_num_decoded_pics, decoder_->GetNumDecodedPics());
    EXPECT_FALSE(decoder_->GetDecodedPicture(&last_decoded_picture_));
    EXPECT_EQ(0, last_decoded_picture_.size);
  }

  bool DecoderFlush() {
    decoder_->FlushBufferedTailPics();
    return decoder_->GetDecodedPicture(&last_decoded_picture_);
  }

  xvc_decoded_picture* DecodeAndFlush(const NalUnit &nal) {
    if (DecodePictureSuccess(nal)) {
      return &last_decoded_picture_;
    }
    if (DecoderFlush()) {
      return &last_decoded_picture_;
    }
    return nullptr;
  }

  std::unique_ptr<xvc::Decoder> decoder_;
  xvc_decoded_picture last_decoded_picture_;
};

}   // namespace xvc_test

#endif  // XVC_TEST_TEST_HELPER_H_
