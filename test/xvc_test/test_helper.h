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

class DecoderHelper {
public:
  virtual void SetUp() {
    decoder_ = new ::xvc::Decoder();
  }
  virtual void TearDown() {
    delete decoder_;
  }

  void DecodeSegmentHeader(const std::vector<uint8_t> &nal) {
    EXPECT_TRUE(decoder_->DecodeNal(&nal[0], nal.size()));
    EXPECT_EQ(0, decoder_->GetNumDecodedPics());
    EXPECT_FALSE(decoder_->GetDecodedPicture(&decoded_picture_));
    EXPECT_EQ(nullptr, decoded_picture_.bytes);
    EXPECT_EQ(0, decoded_picture_.size);
  }

  void DecodePictureSuccess(const std::vector<uint8_t> &nal) {
    EXPECT_TRUE(decoder_->DecodeNal(&nal[0], nal.size()));
    EXPECT_EQ(1, decoder_->GetNumDecodedPics());
    EXPECT_TRUE(decoder_->GetDecodedPicture(&decoded_picture_));
  }

  void DecodePictureFailed(const std::vector<uint8_t> &nal) {
    EXPECT_FALSE(decoder_->DecodeNal(&nal[0], nal.size()));
    EXPECT_EQ(0, decoder_->GetNumDecodedPics());
    EXPECT_FALSE(decoder_->GetDecodedPicture(&decoded_picture_));
    EXPECT_EQ(0, decoded_picture_.size);
  }

  ::testing::AssertionResult
    VerifyDecodedLumaEquals(int width, int height,
                            xvc::Sample expected_sample) {
    size_t sample_size = decoded_picture_.stats.bitdepth == 8 ?
      sizeof(uint8_t) : sizeof(uint16_t);
    EXPECT_EQ(sample_size * width * height * 3 / 2, decoded_picture_.size);
    size_t luma_samples = width * height;
    if (sample_size == sizeof(uint8_t)) {
      uint8_t *bytes = reinterpret_cast<uint8_t*>(decoded_picture_.bytes);
      uint8_t expected = static_cast<uint8_t>(expected_sample);
      auto it =
        std::find_if(bytes, bytes + luma_samples,
                     [expected](const uint8_t &b) { return b != expected; });
      if (it == bytes + luma_samples) {
        return ::testing::AssertionSuccess();
      }
      return ::testing::AssertionFailure() << "found: " <<
        static_cast<int>(*it) << " expected: " << static_cast<int>(expected);
    } else {
      uint16_t *bytes = reinterpret_cast<uint16_t*>(decoded_picture_.bytes);
      uint16_t expected = static_cast<uint16_t>(expected_sample);
      auto it =
        std::find_if(bytes, bytes + luma_samples,
                     [expected](const uint16_t &b) { return b != expected; });
      if (it == bytes + luma_samples) {
        return ::testing::AssertionSuccess();
      }
      return ::testing::AssertionFailure() << "found: " <<
        static_cast<int>(*it) << " expected: " << static_cast<int>(expected);
    }
  }

  ::xvc::Decoder *decoder_;
  xvc_decoded_picture decoded_picture_;
};

class EncoderHelper {
protected:
  virtual void SetUp() {
    encoder_ = CreateEncoder(0, 0, 8, qp_);
  }
  virtual void TearDown() {
  }

  std::unique_ptr<xvc::Encoder>
    CreateEncoder(int width, int height, int internal_bitdepth, int qp) {
    std::unique_ptr<xvc::Encoder> encoder(new xvc::Encoder());
    encoder->SetResolution(width, height);
    encoder->SetChromaFormat(XVC_ENC_CHROMA_FORMAT_420);
    encoder->SetInputBitdepth(8);
    encoder->SetInternalBitdepth(internal_bitdepth);
    encoder->SetSubGopLength(1);
    encoder->SetFramerate(30);
    encoder->SetChecksumMode(1);
    encoder->SetDeblock(1);
    encoder->SetQp(qp);
    xvc::SpeedSettings speed_settings;
    speed_settings.Initialize(xvc::SpeedMode::kSlow);
    encoder->SetSpeedSettings(std::move(speed_settings));
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

  void EncodeFirstFrame(xvc::Sample fill_sample = 0x7F, int bitdepth = 8) {
    std::vector<uint8_t> pic_bytes = CreateSampleBuffer(fill_sample, bitdepth);
    return EncodeFirstFrame(pic_bytes, bitdepth);
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
        std::vector<uint8_t>(nal_units[i].bytes,
                             nal_units[i].bytes + nal_units[i].size));
    }
  }

  static const int qp_ = 32;
  std::unique_ptr<xvc::Encoder> encoder_;
  std::vector<std::vector<uint8_t>> encoded_nal_units_;
};

class EncoderDecoderHelper
  : public ::testing::Test, public EncoderHelper, public DecoderHelper {
protected:
  void SetUp() override {
    EncoderHelper::SetUp();
    DecoderHelper::SetUp();
  }
  void TearDown() override {
    EncoderHelper::TearDown();
    DecoderHelper::TearDown();
  }

  void DecodeFirstPictureSuccess() {
    ASSERT_EQ(2, encoded_nal_units_.size());
    DecodePictureSuccess(encoded_nal_units_[1]);
  }
  void DecodeFirstPictureFailed() {
    ASSERT_EQ(2, encoded_nal_units_.size());
    DecodePictureFailed(encoded_nal_units_[1]);
  }
};

}   // namespace xvc_test

#endif  // XVC_TEST_TEST_HELPER_H_
