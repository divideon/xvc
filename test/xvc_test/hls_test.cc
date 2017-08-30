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

#include "googletest/include/gtest/gtest.h"

#include "xvc_test/test_helper.h"

namespace {

class HlsTest : public ::testing::Test,
  public ::xvc_test::EncoderHelper, public ::xvc_test::DecoderHelper {
public:
  void SetUp() override {
    EncoderHelper::Init();
    DecoderHelper::Init();
  }

  void EncodeWithVersion(int major_version, int minor_version) {
    encoder_->SetResolution(0, 0);
    std::vector<uint8_t> pic_bytes;
    EncodeFirstFrame(pic_bytes, 8);
    // Rewrite version directly in bitstream
    uint8_t *segment_header = &encoded_nal_units_[0][0];
    segment_header++;   // nal_header (1 byte)
    segment_header[3] = (major_version >> 8) & 0xFF;
    segment_header[4] = (major_version >> 0) & 0xFF;
    segment_header[5] = (minor_version >> 8) & 0xFF;
    segment_header[6] = (minor_version >> 0) & 0xFF;
  }

  void EncodeWithRfeValue(int nal_rfe_value) {
    encoder_->SetResolution(0, 0);
    std::vector<uint8_t> pic_bytes;
    EncodeFirstFrame(pic_bytes, 8);
    // Rewrite value directly in bitstream
    uint8_t *nal_unit_header = &encoded_nal_units_[0][0];
    nal_unit_header[0] |= (nal_rfe_value & 3) << 6;
  }
};

TEST_F(HlsTest, RecvSameVersion) {
  EncodeWithVersion(xvc::constants::kXvcMajorVersion,
                    xvc::constants::kXvcMinorVersion);
  DecodeSegmentHeaderSuccess(GetNextNalToDecode());
  ASSERT_EQ(::xvc::Decoder::State::kSegmentHeaderDecoded, decoder_->GetState());
  DecodePictureSuccess(GetNextNalToDecode());
  EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
}

TEST_F(HlsTest, RecvLargerMajorVersion) {
  EncodeWithVersion(xvc::constants::kXvcMajorVersion + 1,
                    xvc::constants::kXvcMinorVersion);
  DecodeSegmentHeaderFailed(GetNextNalToDecode());
  ASSERT_EQ(::xvc::Decoder::State::kDecoderVersionTooLow, decoder_->GetState());
  DecodePictureFailed(GetNextNalToDecode());
  EXPECT_EQ(::xvc::Decoder::State::kDecoderVersionTooLow, decoder_->GetState());
}

TEST_F(HlsTest, RecvLowerMajorVersion) {
  EncodeWithVersion(xvc::constants::kXvcMajorVersion - 1,
                    xvc::constants::kXvcMinorVersion);
  DecodeSegmentHeaderSuccess(GetNextNalToDecode());
  ASSERT_EQ(::xvc::Decoder::State::kSegmentHeaderDecoded, decoder_->GetState());
  DecodePictureSuccess(GetNextNalToDecode());
  EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
}

TEST_F(HlsTest, RecvLargerMinorVersion) {
  EncodeWithVersion(xvc::constants::kXvcMajorVersion,
                    xvc::constants::kXvcMinorVersion + 1);
  DecodeSegmentHeaderSuccess(GetNextNalToDecode());
  ASSERT_EQ(::xvc::Decoder::State::kSegmentHeaderDecoded, decoder_->GetState());
  DecodePictureSuccess(GetNextNalToDecode());
  EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
}

TEST_F(HlsTest, RecvRfeZero) {
  EncodeWithRfeValue(0);
  DecodeSegmentHeaderSuccess(GetNextNalToDecode());
  ASSERT_EQ(::xvc::Decoder::State::kSegmentHeaderDecoded, decoder_->GetState());
  DecodePictureSuccess(GetNextNalToDecode());
  EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
}

TEST_F(HlsTest, RecvRfeOne) {
  EncodeWithRfeValue(1);
  DecodeSegmentHeaderFailed(GetNextNalToDecode());
  ASSERT_EQ(::xvc::Decoder::State::kNoSegmentHeader, decoder_->GetState());
  DecodePictureFailed(GetNextNalToDecode());
  EXPECT_EQ(::xvc::Decoder::State::kNoSegmentHeader, decoder_->GetState());
}

}   // namespace
