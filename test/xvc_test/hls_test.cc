/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "googletest/include/gtest/gtest.h"

#include "xvc_test/test_helper.h"

namespace {

class HlsTest : public ::xvc_test::EncoderDecoderHelper {
public:
  bool ChangeVersion(int major_version, int minor_version) {
    encoder_->SetResolution(0, 0);
    EncodeFirstFrame();
    // Rewrite version directly in bitstream
    auto nal = encoded_nal_units_[0];
    uint8_t *segment_header = &nal[0];
    segment_header++;   // nal_header (1 byte)
    segment_header[3] = (major_version >> 8) & 0xFF;
    segment_header[4] = (major_version >> 0) & 0xFF;
    segment_header[5] = (minor_version >> 8) & 0xFF;
    segment_header[6] = (minor_version >> 0) & 0xFF;
    return decoder_->DecodeNal(&nal[0], nal.size());
  }

  bool ChangeNalRfeValue(int nal_rfe_value) {
    encoder_->SetResolution(0, 0);
    EncodeFirstFrame();
    // Rewrite version directly in bitstream
    auto nal = encoded_nal_units_[0];
    uint8_t *nal_unit_header = &nal[0];
    nal_unit_header[0] |= (nal_rfe_value & 3) << 6;
    return decoder_->DecodeNal(&nal[0], nal.size());
  }
};

TEST_F(HlsTest, RecvSameVersion) {
  EXPECT_TRUE(ChangeVersion(xvc::constants::kXvcMajorVersion,
                            xvc::constants::kXvcMinorVersion));
  ASSERT_EQ(::xvc::Decoder::State::kSegmentHeaderDecoded, decoder_->GetState());
  DecodeFirstPictureSuccess();
  EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
}

TEST_F(HlsTest, RecvLargerMajorVersion) {
  EXPECT_FALSE(ChangeVersion(xvc::constants::kXvcMajorVersion + 1,
                             xvc::constants::kXvcMinorVersion));
  ASSERT_EQ(::xvc::Decoder::State::kDecoderVersionTooLow, decoder_->GetState());
  DecodeFirstPictureFailed();
  EXPECT_EQ(::xvc::Decoder::State::kDecoderVersionTooLow, decoder_->GetState());
}

TEST_F(HlsTest, RecvLowerMajorVersion) {
  EXPECT_TRUE(ChangeVersion(xvc::constants::kXvcMajorVersion - 1,
                            xvc::constants::kXvcMinorVersion));
  ASSERT_EQ(::xvc::Decoder::State::kSegmentHeaderDecoded, decoder_->GetState());
  DecodeFirstPictureSuccess();
  EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
}

TEST_F(HlsTest, RecvLargerMinorVersion) {
  EXPECT_TRUE(ChangeVersion(xvc::constants::kXvcMajorVersion,
                            xvc::constants::kXvcMinorVersion + 1));
  ASSERT_EQ(::xvc::Decoder::State::kSegmentHeaderDecoded, decoder_->GetState());
  DecodeFirstPictureSuccess();
  EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
}

TEST_F(HlsTest, RecvRfeZero) {
  EXPECT_TRUE(ChangeNalRfeValue(0));
  ASSERT_EQ(::xvc::Decoder::State::kSegmentHeaderDecoded, decoder_->GetState());
  DecodeFirstPictureSuccess();
  EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
}

TEST_F(HlsTest, RecvRfeOne) {
  EXPECT_FALSE(ChangeNalRfeValue(1));
  ASSERT_EQ(::xvc::Decoder::State::kNoSegmentHeader, decoder_->GetState());
  DecodeFirstPictureFailed();
  EXPECT_EQ(::xvc::Decoder::State::kNoSegmentHeader, decoder_->GetState());
}

}   // namespace
