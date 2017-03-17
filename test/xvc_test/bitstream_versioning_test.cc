/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "googletest/include/gtest/gtest.h"

#include "xvc_test/test_helper.h"

namespace {

class BitstreamVersioningTest : public ::xvc_test::EncoderDecoderHelper {
public:
  bool Run(int major_version, int minor_version) {
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
};

TEST_F(BitstreamVersioningTest, RecvSameVersion) {
  EXPECT_TRUE(Run(xvc::constants::kXvcMajorVersion,
                  xvc::constants::kXvcMinorVersion));
  ASSERT_EQ(::xvc::Decoder::State::kSegmentHeaderDecoded, decoder_->GetState());
  DecodeFirstPictureSuccess();
  EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
}

TEST_F(BitstreamVersioningTest, RecvLargerMajorVersion) {
  EXPECT_FALSE(Run(xvc::constants::kXvcMajorVersion + 1,
                  xvc::constants::kXvcMinorVersion));
  ASSERT_EQ(::xvc::Decoder::State::kDecoderVersionTooLow, decoder_->GetState());
  DecodeFirstPictureFailed();
  EXPECT_EQ(::xvc::Decoder::State::kDecoderVersionTooLow, decoder_->GetState());
}

TEST_F(BitstreamVersioningTest, RecvLowerMajorVersion) {
  EXPECT_TRUE(Run(xvc::constants::kXvcMajorVersion - 1,
                  xvc::constants::kXvcMinorVersion));
  ASSERT_EQ(::xvc::Decoder::State::kSegmentHeaderDecoded, decoder_->GetState());
  DecodeFirstPictureSuccess();
  EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
}

TEST_F(BitstreamVersioningTest, RecvLargerMinorVersion) {
  EXPECT_TRUE(Run(xvc::constants::kXvcMajorVersion,
                  xvc::constants::kXvcMinorVersion + 1));
  ASSERT_EQ(::xvc::Decoder::State::kSegmentHeaderDecoded, decoder_->GetState());
  DecodeFirstPictureSuccess();
  EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
}

}   // namespace
