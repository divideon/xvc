/******************************************************************************
* Copyright (C) 2018, Divideon.
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
* This library is also available under a commercial license.
* Please visit https://xvc.io/license/ for more information.
******************************************************************************/

#include "googletest/include/gtest/gtest.h"

#include "xvc_test/decoder_helper.h"
#include "xvc_test/encoder_helper.h"

namespace {

class HlsTest : public ::testing::Test,
  public ::xvc_test::EncoderHelper, public ::xvc_test::DecoderHelper {
public:
  void SetUp() override {
    EncoderHelper::Init();
    DecoderHelper::Init();
  }

  void EncodeWithVersion(int major_version, int minor_version) {
    const xvc::SegmentHeader *segment = encoder_->GetCurrentSegment();
    // Forcing const cast since this behavior is typically not needed/exposed
    const_cast<xvc::SegmentHeader*>(segment)->major_version = major_version;
    const_cast<xvc::SegmentHeader*>(segment)->minor_version = minor_version;
    encoder_->SetResolution(0, 0);
    std::vector<uint8_t> pic_bytes;
    auto nals = EncodeOneFrame(pic_bytes, 8);
    ASSERT_EQ(2, nals.size());
  }

  void EncodeWithRfeValue(int nal_rfe_value) {
    encoder_->SetResolution(0, 0);
    std::vector<uint8_t> pic_bytes;
    auto nals = EncodeOneFrame(pic_bytes, 8);
    ASSERT_EQ(2, nals.size());
    // Rewrite value directly in bitstream
    uint8_t *nal_unit_header = &encoded_nal_units_[0][0];
    nal_unit_header[0] |= (nal_rfe_value & 1) << 6;
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
  // Note that sample data in bitstream will still use current major version,
  // hence this test only works if resolution is (0, 0)
  EncodeWithVersion(xvc::constants::kXvcMajorVersion - 1,
                    xvc::constants::kXvcMinorVersion);
  DecodeSegmentHeaderSuccess(GetNextNalToDecode());
  ASSERT_EQ(::xvc::Decoder::State::kSegmentHeaderDecoded, decoder_->GetState());
  DecodePictureSuccess(GetNextNalToDecode());
  EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
}

TEST_F(HlsTest, RecvMajorVersionZero) {
  EncodeWithVersion(0,
                    xvc::constants::kXvcMinorVersion);
  DecodeSegmentHeaderFailed(GetNextNalToDecode());
  ASSERT_EQ(::xvc::Decoder::State::kBitstreamVersionTooLow,
            decoder_->GetState());
  DecodePictureFailed(GetNextNalToDecode());
  EXPECT_EQ(::xvc::Decoder::State::kBitstreamVersionTooLow,
            decoder_->GetState());
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
