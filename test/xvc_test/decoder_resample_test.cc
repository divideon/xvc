/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include <algorithm>
#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_test/test_helper.h"
#include "xvc_test/yuv_helper.h"

namespace {

class DecoderResampleTest : public ::testing::Test,
  public ::xvc_test::EncoderHelper, public ::xvc_test::DecoderHelper {
protected:
  void SetUp() override {
    EncoderHelper::Init();
    DecoderHelper::Init();
    encoder_->SetSubGopLength(1);
    encoder_->SetSegmentLength(1);
  }

  void EncodeDecodeResolution(int size_enc1, int size_enc2, int size_dec,
                                   bool explicit_output_resolution) {
    const int bitdepth = 8;
    encoder_ = CreateEncoder(size_enc1, size_enc1, bitdepth, qp1);
    EncodeFirstFrame(kSample1);
    encoder_ = CreateEncoder(size_enc2, size_enc2, bitdepth, qp2);
    EncodeFirstFrame(kSample2);
    ASSERT_EQ(4, encoded_nal_units_.size());

    if (explicit_output_resolution) {
      decoder_->SetOutputWidth(size_dec);
      decoder_->SetOutputHeight(size_dec);
    }

    DecodeSegmentHeaderSuccess(GetNextNalToDecode());
    auto dec_pic = DecodeAndFlush(GetNextNalToDecode());
    EXPECT_EQ(size_dec, dec_pic->stats.width);
    EXPECT_EQ(size_dec, dec_pic->stats.height);
    EXPECT_TRUE(VerifyDecodedLumaEquals(*dec_pic, kSample1));

    DecodeSegmentHeaderSuccess(GetNextNalToDecode());
    dec_pic = DecodeAndFlush(GetNextNalToDecode());
    EXPECT_EQ(size_dec, dec_pic->stats.width);
    EXPECT_EQ(size_dec, dec_pic->stats.height);
    EXPECT_TRUE(VerifyDecodedLumaEquals(*dec_pic, kSample2));
  }

  void EncodeDecodeBitdepth(int bitdepth_enc1, int bitdepth_enc2,
                            int bitdepth_dec, bool explicit_output) {
    const int size_enc1 = 16;
    const int size_enc2 = 16;
    const int size_dec = 16;
    encoder_ = CreateEncoder(size_enc1, size_enc1, bitdepth_enc1, qp1);
    EncodeFirstFrame(kSample1, bitdepth_enc1);
    encoder_ = CreateEncoder(size_enc2, size_enc2, bitdepth_enc2, qp2);
    EncodeFirstFrame(kSample2, bitdepth_enc2);
    ASSERT_EQ(4, encoded_nal_units_.size());

    if (explicit_output) {
      decoder_->SetOutputBitdepth(bitdepth_dec);
    }

    DecodeSegmentHeaderSuccess(GetNextNalToDecode());
    auto dec_pic = DecodeAndFlush(GetNextNalToDecode());
    EXPECT_EQ(size_dec, dec_pic->stats.width);
    EXPECT_EQ(size_dec, dec_pic->stats.height);
    EXPECT_EQ(bitdepth_dec, dec_pic->stats.bitdepth);
    xvc::Sample expected1 = kSample1;
    if (bitdepth_enc1 > bitdepth_dec) {
      int shift = (bitdepth_enc1 - bitdepth_dec);
      expected1 = (expected1 + (1 << (shift - 1))) >> shift;
    } else {
      expected1 <<= (bitdepth_dec - bitdepth_enc1);
    }
    EXPECT_TRUE(VerifyDecodedLumaEquals(*dec_pic, expected1));

    DecodeSegmentHeaderSuccess(GetNextNalToDecode());
    dec_pic = DecodeAndFlush(GetNextNalToDecode());
    EXPECT_EQ(size_dec, dec_pic->stats.width);
    EXPECT_EQ(size_dec, dec_pic->stats.height);
    EXPECT_EQ(bitdepth_dec, dec_pic->stats.bitdepth);
    xvc::Sample expected2 = kSample2;
    if (bitdepth_enc2 > bitdepth_dec) {
      int shift = bitdepth_enc2 - bitdepth_dec;
      expected2 = (expected2 + (1 << (shift - 1))) >> shift;
    } else {
      expected2 <<= (bitdepth_dec - bitdepth_enc2);
    }
    EXPECT_TRUE(VerifyDecodedLumaEquals(*dec_pic, expected2));
  }

  ::testing::AssertionResult
    VerifyDecodedLumaEquals(const xvc_decoded_picture &dec_pic,
                            xvc::Sample expected_sample) {
    return xvc_test::TestYuvPic::AllSampleEqualTo(
      dec_pic.stats.width, dec_pic.stats.height, dec_pic.stats.bitdepth,
      dec_pic.bytes, dec_pic.size, expected_sample);
  }

  // Decoding exactly the same value depends on quantization
  const int qp1 = 18;
  const int qp2 = 23;
  static const xvc::Sample kSample1 = 43;
  static const xvc::Sample kSample2 = 96;
};

TEST_F(DecoderResampleTest, DownscalingImplicit) {
  EncodeDecodeResolution(16, 24, 16, false);
}

TEST_F(DecoderResampleTest, DownscalingExplicit) {
  EncodeDecodeResolution(16, 24, 16, true);
}

TEST_F(DecoderResampleTest, UpscalingImplicit) {
  EncodeDecodeResolution(24, 16, 24, false);
}

TEST_F(DecoderResampleTest, UpscalingExplicit) {
  EncodeDecodeResolution(24, 16, 24, true);
}

#if XVC_HIGH_BITDEPTH
TEST_F(DecoderResampleTest, BitdepthUpConversionLowToHigh) {
  EncodeDecodeBitdepth(8, 10, 8, false);
}

TEST_F(DecoderResampleTest, BitdepthUpConversionLowToHighExplicit) {
  EncodeDecodeBitdepth(8, 10, 8, true);
}

TEST_F(DecoderResampleTest, BitdepthUpConversionHighToHigh) {
  EncodeDecodeBitdepth(10, 12, 10, false);
}

TEST_F(DecoderResampleTest, BitdepthUpConversionHighToHighExplicit) {
  EncodeDecodeBitdepth(10, 12, 10, true);
}

TEST_F(DecoderResampleTest, BitdepthUpConversionExplicit) {
  EncodeDecodeBitdepth(8, 10, 12, true);
}

TEST_F(DecoderResampleTest, BitdepthDownConversionHighToLow) {
  EncodeDecodeBitdepth(10, 8, 10, false);
}

TEST_F(DecoderResampleTest, BitdepthDownConversionHighToLowExplicit) {
  EncodeDecodeBitdepth(10, 8, 10, true);
}

TEST_F(DecoderResampleTest, BitdepthDownConversionHighToHigh) {
  EncodeDecodeBitdepth(12, 10, 12, false);
}

TEST_F(DecoderResampleTest, BitdepthDownConversionHighToHighExplicit) {
  EncodeDecodeBitdepth(12, 10, 12, true);
}

TEST_F(DecoderResampleTest, BitdepthDownConversionExplicit) {
  EncodeDecodeBitdepth(12, 10, 8, true);
}
#endif

}   // namespace
