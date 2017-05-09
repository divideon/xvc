/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_test/test_helper.h"
#include "xvc_test/yuv_helper.h"

namespace {

class EncodeDecodeTest : public ::testing::TestWithParam<int>,
  public ::xvc_test::EncoderHelper, public ::xvc_test::DecoderHelper {
protected:
  static constexpr int kQp = 27;
  static constexpr double kPsnrThreshold = 28.0;
  static constexpr int kFramesEncoded = 8;
  static constexpr int kSegmentLength = kFramesEncoded * 3;

  void SetUp() override {
    EncoderHelper::Init();
    DecoderHelper::Init();
    encoder_->SetInternalBitdepth(GetParam());
    encoder_->SetSubGopLength(kFramesEncoded);
    encoder_->SetSegmentLength(kSegmentLength);
    encoder_->SetQp(kQp);
  }

  void TearDown() override {
    for (int poc = 0; poc < static_cast<int>(verified_.size()); poc++) {
      ASSERT_TRUE(verified_[poc]) << "Picture poc " << poc;
    }
  }

  void Encode(int width, int height, int frames) {
    encoder_->SetResolution(width, height);
    for (int i = 0; i < frames; i++) {
      auto orig_pic = xvc_test::TestYuvPic(width, height, GetParam(), i, i);
      EncodeOneFrame(orig_pic.GetBytes(), orig_pic.GetBitdepth());
      orig_pics_.push_back(orig_pic);
      verified_.push_back(false);
    }
    EncoderFlush();
  }

  void Decode(int width, int height, int frames, bool do_flush = true) {
    DecodeSegmentHeaderSuccess(GetNextNalToDecode());
    for (int i = 0; i < frames; i++) {
      if (DecodePictureSuccess(GetNextNalToDecode())) {
        VerifyPicture(width, height, last_decoded_picture_);
      }
    }
    while (do_flush && DecoderFlush()) {
      VerifyPicture(width, height, last_decoded_picture_);
    }
  }

  void VerifyPicture(int width, int height,
                     const xvc_decoded_picture &decoded_picture) {
    int poc = decoded_picture.stats.poc;
    EXPECT_EQ(width, decoded_picture.stats.width);
    EXPECT_EQ(height, decoded_picture.stats.height);
    EXPECT_FALSE(verified_[poc]);
    double psnr = orig_pics_[poc].CalcPsnr(decoded_picture.bytes);
    EXPECT_GE(psnr, kPsnrThreshold) << "Picture poc " << poc;
    verified_[poc] = true;
  }

  std::vector<xvc_test::TestYuvPic> orig_pics_;
  std::vector<bool> verified_;
};

TEST_P(EncodeDecodeTest, TwoSubGopZeroResolution) {
  Encode(0, 0, kFramesEncoded * 2 + 1);
  Decode(0, 0, kFramesEncoded * 2 + 1);
}

TEST_P(EncodeDecodeTest, TwoSubGop24x24) {
  Encode(24, 24, kFramesEncoded * 2 + 1);
  Decode(24, 24, kFramesEncoded * 2 + 1);
}

TEST_P(EncodeDecodeTest, SingleSegment16x16) {
  Encode(16, 16, kSegmentLength + 1);
  Decode(16, 16, kSegmentLength, false);
  Decode(16, 16, 1, true);
}

INSTANTIATE_TEST_CASE_P(NormalBitdepth, EncodeDecodeTest,
                        ::testing::Values(8));
#if XVC_HIGH_BITDEPTH
INSTANTIATE_TEST_CASE_P(HighBitdepth, EncodeDecodeTest,
                        ::testing::Values(10));
#endif

}   // namespace
