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

static constexpr int kQp = 27;
static constexpr double kPsnrThreshold = 33.0;
static constexpr int kFramesEncoded = 3;

class ResolutionTest : public ::testing::TestWithParam<int>,
  public ::xvc_test::EncoderHelper, public ::xvc_test::DecoderHelper {
protected:
  void SetUp() override {
    EncoderHelper::Init();
    DecoderHelper::Init();
    encoder_->SetInternalBitdepth(GetParam());
    encoder_->SetSubGopLength(1);
    encoder_->SetQp(kQp);
  }

  void EncodeDecode(int width, int height) {
    std::vector<xvc_test::TestYuvPic> orig_pics;
    encoder_->SetResolution(width, height);
    for (int poc = 0; poc < kFramesEncoded; poc++) {
      auto orig_pic = xvc_test::TestYuvPic(width, height, GetParam(), poc, poc);
      EncodeOneFrame(orig_pic.GetBytes(), orig_pic.GetBitdepth());
      orig_pics.push_back(orig_pic);
    }
    // TODO(PH) Consider re-implementing without orig pic buffering
    // when we support low-delay decoding
    for (int poc = 0; poc < kFramesEncoded; poc++) {
      if (poc == 0) {
        DecodeSegmentHeaderSuccess(GetNextNalToDecode());
      }
      if (DecodePictureSuccess(GetNextNalToDecode())) {
        auto &orig_pic = orig_pics.front();
        double psnr = orig_pic.CalcPsnr(last_decoded_picture_.bytes);
        ASSERT_GE(psnr, kPsnrThreshold);
        orig_pics.erase(orig_pics.begin());
      }
    }
    for (auto &orig_pic : orig_pics) {
      ASSERT_TRUE(DecoderFlush());
      double psnr = orig_pic.CalcPsnr(last_decoded_picture_.bytes);
      ASSERT_GE(psnr, kPsnrThreshold);
    }
    ASSERT_FALSE(DecoderFlush());
  }
};

TEST_P(ResolutionTest, Size8xN) {
  EncodeDecode(8, xvc_test::TestYuvPic::kDefaultSize);
}

TEST_P(ResolutionTest, SizeNx8) {
  EncodeDecode(xvc_test::TestYuvPic::kDefaultSize, 8);
}

TEST_P(ResolutionTest, Size24xN) {
  EncodeDecode(24, xvc_test::TestYuvPic::kDefaultSize);
}

TEST_P(ResolutionTest, SizeNx24) {
  EncodeDecode(xvc_test::TestYuvPic::kDefaultSize, 24);
}

INSTANTIATE_TEST_CASE_P(NormalBitdepth, ResolutionTest,
                        ::testing::Values(8));
#if XVC_HIGH_BITDEPTH
INSTANTIATE_TEST_CASE_P(HighBitdepth, ResolutionTest,
                        ::testing::Values(10));
#endif

}   // namespace
