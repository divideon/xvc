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

#include <list>
#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_test/test_helper.h"
#include "xvc_test/yuv_helper.h"

namespace {

static constexpr int kQp = 27;
static constexpr double kPsnrThreshold = 28.0;
static constexpr int kFramesEncoded = 8;
static constexpr int kSegmentLength = kFramesEncoded * 3;
static constexpr int kPocOffset = 999;

class EncodeDecodeTest : public ::testing::TestWithParam<int>,
  public ::xvc_test::EncoderHelper, public ::xvc_test::DecoderHelper {
protected:
  void SetUp() override {
    EncoderHelper::Init(GetParam());
    DecoderHelper::Init();
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
      auto nals = EncodeOneFrame(orig_pic.GetBytes(), orig_pic.GetBitdepth());
      orig_pics_.push_back(orig_pic);
      verified_.push_back(false);
      for (auto nal : nals) {
        encoded_pocs_.push_back(nal.stats.poc);
      }
    }
    EncoderFlush();
  }

  void Decode(int width, int height, int frames, bool do_flush = true) {
    DecodeSegmentHeaderSuccess(GetNextNalToDecode());
    encoded_pocs_.pop_front();
    for (int i = 0; i < frames; i++) {
      int64_t user_data = kPocOffset + encoded_pocs_.front();
      encoded_pocs_.pop_front();
      if (DecodePictureSuccess(GetNextNalToDecode(), user_data)) {
        VerifyPicture(width, height, last_decoded_picture_);
      }
    }
    while (do_flush && DecoderFlushAndGet()) {
      VerifyPicture(width, height, last_decoded_picture_);
    }
  }

  void VerifyPicture(int width, int height,
                     const xvc_decoded_picture &decoded_picture) {
    int poc = decoded_picture.stats.poc;
    int byte_width = width * (decoded_picture.stats.bitdepth == 8 ? 1 : 2);
    EXPECT_EQ(kPocOffset + poc, decoded_picture.user_data);
    EXPECT_EQ(width, decoded_picture.stats.width);
    EXPECT_EQ(height, decoded_picture.stats.height);
    EXPECT_EQ(decoded_picture.planes[0], decoded_picture.bytes);
    EXPECT_EQ(decoded_picture.planes[1], decoded_picture.planes[0] +
              decoded_picture.stride[0] * height);
    EXPECT_EQ(decoded_picture.planes[2], decoded_picture.planes[1] +
              decoded_picture.stride[1] * height / 2);
    EXPECT_EQ(byte_width, decoded_picture.stride[0]);
    EXPECT_EQ(byte_width / 2, decoded_picture.stride[1]);
    EXPECT_EQ(byte_width / 2, decoded_picture.stride[2]);
    EXPECT_FALSE(verified_[poc]);
    double psnr = orig_pics_[poc].CalcPsnr(decoded_picture.bytes);
    EXPECT_GE(psnr, kPsnrThreshold) << "Picture poc " << poc;
    verified_[poc] = true;
  }

  std::vector<xvc_test::TestYuvPic> orig_pics_;
  std::vector<bool> verified_;
  std::list<int> encoded_pocs_;
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
