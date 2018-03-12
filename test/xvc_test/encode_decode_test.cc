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

#include "xvc_test/decoder_helper.h"
#include "xvc_test/encoder_helper.h"
#include "xvc_test/yuv_helper.h"

namespace {

struct TestParam {
  int internal_bitdepth;
  bool use_leading_pictures;
};

static constexpr int kQp = 27;
static constexpr double kPsnrThreshold = 28.0;
static constexpr int kFramesEncoded = 8;
static constexpr int kSegmentLength = kFramesEncoded * 3;
static constexpr int kPocOffset = 999;

class EncodeDecodeTest : public ::testing::TestWithParam<TestParam>,
  public ::xvc_test::EncoderHelper, public ::xvc_test::DecoderHelper {
protected:
  void SetUp() override {
    xvc::EncoderSettings encoder_settings = GetDefaultEncoderSettings();
    encoder_settings.leading_pictures = GetParam().use_leading_pictures ? 1 : 0;
    SetupEncoder(encoder_settings, 0, 0, GetParam().internal_bitdepth, kQp);
    encoder_->SetSubGopLength(kFramesEncoded);
    encoder_->SetSegmentLength(kSegmentLength);
    DecoderHelper::Init();
  }

  void TearDown() override {
    for (int poc = 0; poc < static_cast<int>(verified_.size()); poc++) {
      ASSERT_TRUE(verified_[poc]) << "Picture poc " << poc;
    }
  }

  void Encode(int width, int height, int frames) {
    const int input_bitdepth = GetParam().internal_bitdepth;
    encoder_->SetResolution(width, height);
    for (int i = 0; i < frames; i++) {
      xvc_test::TestYuvPic orig_pic(width, height, input_bitdepth, i, i);
      std::vector<xvc_enc_nal_stats> nal_stats =
        EncodeOneFrame(orig_pic.GetBytes(), orig_pic.GetBitdepth());
      orig_pics_.emplace_back(std::move(orig_pic));
      verified_.push_back(false);
      for (auto stats : nal_stats) {
        encoded_pocs_.push_back(stats.poc);
      }
    }
    for (xvc_enc_nal_stats stats : EncoderFlush()) {
      encoded_pocs_.push_back(stats.poc);
    }
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
    ASSERT_EQ(poc, decoded_picture.user_data - kPocOffset);
    ASSERT_NO_FATAL_FAILURE(AssertValidPicture420(width, height,
                                                  decoded_picture));
    EXPECT_FALSE(verified_[poc]);
    verified_[poc] = true;
    double psnr = orig_pics_[poc].CalcPsnr(decoded_picture.bytes);
    EXPECT_GE(psnr, kPsnrThreshold) << "Picture poc " << poc;
    if (decoded_picture.size > 0) {
      EXPECT_TRUE(xvc_test::TestYuvPic::SamePictureBytes(
        &rec_pics_[poc][0], rec_pics_[poc].size(),
        reinterpret_cast<const uint8_t*>(decoded_picture.bytes),
        decoded_picture.size));
    }
  }

  std::vector<xvc_test::TestYuvPic> orig_pics_;
  std::vector<bool> verified_;
  std::list<int> encoded_pocs_;
};

TEST_P(EncodeDecodeTest, TwoSubGopZeroResolution) {
  const int nbr_pictures = kFramesEncoded * 2 +
    (!GetParam().use_leading_pictures ? 1 : 0);
  Encode(0, 0, nbr_pictures);
  Decode(0, 0, nbr_pictures);
}

TEST_P(EncodeDecodeTest, TwoSubGop24x24) {
  const int nbr_pictures = kFramesEncoded * 2 +
    (!GetParam().use_leading_pictures ? 1 : 0);
  Encode(24, 24, nbr_pictures);
  Decode(24, 24, nbr_pictures);
}

TEST_P(EncodeDecodeTest, LowDelayTwoSubGop24x24) {
  const int nbr_pictures = kFramesEncoded * 2 +
    (!GetParam().use_leading_pictures ? 1 : 0);
  encoder_->SetLowDelay(true);
  Encode(24, 24, nbr_pictures);
  Decode(24, 24, nbr_pictures);
}

TEST_P(EncodeDecodeTest, SingleSegment16x16) {
  if (GetParam().use_leading_pictures) {
    Encode(16, 16, kSegmentLength);
    Decode(16, 16, kSegmentLength);
  } else {
    Encode(16, 16, kSegmentLength + 1);
    Decode(16, 16, kSegmentLength, false);
    Decode(16, 16, 1, true);
  }
}

INSTANTIATE_TEST_CASE_P(NormalBitdepth, EncodeDecodeTest,
                        ::testing::Values(TestParam({ 8, false }),
                                          TestParam({ 8, true })));
#if XVC_HIGH_BITDEPTH
INSTANTIATE_TEST_CASE_P(HighBitdepth, EncodeDecodeTest,
                        ::testing::Values(TestParam({ 10, false }),
                                          TestParam({ 10, true })));
#endif

}   // namespace
