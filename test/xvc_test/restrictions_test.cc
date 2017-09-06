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

#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_test/test_helper.h"
#include "xvc_test/yuv_helper.h"

namespace {

static constexpr int kSubGopLength = 8;

class RestrictionsTest : public ::testing::TestWithParam<int>,
  public ::xvc_test::EncoderHelper, public ::xvc_test::DecoderHelper {
protected:
  void SetUp() override {
    EncoderHelper::Init();
    DecoderHelper::Init();
  }

  void Encode(const xvc::EncoderSettings &settings, int width, int height,
              int frames) {
    encoder_ = CreateEncoder(settings, width, height, 8, kDefaultQp);
    encoder_->SetInternalBitdepth(GetParam());
    encoder_->SetSubGopLength(kSubGopLength);
    encoder_->SetSegmentLength(kSubGopLength * 2 + 1);
    encoder_->SetQp(kDefaultQp);
    encoder_->SetResolution(width, height);
    for (int i = 0; i < frames; i++) {
      auto orig_pic = xvc_test::TestYuvPic(width, height, GetParam(), i, i);
      EncodeOneFrame(orig_pic.GetBytes(), orig_pic.GetBitdepth());
      orig_pics_.push_back(orig_pic);
      verified_.push_back(false);
    }
    EncoderFlush();
  }

  void Decode(int width, int height, int frames1, int frames2) {
    DecodeSegmentHeaderSuccess(GetNextNalToDecode());
    for (int i = 0; i < frames1; i++) {
      if (DecodePictureSuccess(GetNextNalToDecode())) {
        VerifyPicture(width, height, last_decoded_picture_);
      }
    }
    DecodeSegmentHeaderSuccess(GetNextNalToDecode());
    for (int i = 0; i < frames1; i++) {
      if (DecodePictureSuccess(GetNextNalToDecode())) {
        VerifyPicture(width, height, last_decoded_picture_);
      }
    }
    while (DecoderFlushAndGet()) {
      VerifyPicture(width, height, last_decoded_picture_);
    }
    for (int i = 0; i < frames1 + frames2; i++) {
      ASSERT_TRUE(verified_[i]) << " poc " << i;
      verified_[i] = false;
    }
  }

  void VerifyPicture(int width, int height,
                     const xvc_decoded_picture &decoded_picture) {
    int poc = decoded_picture.stats.poc;
    EXPECT_EQ(width, decoded_picture.stats.width);
    EXPECT_EQ(height, decoded_picture.stats.height);
    EXPECT_FALSE(verified_[poc]);
    verified_[poc] = true;
  }

  std::vector<xvc_test::TestYuvPic> orig_pics_;
  std::vector<bool> verified_;
};

typedef RestrictionsTest RestrictionsDeathTest;

TEST_P(RestrictionsTest, HandlesSwitchingRestrictionAtSegmentHeader) {
  xvc::EncoderSettings enc_settings1;
  enc_settings1.Initialize(xvc::SpeedMode::kSlow);
  xvc::EncoderSettings enc_settings2;
  enc_settings2.Initialize(xvc::RestrictedMode::kModeA);

  Encode(enc_settings1, 16, 16, kSubGopLength + 1);
  Encode(enc_settings2, 16, 16, kSubGopLength + 1);
  Decode(16, 16, kSubGopLength + 1, kSubGopLength + 1);
}

TEST_P(RestrictionsTest, SupportParallelDecodeWhenRestrictionChanges) {
  const int num_iterations = 10;
  xvc::EncoderSettings enc_settings1;
  enc_settings1.Initialize(xvc::SpeedMode::kSlow);
  xvc::EncoderSettings enc_settings2;
  enc_settings2.Initialize(xvc::RestrictedMode::kModeA);

  Encode(enc_settings1, 16, 16, kSubGopLength + 1);
  Encode(enc_settings2, 16, 16, kSubGopLength + 1);
  DecoderHelper::Init(true);
  int decoded_pictures = 0;
  for (int iterations = 0; iterations < num_iterations; iterations++) {
    for (int i = 0; i < static_cast<int>(encoded_nal_units_.size()); i++) {
      auto &nal = encoded_nal_units_[i];
      decoder_->DecodeNal(&nal[0], nal.size());
      if (decoder_->GetDecodedPicture(&last_decoded_picture_)) {
        decoded_pictures++;
      }
    }
  }
  while (DecoderFlushAndGet()) {
    decoded_pictures++;
  }
  ASSERT_EQ(decoded_pictures, num_iterations * 2 * (kSubGopLength + 1));
}

TEST_P(RestrictionsDeathTest, FailsIfRestrictionFlagChangesAreNotSignaled) {
  xvc::EncoderSettings enc_settings1;
  enc_settings1.Initialize(xvc::SpeedMode::kSlow);
  xvc::EncoderSettings enc_settings2;
  enc_settings2.Initialize(xvc::RestrictedMode::kModeA);

  Encode(enc_settings1, 16, 16, kSubGopLength + 1);
  size_t nbr_of_nal_units = encoded_nal_units_.size();
  Encode(enc_settings2, 16, 16, kSubGopLength + 1);
  encoded_nal_units_[nbr_of_nal_units] = encoded_nal_units_[0];
  ASSERT_DEATH({ Decode(16, 16, kSubGopLength + 1, kSubGopLength + 1); },
               "0");
}

INSTANTIATE_TEST_CASE_P(NormalBitdepth, RestrictionsTest,
                        ::testing::Values(8));
#if XVC_HIGH_BITDEPTH
INSTANTIATE_TEST_CASE_P(HighBitdepth, RestrictionsTest,
                        ::testing::Values(10));
#endif

}   // namespace
