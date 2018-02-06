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

#include <algorithm>
#include <list>
#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_test/decoder_helper.h"
#include "xvc_test/encoder_helper.h"
#include "xvc_test/yuv_helper.h"

namespace {

static constexpr int kWidth = 16;
static constexpr int kHeight = 8;
static constexpr int kQp = 20;
static constexpr double kPsnrThreshold = 41.0;
static constexpr int kPocOffset = 999;  // to avoid start at zero

class AllIntraTest : public ::testing::TestWithParam<int>,
  public ::xvc_test::EncoderHelper, public ::xvc_test::DecoderHelper {
protected:
  void SetUp() override {
    EncoderHelper::Init(GetParam());
    DecoderHelper::Init();
    encoder_->SetResolution(kWidth, kHeight);
    encoder_->SetSubGopLength(1);
    encoder_->SetNumRefPics(0);
    encoder_->SetQp(kQp);
  }

  void TearDown() override {
    for (int poc = 0; poc < static_cast<int>(verified_.size()); poc++) {
      ASSERT_TRUE(verified_[poc]) << "Picture poc " << poc;
    }
  }

  void Encode(int frames) {
    for (int i = 0; i < frames; i++) {
      xvc_test::TestYuvPic orig_pic{ kWidth, kHeight, GetParam(), i, i };
      std::vector<xvc_enc_nal_stats> nal_stats =
        EncodeOneFrame(orig_pic.GetBytes(), orig_pic.GetBitdepth());
      orig_pics_.emplace_back(std::move(orig_pic));
      verified_.push_back(false);
      for (xvc_enc_nal_stats stats : nal_stats) {
        encoded_pocs_.push_back(stats.poc);
      }
    }
    for (xvc_enc_nal_stats stats : EncoderFlush()) {
      encoded_pocs_.push_back(stats.poc);
    }
    for (int i = 0; i < frames; i++) {
      EXPECT_NE(std::find(encoded_pocs_.begin(), encoded_pocs_.end(), i),
                encoded_pocs_.end()) << " poc " << i;
    }
  }

  void Decode(int segment_header_nal_interval, bool check_psnr = true) {
    DecodeSegmentHeaderSuccess(GetNextNalToDecode());
    encoded_pocs_.pop_front();    // ignore segment header nal
    int nals_decoded = 1;
    while (HasMoreNals()) {
      ASSERT_FALSE(encoded_pocs_.empty());
      int64_t user_data = kPocOffset + encoded_pocs_.front();
      encoded_pocs_.pop_front();
      if (segment_header_nal_interval > 0 &&
          nals_decoded++ % segment_header_nal_interval == 0) {
        DecodeSegmentHeaderSuccess(GetNextNalToDecode());
        continue;
      }
      if (DecodePictureSuccess(GetNextNalToDecode(), user_data)) {
        if (HasFailure())
          return;
        VerifyPicture(kWidth, kHeight, check_psnr, last_decoded_picture_);
      }
    }
    while (DecoderFlushAndGet()) {
      if (HasFailure())
        return;
      VerifyPicture(kWidth, kHeight, check_psnr, last_decoded_picture_);
    }
  }

  void VerifyPicture(int width, int height, bool check_psnr,
                     const xvc_decoded_picture &decoded_picture) {
    int poc = decoded_picture.stats.poc;
    ASSERT_EQ(poc, decoded_picture.user_data - kPocOffset);
    ASSERT_NO_FATAL_FAILURE(AssertValidPicture420(width, height,
                                                  decoded_picture));
    EXPECT_FALSE(verified_[poc]);
    verified_[poc] = true;
    if (decoded_picture.size > 0) {
      EXPECT_TRUE(xvc_test::TestYuvPic::SamePictureBytes(
        &rec_pics_[poc][0], rec_pics_[poc].size(),
        reinterpret_cast<const uint8_t*>(decoded_picture.bytes),
        decoded_picture.size));
    }
    if (encoder_->GetCurrentSegment()->max_sub_gop_length == 1) {
      // only reliable to check qp on when not using any sub-gop structure
      EXPECT_EQ(kQp, decoded_picture.stats.qp);
    }
    if (check_psnr) {
      double psnr = orig_pics_[poc].CalcPsnr(decoded_picture.bytes);
      EXPECT_GE(psnr, kPsnrThreshold) << "Picture poc " << poc;
    }
  }

  std::vector<xvc_test::TestYuvPic> orig_pics_;
  std::vector<bool> verified_;
  std::list<int> encoded_pocs_;
};

TEST_P(AllIntraTest, OnePicture) {
  encoder_->SetSegmentLength(1000);
  Encode(1);
  Decode(0);
}

TEST_P(AllIntraTest, TwoPictures) {
  encoder_->SetSegmentLength(1000);
  Encode(2);
  Decode(0);
}

TEST_P(AllIntraTest, ThreePictures) {
  encoder_->SetSegmentLength(1000);
  Encode(3);
  Decode(0);
}

TEST_P(AllIntraTest, ThreePicturesSegmentHeaderEveryPic) {
  encoder_->SetSegmentLength(1);
  Encode(3);
  Decode(2);
}

TEST_P(AllIntraTest, TenPictures) {
  Encode(10);
  Decode(0);
}

TEST_P(AllIntraTest, SubGopLength4) {
  encoder_->SetSubGopLength(4);
  Encode(6);
  Decode(0, false);
}

TEST_P(AllIntraTest, SegmentHeaderBeforeEveryPicture) {
  encoder_->SetSegmentLength(1);
  Encode(10);
  Decode(2);
}

TEST_P(AllIntraTest, ClosedGopEveryPicture) {
  encoder_->SetSegmentLength(1);
  encoder_->SetClosedGopInterval(1);
  Encode(9);
  Decode(2);
}

TEST_P(AllIntraTest, ClosedGopEvery3rdPicture) {
  encoder_->SetSegmentLength(1);
  encoder_->SetClosedGopInterval(3);
  Encode(11);
  Decode(2);
}

INSTANTIATE_TEST_CASE_P(NormalBitdepth, AllIntraTest,
                        ::testing::Values(8));
#if XVC_HIGH_BITDEPTH
INSTANTIATE_TEST_CASE_P(HighBitdepth, AllIntraTest,
                        ::testing::Values(10));
#endif

}   // namespace
