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

static constexpr int kWidth = 32;
static constexpr int kHeight = 32;
static constexpr int kQp = 27;
static constexpr int kSubGopLength = 8;
static constexpr int kSegmentLength = kSubGopLength * 3;

class SimdTest : public ::testing::TestWithParam<int>,
  public ::xvc_test::EncoderHelper, public ::xvc_test::DecoderHelper {
protected:
  void Encode(int width, int height, int frames, bool use_simd) {
    EncoderHelper::Init();
    encoder_->SetInternalBitdepth(GetParam());
    encoder_->SetSubGopLength(kSubGopLength);
    encoder_->SetSegmentLength(kSegmentLength);
    encoder_->SetQp(kQp);
    if (use_simd) {
      encoder_->SetCpuCapabilities(xvc::SimdCpu::GetRuntimeCapabilities());
    } else {
      encoder_->SetCpuCapabilities(std::set<xvc::CpuCapability>());
    }
    encoder_->SetResolution(width, height);
    for (int i = 0; i < frames; i++) {
      auto orig_pic = xvc_test::TestYuvPic(width, height, GetParam(), i, i);
      EncodeOneFrame(orig_pic.GetBytes(), orig_pic.GetBitdepth());
      orig_pics_.push_back(orig_pic);
      verified_.push_back(false);
    }
    EncoderFlush();
  }

  std::vector<std::vector<uint8_t>> Decode(int width, int height, int frames,
                                           bool use_simd) {
    DecoderHelper::Init();
    if (use_simd) {
      decoder_->SetCpuCapabilities(xvc::SimdCpu::GetRuntimeCapabilities());
    } else {
      decoder_->SetCpuCapabilities(std::set<xvc::CpuCapability>());
    }
    std::vector<std::vector<uint8_t>> dec_pic_bytes(verified_.size());
    DecodeSegmentHeaderSuccess(GetNextNalToDecode());
    for (int i = 0; i < frames; i++) {
      if (DecodePictureSuccess(GetNextNalToDecode())) {
        VerifyPicture(width, height, last_decoded_picture_, &dec_pic_bytes);
      }
    }
    while (DecoderFlush()) {
      VerifyPicture(width, height, last_decoded_picture_, &dec_pic_bytes);
    }
    for (int poc = 0; poc < static_cast<int>(verified_.size()); poc++) {
      EXPECT_TRUE(verified_[poc]) << "Picture poc " << poc;
      verified_[poc] = false;
    }
    return dec_pic_bytes;
  }

  void VerifyPicture(int width, int height,
                     const xvc_decoded_picture &decoded_picture,
                     std::vector<std::vector<uint8_t>> *dec_pic_bytes) {
    int poc = decoded_picture.stats.poc;
    EXPECT_EQ(width, decoded_picture.stats.width);
    EXPECT_EQ(height, decoded_picture.stats.height);
    EXPECT_FALSE(verified_[poc]);
    (*dec_pic_bytes)[poc].resize(decoded_picture.size);
    std::memcpy(&(*dec_pic_bytes)[poc][0], decoded_picture.bytes,
                decoded_picture.size);
    verified_[poc] = true;
  }

  void AssertPicturesEqual(const std::vector<std::vector<uint8_t>> &seq1,
                           const std::vector<std::vector<uint8_t>> &seq2) {
    ASSERT_EQ(seq1.size(), seq2.size());
    for (int poc = 0; poc < static_cast<int>(seq1.size()); poc++) {
      ASSERT_EQ(seq1[poc].size(), seq2[poc].size());
      for (int pos = 0; pos < static_cast<int>(seq1[poc].size()); pos++) {
        ASSERT_TRUE(seq1[poc][pos] == seq2[poc][pos])
          << "for poc= " << poc << " pos= " << pos;
      }
    }
  }

  std::vector<xvc_test::TestYuvPic> orig_pics_;
  std::vector<bool> verified_;
};

TEST_P(SimdTest, VerifyDecodeWithWithout) {
  Encode(kWidth, kHeight, kSubGopLength + 1, false);
  ResetBitstreamPosition();
  auto dec_plain = Decode(kWidth, kHeight, kSubGopLength + 1, false);
  ResetBitstreamPosition();
  auto dec_simd = Decode(kWidth, kHeight, kSubGopLength + 1, true);
  AssertPicturesEqual(dec_plain, dec_simd);
}

INSTANTIATE_TEST_CASE_P(NormalBitdepth, SimdTest,
                        ::testing::Values(8));
#if XVC_HIGH_BITDEPTH
INSTANTIATE_TEST_CASE_P(HighBitdepth, SimdTest,
                        ::testing::Values(10, 12));
#endif

}   // namespace
