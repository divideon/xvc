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

#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_test/decoder_helper.h"
#include "xvc_test/encoder_helper.h"
#include "xvc_test/yuv_helper.h"

using xvc_test::NalUnit;

namespace {

static constexpr int kWidth = 32;
static constexpr int kHeight = 32;
static constexpr int kQp = 27;
static constexpr int kSubGopLength = 8;
static constexpr int kSegmentLength = kSubGopLength * 3;

class SimdTest : public ::testing::TestWithParam<int>,
  public ::xvc_test::EncoderHelper, public ::xvc_test::DecoderHelper {
protected:
  void SetUp() override {
    no_simd_caps_ = std::set<xvc::CpuCapability>();
    all_simd_caps_ = xvc::SimdCpu::GetRuntimeCapabilities();
    encoder_separate_caps_ = xvc::SimdCpu::GetMaskedCaps(
      (1 << static_cast<int>(xvc::CpuCapability::kSse2)) |
      (1 << static_cast<int>(xvc::CpuCapability::kSse4_2)) |
      (1 << static_cast<int>(xvc::CpuCapability::kAvx)) |
      (1 << static_cast<int>(xvc::CpuCapability::kAvx2)));
  }

  std::vector<NalUnit> Encode(int width, int height, int frames,
                              const std::set<xvc::CpuCapability> &simd_caps) {
    EncoderHelper::Init(GetParam());
    encoder_->SetSubGopLength(kSubGopLength);
    encoder_->SetSegmentLength(kSegmentLength);
    encoder_->SetQp(kQp);
    encoder_->SetCpuCapabilities(simd_caps);
    encoder_->SetResolution(width, height);
    for (int i = 0; i < frames; i++) {
      auto orig_pic = xvc_test::TestYuvPic(width, height, GetParam(), i, i);
      EncodeOneFrame(orig_pic.GetBytes(), orig_pic.GetBitdepth());
      orig_pics_.push_back(orig_pic);
      verified_.push_back(false);
    }
    EncoderFlush();
    return encoded_nal_units_;
  }

  std::vector<std::vector<uint8_t>>
    Decode(int width, int height, int frames,
           const std::set<xvc::CpuCapability> &simd_caps) {
    ResetBitstreamPosition();
    DecoderHelper::Init();
    decoder_->SetCpuCapabilities(simd_caps);
    std::vector<std::vector<uint8_t>> dec_pic_bytes(verified_.size());
    DecodeSegmentHeaderSuccess(GetNextNalToDecode());
    for (int i = 0; i < frames; i++) {
      if (DecodePictureSuccess(GetNextNalToDecode())) {
        VerifyPicture(width, height, last_decoded_picture_, &dec_pic_bytes);
      }
    }
    while (DecoderFlushAndGet()) {
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

  ::testing::AssertionResult
    IsPicturesEqual(const std::vector<std::vector<uint8_t>> &seq1,
                    const std::vector<std::vector<uint8_t>> &seq2) {
    if (seq1.size() != seq2.size()) {
      return ::testing::AssertionFailure() <<
        seq1.size() << " not equal to " << seq2.size();
    }
    for (int poc = 0; poc < static_cast<int>(seq1.size()); poc++) {
      if (seq1[poc].size() != seq2[poc].size()) {
        return ::testing::AssertionFailure() <<
          "different size for poc=" << poc;
      }
      for (int pos = 0; pos < static_cast<int>(seq1[poc].size()); pos++) {
        if (seq1[poc][pos] != seq2[poc][pos]) {
          return ::testing::AssertionFailure() <<
            "for poc= " << poc << " pos= " << pos;
        }
      }
    }
    return ::testing::AssertionSuccess();
  }

  ::testing::AssertionResult
    IsBitsteramEqual(std::vector<NalUnit> bitstream1,
                     std::vector<NalUnit> bitstream2) {
    EXPECT_EQ(bitstream1.size(), bitstream2.size());
    for (size_t i = 0; i < bitstream1.size(); i++) {
      if (bitstream1[i] != bitstream2[i]) {
        return ::testing::AssertionFailure() <<
          "bitstreams not equal for nal unit " << i;
      }
    }
    return ::testing::AssertionSuccess();
  }

  std::set<xvc::CpuCapability> no_simd_caps_;
  std::set<xvc::CpuCapability> all_simd_caps_;
  std::set<xvc::CpuCapability> encoder_separate_caps_;
  std::vector<xvc_test::TestYuvPic> orig_pics_;
  std::vector<bool> verified_;
};

TEST_P(SimdTest, EncoderSimd) {
  std::vector<NalUnit> nals_without =
    Encode(kWidth, kHeight, kSubGopLength + 1, no_simd_caps_);
  Decode(kWidth, kHeight, kSubGopLength + 1, all_simd_caps_);  // can decode
  std::vector<NalUnit> nals_with =
    Encode(kWidth, kHeight, kSubGopLength + 1, all_simd_caps_);
  ASSERT_TRUE(IsBitsteramEqual(nals_without, nals_with));
  // Run *a limited set* of supported vector instruction sets individually
  for (xvc::CpuCapability cpu_cap : encoder_separate_caps_) {
    std::vector<NalUnit> bitstream =
      Encode(kWidth, kHeight, kSubGopLength + 1, { cpu_cap });
    ASSERT_TRUE(IsBitsteramEqual(nals_without, bitstream)) <<
      "for cap=" << static_cast<int>(cpu_cap);
  }
}

TEST_P(SimdTest, DecoderSimd) {
  Encode(kWidth, kHeight, kSubGopLength + 1, no_simd_caps_);
  auto dec_plain = Decode(kWidth, kHeight, kSubGopLength + 1, no_simd_caps_);
  auto dec_simd = Decode(kWidth, kHeight, kSubGopLength + 1, all_simd_caps_);
  ASSERT_TRUE(IsPicturesEqual(dec_plain, dec_simd));
  // Run *all* supported vector instruction sets individually
  for (xvc::CpuCapability cpu_cap : all_simd_caps_) {
    auto dec_single = Decode(kWidth, kHeight, kSubGopLength + 1, { cpu_cap });
    ASSERT_TRUE(IsPicturesEqual(dec_plain, dec_single)) <<
      "for cap=" << static_cast<int>(cpu_cap);
  }
}

INSTANTIATE_TEST_CASE_P(NormalBitdepth, SimdTest,
                        ::testing::Values(8));
#if XVC_HIGH_BITDEPTH
INSTANTIATE_TEST_CASE_P(HighBitdepth, SimdTest,
                        ::testing::Values(10, 12));
#endif

}   // namespace
