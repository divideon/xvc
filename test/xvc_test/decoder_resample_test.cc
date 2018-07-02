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
#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_test/decoder_helper.h"
#include "xvc_test/encoder_helper.h"
#include "xvc_test/yuv_helper.h"

namespace {

struct TestParam {
  bool explicit_output_format;
  bool use_src_padding;
};

static const int kPicSize = 16;
static const int kBitdepth = 8;
// Decoding exactly the same sample value depends on quantization level...
static const int kQp1 = 18;
static const xvc::Sample kSample1 = 43;
static const int kQp2 = 23;
static const xvc::Sample kSample2 = 96;

class DecoderResampleTest : public ::testing::TestWithParam<TestParam>,
  public ::xvc_test::EncoderHelper, public ::xvc_test::DecoderHelper {
protected:
  void SetUp() override {
    EncoderHelper::Init();
    DecoderHelper::Init();
  }

  void EncodeSegment(xvc::Sample orig_sample, int  qp, int resolution,
                     int bitdepth,
                     xvc::ChromaFormat chroma_fmt = xvc::ChromaFormat::k420) {
    xvc::EncoderSettings encoder_settings = GetDefaultEncoderSettings();
    encoder_settings.source_padding = GetParam().use_src_padding;
    SetupEncoder(encoder_settings, resolution, resolution, bitdepth, qp);
    encoder_->SetSubGopLength(1);
    encoder_->SetSegmentLength(1);
    encoder_->SetChromaFormat(chroma_fmt);
    auto pic_bytes = CreateSampleBuffer(orig_sample, bitdepth);
    auto nals = EncodeOneFrame(pic_bytes, bitdepth);
    ASSERT_EQ(2, nals.size());
  }

  void DecodeResolution(int size_dec, int expected_segments = 2) {
    ASSERT_EQ(expected_segments * 2, encoded_nal_units_.size());
    if (GetParam().explicit_output_format) {
      decoder_->SetOutputWidth(size_dec);
      decoder_->SetOutputHeight(size_dec);
    }

    DecodeSegmentHeaderSuccess(GetNextNalToDecode());
    auto dec_pic = DecodeAndFlush(GetNextNalToDecode());
    EXPECT_EQ(size_dec, dec_pic->stats.width);
    EXPECT_EQ(size_dec, dec_pic->stats.height);
    EXPECT_EQ(kBitdepth, dec_pic->stats.bitdepth);
    EXPECT_TRUE(VerifyDecodedLumaEquals(*dec_pic, kSample1));
    if (expected_segments > 1) {
      DecodeSegmentHeaderSuccess(GetNextNalToDecode());
      dec_pic = DecodeAndFlush(GetNextNalToDecode());
      EXPECT_EQ(size_dec, dec_pic->stats.width);
      EXPECT_EQ(size_dec, dec_pic->stats.height);
      EXPECT_EQ(kBitdepth, dec_pic->stats.bitdepth);
      EXPECT_TRUE(VerifyDecodedLumaEquals(*dec_pic, kSample2));
    }
  }

  void DecodeChromaFormat(xvc_dec_chroma_format chroma_fmt_dec) {
    ASSERT_EQ(4, encoded_nal_units_.size());
    if (GetParam().explicit_output_format) {
      decoder_->SetOutputChromaFormat(chroma_fmt_dec);
    }

    DecodeSegmentHeaderSuccess(GetNextNalToDecode());
    auto dec_pic = DecodeAndFlush(GetNextNalToDecode());
    EXPECT_EQ(kPicSize, dec_pic->stats.width);
    EXPECT_EQ(kPicSize, dec_pic->stats.height);
    EXPECT_EQ(kBitdepth, dec_pic->stats.bitdepth);
    EXPECT_EQ(chroma_fmt_dec, dec_pic->stats.chroma_format);

    DecodeSegmentHeaderSuccess(GetNextNalToDecode());
    dec_pic = DecodeAndFlush(GetNextNalToDecode());
    EXPECT_EQ(kPicSize, dec_pic->stats.width);
    EXPECT_EQ(kPicSize, dec_pic->stats.height);
    EXPECT_EQ(kBitdepth, dec_pic->stats.bitdepth);
    EXPECT_EQ(chroma_fmt_dec, dec_pic->stats.chroma_format);
  }

  void DecodeBitdepth(int size_dec, int bitdepth_enc1, int bitdepth_enc2,
                      int bitdepth_dec) {
    ASSERT_EQ(4, encoded_nal_units_.size());
    if (GetParam().explicit_output_format) {
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
};

TEST_P(DecoderResampleTest, OddResolution) {
  EncodeSegment(kSample1, kQp1, 12, kBitdepth);
  DecodeResolution(12, 1);
}

TEST_P(DecoderResampleTest, Downscaling) {
  EncodeSegment(kSample1, kQp1, 16, kBitdepth);
  EncodeSegment(kSample2, kQp2, 24, kBitdepth);
  DecodeResolution(16);
}

TEST_P(DecoderResampleTest, DownscalingOdd1) {
  EncodeSegment(kSample1, kQp1, 12, kBitdepth);
  EncodeSegment(kSample2, kQp2, 24, kBitdepth);
  DecodeResolution(12);
}

TEST_P(DecoderResampleTest, DownscalingOdd2) {
  EncodeSegment(kSample1, kQp1, 16, kBitdepth);
  EncodeSegment(kSample2, kQp2, 22, kBitdepth);
  DecodeResolution(16);
}

TEST_P(DecoderResampleTest, DownscalingOddBoth) {
  EncodeSegment(kSample1, kQp1, 10, kBitdepth);
  EncodeSegment(kSample2, kQp2, 20, kBitdepth);
  DecodeResolution(10);
}

TEST_P(DecoderResampleTest, Upscaling) {
  EncodeSegment(kSample1, kQp1, 24, kBitdepth);
  EncodeSegment(kSample2, kQp2, 16, kBitdepth);
  DecodeResolution(24);
}

TEST_P(DecoderResampleTest, UpscalingOddBoth) {
  EncodeSegment(kSample1, kQp1, 20, kBitdepth);
  EncodeSegment(kSample2, kQp2, 14, kBitdepth);
  DecodeResolution(20);
}

TEST_P(DecoderResampleTest, Chroma420ToMono) {
  EncodeSegment(kSample1, kQp1, kPicSize, kBitdepth, xvc::ChromaFormat::k420);
  EncodeSegment(kSample2, kQp2, kPicSize, kBitdepth,
                xvc::ChromaFormat::kMonochrome);
  DecodeChromaFormat(XVC_DEC_CHROMA_FORMAT_420);
}

TEST_P(DecoderResampleTest, ChromaMonoTo420) {
  EncodeSegment(kSample1, kQp1, kPicSize, kBitdepth,
                xvc::ChromaFormat::kMonochrome);
  EncodeSegment(kSample2, kQp2, kPicSize, kBitdepth, xvc::ChromaFormat::k420);
  DecodeChromaFormat(XVC_DEC_CHROMA_FORMAT_MONOCHROME);
}

TEST_P(DecoderResampleTest, Chroma420To444) {
  EncodeSegment(kSample1, kQp1, kPicSize, kBitdepth, xvc::ChromaFormat::k420);
  EncodeSegment(kSample2, kQp2, kPicSize, kBitdepth, xvc::ChromaFormat::k444);
  DecodeChromaFormat(XVC_DEC_CHROMA_FORMAT_420);
}

TEST_P(DecoderResampleTest, Chroma422To444) {
  EncodeSegment(kSample1, kQp1, kPicSize, kBitdepth, xvc::ChromaFormat::k422);
  EncodeSegment(kSample2, kQp2, kPicSize, kBitdepth, xvc::ChromaFormat::k444);
  DecodeChromaFormat(XVC_DEC_CHROMA_FORMAT_422);
}

TEST_P(DecoderResampleTest, Chroma444To420) {
  EncodeSegment(kSample1, kQp1, kPicSize, kBitdepth, xvc::ChromaFormat::k444);
  EncodeSegment(kSample2, kQp2, kPicSize, kBitdepth, xvc::ChromaFormat::k420);
  DecodeChromaFormat(XVC_DEC_CHROMA_FORMAT_444);
}

TEST_P(DecoderResampleTest, Chroma444To422) {
  EncodeSegment(kSample1, kQp1, kPicSize, kBitdepth, xvc::ChromaFormat::k444);
  EncodeSegment(kSample2, kQp2, kPicSize, kBitdepth, xvc::ChromaFormat::k422);
  DecodeChromaFormat(XVC_DEC_CHROMA_FORMAT_444);
}

#if XVC_HIGH_BITDEPTH
TEST_P(DecoderResampleTest, BitdepthUpConversionLowToHigh) {
  EncodeSegment(kSample1, kQp1, kPicSize, 8);
  EncodeSegment(kSample2, kQp2, kPicSize, 10);
  DecodeBitdepth(kPicSize, 8, 10, 8);
}

TEST_P(DecoderResampleTest, BitdepthUpConversionHighToHigh) {
  EncodeSegment(kSample1, kQp1, kPicSize, 10);
  EncodeSegment(kSample2, kQp2, kPicSize, 12);
  DecodeBitdepth(kPicSize, 10, 12, 10);
}

TEST_P(DecoderResampleTest, BitdepthDownConversionHighToLow) {
  EncodeSegment(kSample1, kQp1, kPicSize, 10);
  EncodeSegment(kSample2, kQp2, kPicSize, 8);
  DecodeBitdepth(kPicSize, 10, 8, 10);
}

TEST_P(DecoderResampleTest, BitdepthDownConversionHighToHigh) {
  EncodeSegment(kSample1, kQp1, kPicSize, 12);
  EncodeSegment(kSample2, kQp2, kPicSize, 10);
  DecodeBitdepth(kPicSize, 12, 10, 12);
}
#endif

INSTANTIATE_TEST_CASE_P(TestParam, DecoderResampleTest,
                        ::testing::Values(TestParam({ false, false }),
                                          TestParam({ true, false }),
                                          TestParam({ false, true }),
                                          TestParam({ true, true })));

}   // namespace
