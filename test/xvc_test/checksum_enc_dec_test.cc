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

#include <array>
#include <memory>
#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_common_lib/segment_header.h"
#include "xvc_common_lib/simd_functions.h"
#include "xvc_dec_lib/picture_decoder.h"
#include "xvc_enc_lib/picture_encoder.h"

namespace {

struct TestParam {
  int bitdepth;
  bool robust_checksum;
};

class ChecksumEncDecTest : public ::testing::TestWithParam<TestParam> {
protected:
  xvc::SegmentHeader SegmentHeaderHelper(int width, int height, int bitdepth,
                                         xvc::Checksum::Mode checksum_mode,
                                         xvc::ChromaFormat chroma_fmt,
                                         xvc::PicNum sub_gop_length) {
    xvc::SegmentHeader sh;
    sh.codec_identifier = xvc::constants::kXvcCodecIdentifier;
    sh.major_version = xvc::constants::kXvcMajorVersion;
    sh.minor_version = xvc::constants::kXvcMinorVersion;
    sh.soc = 0;
    sh.SetWidth(width);
    sh.SetHeight(height);
    sh.chroma_format = chroma_fmt;
    sh.internal_bitdepth = bitdepth;
    sh.max_sub_gop_length = sub_gop_length;
    sh.open_gop = true;
    sh.num_ref_pics = 1;
    sh.max_binary_split_depth = xvc::constants::kMaxBinarySplitDepth;
    sh.checksum_mode = checksum_mode;
    sh.deblock = true;
    return sh;
  }

  void SetUp() override {
    const xvc::PicNum sub_gop_length = 1;
    int internal_bitdepth = GetParam().bitdepth;
    xvc::Checksum::Mode checksum_mode = GetParam().robust_checksum ?
      xvc::Checksum::Mode::kMaxRobust : xvc::Checksum::Mode::kMinOverhead;
    segment_ = SegmentHeaderHelper(kPicWidth, kPicHeight, internal_bitdepth,
                                   checksum_mode, xvc::ChromaFormat::k420,
                                   sub_gop_length);
    output_pic_format_ =
      xvc::PictureFormat(kPicWidth, kPicHeight, input_bitdepth_,
                         segment_.chroma_format, segment_.color_matrix, false);
    input_bitdepth_ = segment_.internal_bitdepth;
    int mask = (1 << input_bitdepth_) - 1;
    for (int i = 0; i < static_cast<int>(input_pic_.size()); i++) {
      input_pic_[i] = i & mask;  // random yuv file
    }
    xvc::PictureFormat internal_pic_format = segment_.GetInternalPicFormat();
    std::set<xvc::CpuCapability> caps = xvc::SimdCpu::GetRuntimeCapabilities();
    simd_.reset(new xvc::EncoderSimdFunctions(caps, internal_bitdepth));
    pic_encoder_ =
      std::make_shared<xvc::PictureEncoder>(*simd_, internal_pic_format,
                                            segment_.GetCropWidth(),
                                            segment_.GetCropHeight());
    pic_decoder_ =
      std::make_shared<xvc::PictureDecoder>(*simd_, internal_pic_format,
                                            segment_.GetCropWidth(),
                                            segment_.GetCropHeight());
  }

  std::vector<uint8_t>* EncodePicture(const xvc::Sample *orig) {
    int buffer_flag = 0;
    pic_encoder_->GetPicData()->SetNalType(xvc::NalUnitType::kIntraPicture);
    pic_encoder_->GetPicData()->SetPoc(0);
    pic_encoder_->GetPicData()->SetDoc(0);
    pic_encoder_->GetPicData()->SetTid(0);
    xvc::Resampler resampler;
    xvc::PictureFormat input_format(kPicWidth, kPicHeight, input_bitdepth_,
                                    segment_.chroma_format,
                                    segment_.color_matrix, false);
    resampler.ConvertFrom(input_format, reinterpret_cast<const uint8_t*>(orig),
                          pic_encoder_->GetOrigPic().get());

    xvc::EncoderSettings encoder_settings;
    encoder_settings.Initialize(xvc::SpeedMode::kSlow);
    encoder_settings.Tune(xvc::TuneMode::kPsnr);
    return pic_encoder_->Encode(segment_, segment_qp_, buffer_flag,
                                encoder_settings);
  }

  bool DecodePicture(const std::vector<uint8_t> &bitstream) {
    xvc::PicNum sub_gop_end_poc = 0, sub_gop_start_poc = 0;
    xvc::PicNum sub_gop_length = segment_.max_sub_gop_length;
    int num_buffered_nals = 0;
    int doc = 0;
    xvc::BitReader bit_reader(&bitstream[0], bitstream.size());
    pic_decoder_->SetOutputStatus(xvc::OutputStatus::kHasBeenOutput);
    auto pic_header =
      pic_decoder_->DecodeHeader(&bit_reader, &sub_gop_end_poc,
                                 &sub_gop_start_poc, &sub_gop_length,
                                 sub_gop_length, sub_gop_length, doc,
                                 segment_.soc, num_buffered_nals,
                                 segment_.leading_pictures);
    // TODO(PH) Also verify inter pictures?
    xvc::ReferencePictureLists ref_pic_list;
    pic_decoder_->Init(segment_, pic_header, std::move(ref_pic_list),
                       output_pic_format_, 0);
    return pic_decoder_->Decode(segment_, &bit_reader, true);
  }

  static const int kPicWidth = 16;
  static const int kPicHeight = 16;
  static const int segment_qp_ = 27;
  int input_bitdepth_ = 8;
  xvc::SegmentHeader segment_;
  xvc::PictureFormat output_pic_format_;
  std::array<xvc::Sample, kPicWidth * kPicHeight * 3> input_pic_;
  std::unique_ptr<xvc::EncoderSimdFunctions> simd_;
  std::shared_ptr<xvc::PictureEncoder> pic_encoder_;
  std::shared_ptr<xvc::PictureDecoder> pic_decoder_;
};

TEST_P(ChecksumEncDecTest, CodingSamePictureGivesIdenticalChecksum) {
  std::vector<uint8_t> bitstream1(*EncodePicture(&input_pic_[0]));
  std::vector<uint8_t> enc_checksum1 = pic_encoder_->GetLastChecksum();
  std::vector<uint8_t> bitstream2(*EncodePicture(&input_pic_[0]));
  std::vector<uint8_t> enc_checksum2 = pic_encoder_->GetLastChecksum();
  ASSERT_EQ(enc_checksum1, enc_checksum2);

  ASSERT_TRUE(DecodePicture(bitstream1));
  ASSERT_EQ(enc_checksum1, pic_decoder_->GetLastChecksum());
  ASSERT_TRUE(DecodePicture(bitstream2));
  ASSERT_EQ(enc_checksum2, pic_decoder_->GetLastChecksum());
}

TEST_P(ChecksumEncDecTest, DifferentPictureGivesDifferentChecksum) {
  std::vector<uint8_t> bitstream1(*EncodePicture(&input_pic_[0]));
  std::vector<uint8_t> enc_checksum1 = pic_encoder_->GetLastChecksum();

  std::vector<xvc::Sample> orig2(input_pic_.begin(), input_pic_.end());
  for (int i = 0; i < segment_.GetInternalWidth(); i++) {
    orig2[i] += 10 << (GetParam().bitdepth - 8);   // random modification
  }
  std::vector<uint8_t> bitstream2(*EncodePicture(&orig2[0]));
  std::vector<uint8_t> enc_checksum2 = pic_encoder_->GetLastChecksum();
  ASSERT_NE(enc_checksum1, enc_checksum2);

  ASSERT_TRUE(DecodePicture(bitstream1));
  ASSERT_EQ(enc_checksum1, pic_decoder_->GetLastChecksum());
  ASSERT_TRUE(DecodePicture(bitstream2));
  ASSERT_EQ(enc_checksum2, pic_decoder_->GetLastChecksum());
}

TEST_P(ChecksumEncDecTest, MismatchingChecksumFailsDecode) {
  std::vector<uint8_t> bitstream1(*EncodePicture(&input_pic_[0]));
  std::vector<uint8_t> enc_checksum1 = pic_encoder_->GetLastChecksum();

  auto bitstream2 = bitstream1;
  auto bitstream2_it = bitstream2.end() - enc_checksum1.size();
  *bitstream2_it = (*bitstream2_it) - 1;
  ASSERT_FALSE(DecodePicture(bitstream2));

  auto bitstream3 = bitstream1;
  bitstream3[bitstream3.size() - 1]++;
  ASSERT_FALSE(DecodePicture(bitstream2));

  ASSERT_TRUE(DecodePicture(bitstream1));
  ASSERT_EQ(enc_checksum1, pic_decoder_->GetLastChecksum());
}

INSTANTIATE_TEST_CASE_P(NormalBitdepth, ChecksumEncDecTest,
                        ::testing::Values(TestParam({ 8, false }),
                                          TestParam({ 8, true })));
#if XVC_HIGH_BITDEPTH
INSTANTIATE_TEST_CASE_P(HighBitdepth, ChecksumEncDecTest,
                        ::testing::Values(TestParam({ 10, false }),
                                          TestParam({ 10, true })));
#endif

}   // namespace
