/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include <array>
#include <memory>
#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_common_lib/segment_header.h"
#include "xvc_dec_lib/picture_decoder.h"
#include "xvc_enc_lib/picture_encoder.h"

namespace {

class ChecksumEncDecTest : public ::testing::TestWithParam<int> {
protected:
  xvc::SegmentHeader SegmentHeaderHelper(int width, int height, int bitdepth,
                                         xvc::ChromaFormat chroma_fmt,
                                         xvc::PicNum sub_gop_length) {
    xvc::SegmentHeader sh;
    sh.codec_identifier = xvc::constants::kXvcCodecIdentifier;
    sh.major_version = xvc::constants::kXvcMajorVersion;
    sh.minor_version = xvc::constants::kXvcMinorVersion;
    sh.soc = 0;
    sh.pic_width = width;
    sh.pic_height = height;
    sh.chroma_format = chroma_fmt;
    sh.internal_bitdepth = bitdepth;
    sh.max_sub_gop_length = sub_gop_length;
    sh.open_gop = true;
    sh.num_ref_pics = 1;
    sh.deblock = true;
    return sh;
  }

  void SetUp() override {
    const xvc::PicNum sub_gop_length = 1;
    int internal_bitdepth = GetParam();
    segment_ = SegmentHeaderHelper(kPicWidth, kPicHeight, internal_bitdepth,
                                   xvc::ChromaFormat::k420, sub_gop_length);
    input_bitdepth_ = segment_.internal_bitdepth;
    int mask = (1 << input_bitdepth_) - 1;
    for (int i = 0; i < static_cast<int>(input_pic_.size()); i++) {
      input_pic_[i] = i & mask;  // random yuv file
    }
    pic_encoder_ =
      std::make_shared<xvc::PictureEncoder>(segment_.chroma_format,
                                            segment_.pic_width,
                                            segment_.pic_height,
                                            segment_.internal_bitdepth);
    pic_decoder_ =
      std::make_shared<xvc::PictureDecoder>(segment_.chroma_format,
                                            segment_.pic_width,
                                            segment_.pic_height,
                                            segment_.internal_bitdepth);
  }

  std::vector<uint8_t>* EncodePicture(xvc::Sample *orig) {
    int buffer_flag = 0;
    bool flat_lamda = false;
    pic_encoder_->GetOrigPic()->CopyFrom(reinterpret_cast<uint8_t*>(orig),
                                         input_bitdepth_);
    pic_encoder_->GetPicData()->SetNalType(xvc::NalUnitType::kIntraPicture);
    return pic_encoder_->Encode(segment_, segment_qp_,
                                segment_.max_sub_gop_length, buffer_flag,
                                flat_lamda);
  }

  bool DecodePicture(const std::vector<uint8_t> &bitstream) {
    xvc::PicNum sub_gop_end_poc, sub_gop_start_poc;
    xvc::PicNum sub_gop_length = segment_.max_sub_gop_length;
    int num_buffered_nals = 0;
    xvc::BitReader bit_reader(&bitstream[0], bitstream.size());
    pic_decoder_->DecodeHeader(&bit_reader, &sub_gop_end_poc,
                               &sub_gop_start_poc, &sub_gop_length,
                               segment_.soc, num_buffered_nals);
    int max_tid = xvc::SegmentHeader::GetMaxTid(sub_gop_length);
    return pic_decoder_->Decode(&bit_reader, max_tid);
  }

  static const int kPicWidth = 16;
  static const int kPicHeight = 16;
  static const int segment_qp_ = 27;
  int input_bitdepth_ = 8;
  xvc::SegmentHeader segment_;
  std::array<xvc::Sample, kPicWidth * kPicHeight * 3> input_pic_;
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
  for (int i = 0; i < segment_.pic_width; i++) {
    orig2[i] += 10;   // random modification
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
                        ::testing::Values(8));
#if XVC_HIGH_BITDEPTH
INSTANTIATE_TEST_CASE_P(HighBitdepth, ChecksumEncDecTest,
                        ::testing::Values(10));
#endif

}   // namespace
