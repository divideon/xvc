/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include <array>
#include <memory>
#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_dec_lib/picture_decoder.h"
#include "xvc_enc_lib/picture_encoder.h"

namespace {

class ChecksumEncDecTest : public ::testing::TestWithParam<int> {
protected:
  void SetUp() override {
    internal_bitdepth_ = GetParam();
    input_bitdepth_ = internal_bitdepth_;
    int mask = (1 << input_bitdepth_) - 1;
    for (int i = 0; i < static_cast<int>(input_pic_.size()); i++) {
      input_pic_[i] = i & mask;  // random yuv file
    }
    pic_encoder_ =
      std::make_shared<xvc::PictureEncoder>(chroma_format_, pic_width_,
                                            pic_height_, internal_bitdepth_);
    pic_decoder_ =
      std::make_shared<xvc::PictureDecoder>(chroma_format_, pic_width_,
                                            pic_height_, internal_bitdepth_);
  }

  std::vector<uint8_t>* EncodePicture(xvc::Sample *orig) {
    xvc::PicNum sub_gop_length = 1;
    int buffer_flag = 0;
    bool flat_lamda = false;
    pic_encoder_->GetOrigPic()->CopyFrom(reinterpret_cast<uint8_t*>(orig),
                                         input_bitdepth_);
    pic_encoder_->GetPicData()->SetNalType(xvc::NalUnitType::kIntraPicture);
    return pic_encoder_->Encode(base_qp_, sub_gop_length, buffer_flag,
                                flat_lamda);
  }

  bool DecodePicture(const std::vector<uint8_t> &bitstream) {
    xvc::PicNum sub_gop_end_poc, sub_gop_start_poc, sub_gop_length;
    int num_buffered_nals = 0;
    xvc::SegmentNum soc = 0;
    xvc::BitReader bit_reader(&bitstream[0], bitstream.size());
    pic_decoder_->DecodeHeader(&bit_reader, &sub_gop_end_poc,
                               &sub_gop_start_poc, &sub_gop_length, soc,
                               num_buffered_nals);
    return pic_decoder_->Decode(&bit_reader, sub_gop_length);
  }

  xvc::ChromaFormat chroma_format_ = xvc::ChromaFormat::k420;
  int pic_width_ = 16;
  int pic_height_ = 16;
  int internal_bitdepth_ = 8;
  int base_qp_ = 27;
  int input_bitdepth_ = 8;
  std::array<xvc::Sample, 16 * 16 * 3> input_pic_;
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
  for (int i = 0; i < pic_width_; i++) {
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
