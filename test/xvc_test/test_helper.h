/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_TEST_TEST_HELPER_H_
#define XVC_TEST_TEST_HELPER_H_

#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_dec_lib/decoder.h"
#include "xvc_enc_lib/encoder.h"

namespace xvc_test {

class DecoderHelper {
public:
  virtual void SetUp() {
    decoder_ = new ::xvc::Decoder();
  }
  virtual void TearDown() {
    delete decoder_;
  }

  void DecodeSegmentHeader(const xvc_enc_nal_unit &nal) {
    EXPECT_TRUE(decoder_->DecodeNal(nal.bytes, nal.size));
    EXPECT_EQ(0, decoder_->GetNumDecodedPics());
    decoder_->GetDecodedPicture(&decoded_picture_);
    EXPECT_EQ(nullptr, decoded_picture_.bytes);
    EXPECT_EQ(0, decoded_picture_.size);
  }

  void DecodePictureSuccess(const xvc_enc_nal_unit &nal) {
    EXPECT_TRUE(decoder_->DecodeNal(nal.bytes, nal.size));
    EXPECT_EQ(1, decoder_->GetNumDecodedPics());
    decoder_->GetDecodedPicture(&decoded_picture_);
  }

  void DecodePictureFailed(const xvc_enc_nal_unit &nal) {
    EXPECT_FALSE(decoder_->DecodeNal(nal.bytes, nal.size));
    EXPECT_EQ(0, decoder_->GetNumDecodedPics());
    decoder_->GetDecodedPicture(&decoded_picture_);
    EXPECT_EQ(0, decoded_picture_.size);
  }

  ::xvc::Decoder *decoder_;
  xvc_decoded_picture decoded_picture_;
};

class EncoderHelper {
protected:
  virtual void SetUp() {
    encoder_ = new ::xvc::Encoder(8);
    encoder_->SetChromaFormat(XVC_ENC_CHROMA_FORMAT_420);
    encoder_->SetResolution(0, 0);
    encoder_->SetDeblock(1);
  }
  virtual void TearDown() {
    delete encoder_;
  }

  void EncodeFirstFrame(int width, int height) {
    size_t plane_samples = width * height;
    std::vector <uint8_t> picture_bytes(plane_samples *
                                        ::xvc::constants::kMaxYuvComponents);
    std::fill(picture_bytes.begin(), picture_bytes.end(), 0xff);
    uint8_t *in_pic = picture_bytes.empty() ? nullptr : &picture_bytes[0];
    xvc_enc_nal_unit *nal_units = nullptr;
    xvc_enc_pic_buffer rec;

    int num_nals = encoder_->Encode(in_pic, &nal_units, false, &rec);
    ASSERT_EQ(2, num_nals);
    EXPECT_EQ(static_cast<uint32_t>(xvc::NalUnitType::kSegmentHeader),
              nal_units[0].stats.nal_unit_type);
    EXPECT_EQ(static_cast<uint32_t>(xvc::NalUnitType::kIntraAccessPicture),
              nal_units[1].stats.nal_unit_type);
    nal_units_.insert(nal_units_.end(), nal_units, nal_units + num_nals);
  }

  ::xvc::Encoder *encoder_;
  std::vector<xvc_enc_nal_unit> nal_units_;
};

class EncoderDecoderHelper
  : public ::testing::Test, public EncoderHelper, public DecoderHelper {
protected:
  virtual void SetUp() {
    EncoderHelper::SetUp();
    DecoderHelper::SetUp();
  }
  virtual void TearDown() {
    EncoderHelper::TearDown();
    DecoderHelper::TearDown();
  }

  void DecodeFirstPictureSuccess() {
    ASSERT_EQ(2, nal_units_.size());
    DecodePictureSuccess(nal_units_[1]);
  }
  void DecodeFirstPictureFailed() {
    ASSERT_EQ(2, nal_units_.size());
    DecodePictureFailed(nal_units_[1]);
  }
};

}   // namespace xvc_test

#endif  // XVC_TEST_TEST_HELPER_H_
