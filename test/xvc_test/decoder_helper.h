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

#ifndef XVC_TEST_DECODER_HELPER_H_
#define XVC_TEST_DECODER_HELPER_H_

#include <memory>
#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_dec_lib/decoder.h"

namespace xvc_test {

using NalUnit = std::vector<uint8_t>;

class DecoderHelper {
public:
  void Init(bool use_threads = false) {
    const int num_threads = use_threads ? -1 : 0;
    decoder_ = std::unique_ptr<xvc::Decoder>(new ::xvc::Decoder(num_threads));
  }

  void DecodeSegmentHeaderSuccess(const NalUnit &nal) {
    xvc::PicNum before_num_decoded_pics = decoder_->GetNumDecodedPics();
    EXPECT_TRUE(decoder_->DecodeNal(&nal[0], nal.size()));
    EXPECT_EQ(::xvc::Decoder::State::kSegmentHeaderDecoded,
              decoder_->GetState());
    EXPECT_EQ(before_num_decoded_pics, decoder_->GetNumDecodedPics());
    EXPECT_EQ(0, decoder_->GetNumCorruptedPics());
    EXPECT_FALSE(decoder_->GetDecodedPicture(&last_decoded_picture_));
    EXPECT_EQ(nullptr, last_decoded_picture_.bytes);
    EXPECT_EQ(0, last_decoded_picture_.size);
  }

  void DecodeSegmentHeaderFailed(const NalUnit &nal) {
    xvc::PicNum before_num_decoded_pics = decoder_->GetNumDecodedPics();
    EXPECT_FALSE(decoder_->DecodeNal(&nal[0], nal.size()));
    EXPECT_EQ(before_num_decoded_pics, decoder_->GetNumDecodedPics());
    EXPECT_FALSE(decoder_->GetDecodedPicture(&last_decoded_picture_));
    EXPECT_EQ(nullptr, last_decoded_picture_.bytes);
    EXPECT_EQ(0, last_decoded_picture_.size);
  }

  bool DecodePictureSuccess(const NalUnit &nal, int64_t user_data = 0) {
    xvc::PicNum before_num_decoded_pics = decoder_->GetNumDecodedPics();
    EXPECT_TRUE(decoder_->DecodeNal(&nal[0], nal.size(), user_data));
    EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
    EXPECT_EQ(before_num_decoded_pics + 1, decoder_->GetNumDecodedPics());
    EXPECT_EQ(0, decoder_->GetNumCorruptedPics());
    return decoder_->GetDecodedPicture(&last_decoded_picture_);
  }

  void DecodePictureFailed(const NalUnit &nal) {
    xvc::PicNum before_num_decoded_pics = decoder_->GetNumDecodedPics();
    EXPECT_FALSE(decoder_->DecodeNal(&nal[0], nal.size()));
    EXPECT_EQ(before_num_decoded_pics, decoder_->GetNumDecodedPics());
    EXPECT_FALSE(decoder_->GetDecodedPicture(&last_decoded_picture_));
    EXPECT_EQ(0, last_decoded_picture_.size);
  }

  bool DecoderFlushAndGet() {
    decoder_->FlushBufferedNalUnits();
    return decoder_->GetDecodedPicture(&last_decoded_picture_);
  }

  xvc_decoded_picture* DecodeAndFlush(const NalUnit &nal) {
    if (DecodePictureSuccess(nal)) {
      return &last_decoded_picture_;
    }
    if (DecoderFlushAndGet()) {
      return &last_decoded_picture_;
    }
    return nullptr;
  }

  static
    void AssertValidPicture420(int width, int height,
                               const xvc_decoded_picture &decoded_picture) {
    const int byte_width =
      width * (decoded_picture.stats.bitdepth == 8 ? 1 : 2);
    ASSERT_EQ(width, decoded_picture.stats.width);
    ASSERT_EQ(height, decoded_picture.stats.height);
    ASSERT_EQ(decoded_picture.planes[0], decoded_picture.bytes);
    ASSERT_EQ(decoded_picture.planes[1], decoded_picture.planes[0] +
              decoded_picture.stride[0] * height);
    ASSERT_EQ(decoded_picture.planes[2], decoded_picture.planes[1] +
              decoded_picture.stride[1] * height / 2);
    ASSERT_EQ(byte_width, decoded_picture.stride[0]);
    ASSERT_EQ(byte_width / 2, decoded_picture.stride[1]);
    ASSERT_EQ(byte_width / 2, decoded_picture.stride[2]);
  }

protected:
  std::unique_ptr<xvc::Decoder> decoder_;
  xvc_decoded_picture last_decoded_picture_;
};

}   // namespace xvc_test

#endif  // XVC_TEST_DECODER_HELPER_H_
