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
    size_t decoded_bytes = decoder_->DecodeNal(&nal[0], nal.size());
    EXPECT_EQ(nal.size(), decoded_bytes);
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
    size_t decoded_bytes = decoder_->DecodeNal(&nal[0], nal.size());
    EXPECT_EQ(size_t(xvc::Decoder::kInvalidNal), decoded_bytes);
    EXPECT_EQ(before_num_decoded_pics, decoder_->GetNumDecodedPics());
    EXPECT_FALSE(decoder_->GetDecodedPicture(&last_decoded_picture_));
    EXPECT_EQ(nullptr, last_decoded_picture_.bytes);
    EXPECT_EQ(0, last_decoded_picture_.size);
  }

  bool DecodePictureSuccess(const NalUnit &nal, int64_t user_data = 0) {
    xvc::PicNum before_num_decoded_pics = decoder_->GetNumDecodedPics();
    size_t decoded_bytes = decoder_->DecodeNal(&nal[0], nal.size(), user_data);
    EXPECT_EQ(nal.size(), decoded_bytes);
    EXPECT_EQ(::xvc::Decoder::State::kPicDecoded, decoder_->GetState());
    EXPECT_EQ(before_num_decoded_pics + 1, decoder_->GetNumDecodedPics());
    EXPECT_EQ(0, decoder_->GetNumCorruptedPics());
    return decoder_->GetDecodedPicture(&last_decoded_picture_);
  }

  void DecodePictureFailed(const NalUnit &nal) {
    xvc::PicNum before_num_decoded_pics = decoder_->GetNumDecodedPics();
    size_t decoded_bytes = decoder_->DecodeNal(&nal[0], nal.size());
    EXPECT_EQ(size_t(xvc::Decoder::kInvalidNal), decoded_bytes);
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
