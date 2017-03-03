/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_test/test_helper.h"

namespace {

class EncodeDecodeTest : public ::xvc_test::EncoderDecoderHelper {
protected:
  void EncodeDecode(int width, int height) {
    EncodeFirstFrame(width, height);
    DecodeSegmentHeader(nal_units_[0]);
    DecodeFirstPictureSuccess();
  }
};

TEST_F(EncodeDecodeTest, SingleFrameEmptySize) {
  encoder_->SetResolution(0, 0);
  EncodeDecode(0, 0);
  ASSERT_EQ(0, decoded_picture_.size);
}

TEST_F(EncodeDecodeTest, SingleFrameSingleCU16x16) {
  encoder_->SetResolution(16, 16);
  encoder_->SetInputBitdepth(8);
  EncodeDecode(16, 16);
  ASSERT_EQ(16*16 + (16*16/2), decoded_picture_.size);
}

}   // namespace
