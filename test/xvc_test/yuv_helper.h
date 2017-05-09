/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_TEST_YUV_HELPER_H_
#define XVC_TEST_YUV_HELPER_H_

#include <array>
#include <cassert>
#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_common_lib/common.h"

namespace xvc_test {

class TestYuvPic {
public:
  using Sample = ::xvc::Sample;
  static const int kDefaultSize = 32;

  TestYuvPic(int width, int height, int bitdepth, int dx = 0, int dy = 0,
             xvc::ChromaFormat chroma_fmt = xvc::ChromaFormat::k420);
  const std::vector<uint8_t> GetBytes() const { return bytes_; }
  int GetBitdepth() const { return bitdepth_; }
  double CalcPsnr(const char *bytes) const;

  static ::testing::AssertionResult
    AllSampleEqualTo(int width, int height, int bitdepth,
                     const char* pic_bytes, size_t size,
                     xvc::Sample expected_sample);

private:
  static const int kInternalPicSize = 40;
  static const int kInternalBitdepth = 10;
  static const int kInternalBufferSize =
    3 * kInternalPicSize * kInternalPicSize / 2;
  static std::array<uint16_t, kInternalBufferSize> kTestSamples;

  int width_;
  int height_;
  int bitdepth_;
  std::vector<Sample> samples_;
  std::vector<uint8_t> bytes_;
};

}   // namespace xvc_test

#endif  // XVC_TEST_YUV_HELPER_H_
