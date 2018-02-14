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

#ifndef XVC_TEST_YUV_HELPER_H_
#define XVC_TEST_YUV_HELPER_H_

#include <array>
#include <cassert>
#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/sample_buffer.h"
#include "xvc_common_lib/yuv_pic.h"

namespace xvc_test {

class TestYuvPic {
public:
  using Sample = ::xvc::Sample;
  static const int kDefaultSize = 32;
  static const int kInternalPicSize = 40;

  TestYuvPic(int width, int height, int bitdepth, int dx = 0, int dy = 0,
             xvc::ChromaFormat chroma_fmt = xvc::ChromaFormat::k420);
  const std::vector<uint8_t> GetBytes() const { return bytes_; }
  const Sample* GetSamplePtr(xvc::YuvComponent comp) const;
  xvc::SampleBufferConst GetSampleBuffer() const {
    return xvc::SampleBufferConst(&samples_[0], width_);
  }
  ptrdiff_t GetStride(xvc::YuvComponent comp) const {
    return stride_[static_cast<int>(comp)];
  }
  Sample GetAverageSample();
  int GetBitdepth() const { return bitdepth_; }
  double CalcPsnr(const char *bytes) const;
  double CalcPsnr(xvc::YuvComponent comp, const xvc::YuvPicture &ref_pic);
  void FillLargerBuffer(int out_width, int out_height, xvc::SampleBuffer *out);

  static ::testing::AssertionResult
    SameSamples(int src_width, int src_height,
                const xvc_test::TestYuvPic &orig_pic, int orig_upshift,
                const xvc::YuvPicture &ref_pic, int ref_upshift);
  static ::testing::AssertionResult
    SamePictureBytes(const uint8_t *pic1, size_t size1,
                     const uint8_t *pic2, size_t size2);
  static ::testing::AssertionResult
    AllSampleEqualTo(int width, int height, int bitdepth,
                     const char* pic_bytes, size_t size,
                     xvc::Sample expected_sample);

private:
  static const int kInternalBitdepth = 10;
  static const int kInternalBufferSize =
    3 * kInternalPicSize * kInternalPicSize / 2;
  static std::array<uint16_t, kInternalBufferSize> kTestSamples;

  int width_;
  int height_;
  int bitdepth_;
  xvc::ChromaFormat chroma_fmt_;
  std::array<ptrdiff_t, xvc::constants::kMaxYuvComponents> stride_;
  std::vector<Sample> samples_;
  std::vector<uint8_t> bytes_;
};

}   // namespace xvc_test

#endif  // XVC_TEST_YUV_HELPER_H_
