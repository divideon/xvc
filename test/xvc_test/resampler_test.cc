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

#include <algorithm>
#include <list>
#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_common_lib/resample.h"
#include "xvc_test/yuv_helper.h"

namespace {

using xvc_test::TestYuvPic;

static const int kInputBitdepth = 8;
static const bool kRefPadding = false;
static const bool kOutputDither = false;

static const bool kModePadding = true;
static const bool kModeResampling = false;

class ResamplerTest : public ::testing::TestWithParam<int> {
protected:
  void SetUp() override {
    upshift = GetParam() - kInputBitdepth;
    fmt_40x40_yuv420p8 =
      xvc::PictureFormat(40, 40, kInputBitdepth, xvc::ChromaFormat::k420,
                         xvc::ColorMatrix::k601, kOutputDither);
    fmt_34x34_yuv420p8 =
      xvc::PictureFormat(34, 34, kInputBitdepth, xvc::ChromaFormat::k420,
                         xvc::ColorMatrix::k601, kOutputDither);
    fmt_40x40_rgba8 =
      xvc::PictureFormat(40, 40, kInputBitdepth, xvc::ChromaFormat::kArgb,
                         xvc::ColorMatrix::k601, kOutputDither);
    fmt_34x34_rgba8 =
      xvc::PictureFormat(34, 34, kInputBitdepth, xvc::ChromaFormat::kArgb,
                         xvc::ColorMatrix::k601, kOutputDither);
    fmt_40x40_internal =
      xvc::PictureFormat(40, 40, GetParam(), xvc::ChromaFormat::k420,
                         xvc::ColorMatrix::kUndefined, kOutputDither);
  }

  TestYuvPic CreateOrigPic(const xvc::PictureFormat &format) {
    return TestYuvPic(format.width, format.height, format.bitdepth, 0, 0,
                      format.chroma_format);
  }

  xvc::YuvPicture CreateYuvPic(const xvc::PictureFormat &format,
                               const xvc::PictureFormat *src_format) {
    int pad_width = src_format ? format.width - src_format->width : 0;
    int pad_height = src_format ? format.height - src_format->height : 0;
    return xvc::YuvPicture(format, kRefPadding, pad_width, pad_height);
  }

  std::vector<uint8_t>
    ConvertFromAndBack(bool use_sample_padding,
                       const std::vector<uint8_t> &input_bytes,
                       const xvc::PictureFormat &input_fmt,
                       const xvc::PictureFormat &internal_fmt,
                       const xvc::PictureFormat &output_fmt) {
    const int pad_width = use_sample_padding ?
      internal_fmt.width - input_fmt.width : 0;
    const int pad_height = use_sample_padding ?
      internal_fmt.height - input_fmt.height : 0;
    xvc::YuvPicture rec_pic(internal_fmt, true, pad_width, pad_height);
    resampler.ConvertFrom(input_fmt, &input_bytes[0], &rec_pic);
    rec_pic.PadBorder();    // required for resampling to work
    std::vector<uint8_t> output_bytes;
    resampler.ConvertTo(rec_pic, output_fmt, &output_bytes);
    return output_bytes;
  }

  std::vector<uint8_t> ConvertArgb601ToLuma(const xvc::PictureFormat &format,
                                            std::vector<uint8_t> src_bytes) {
    EXPECT_EQ(format.width * format.height * 4, src_bytes.size());
    EXPECT_EQ(8, format.bitdepth) << "only 8 bitdepth implemented in test";
    // Extract luma and compare original (lossy due to fixed point in lib)
    std::vector<uint8_t> luma_bytes(format.width * format.height);
    for (int i = 0; i < static_cast<int>(luma_bytes.size()); i++) {
      EXPECT_EQ(255, src_bytes[i * 4 + 3]) << "alpha";
      if (HasFailure())
        break;
      double luma = 16 +
        (0.257 * src_bytes[i * 4 + 0] +
         0.504 * src_bytes[i * 4 + 1] +
         0.098 * src_bytes[i * 4 + 2]);
      luma_bytes[i] = static_cast<uint8_t>(luma + 0.5);
    }
    return luma_bytes;
  }

  xvc::Resampler resampler;
  int upshift;
  xvc::PictureFormat fmt_40x40_yuv420p8;
  xvc::PictureFormat fmt_34x34_yuv420p8;
  xvc::PictureFormat fmt_40x40_rgba8;
  xvc::PictureFormat fmt_34x34_rgba8;
  xvc::PictureFormat fmt_40x40_internal;
};

TEST_P(ResamplerTest, FromBytesNormalResolutionWithCrop_40to40to40) {
  TestYuvPic orig_pic = CreateOrigPic(fmt_40x40_yuv420p8);
  xvc::YuvPicture dst_pic = CreateYuvPic(fmt_40x40_internal,
                                         &fmt_40x40_yuv420p8);
  resampler.ConvertFrom(fmt_40x40_yuv420p8, &orig_pic.GetBytes()[0], &dst_pic);
  ASSERT_TRUE(TestYuvPic::SameSamples(40, 40, orig_pic, upshift, dst_pic, 0));
}

TEST_P(ResamplerTest, FromBytesNormalResolutionWithResample_40to40to40) {
  TestYuvPic orig_pic = CreateOrigPic(fmt_40x40_yuv420p8);
  xvc::YuvPicture dst_pic = CreateYuvPic(fmt_40x40_internal, nullptr);
  resampler.ConvertFrom(fmt_40x40_yuv420p8, &orig_pic.GetBytes()[0], &dst_pic);
  ASSERT_TRUE(TestYuvPic::SameSamples(40, 40, orig_pic, upshift, dst_pic, 0));
}

TEST_P(ResamplerTest, FromBytesOddResolutionWithPadding_34to40to34) {
  TestYuvPic orig_pic = CreateOrigPic(fmt_34x34_yuv420p8);
  xvc::YuvPicture dst_pic = CreateYuvPic(fmt_40x40_internal,
                                         &fmt_34x34_yuv420p8);
  resampler.ConvertFrom(fmt_34x34_yuv420p8, &orig_pic.GetBytes()[0], &dst_pic);
  ASSERT_TRUE(TestYuvPic::SameSamples(34, 34, orig_pic, upshift, dst_pic, 0));
}

TEST_P(ResamplerTest, FromBytesOddResolutionWithResample_34to40to34) {
  TestYuvPic orig_pic = CreateOrigPic(fmt_34x34_yuv420p8);
  xvc::YuvPicture dst_pic = CreateYuvPic(fmt_40x40_internal, nullptr);
  resampler.ConvertFrom(fmt_34x34_yuv420p8, &orig_pic.GetBytes()[0], &dst_pic);
  // TODO(PH) Calculating PSNR between pictures of different size...
  double psnr_y = orig_pic.CalcPsnr(xvc::YuvComponent::kY, dst_pic);
  ASSERT_GE(psnr_y, 25.0);
  ASSERT_LT(psnr_y, 30.0);
  ASSERT_GE(orig_pic.CalcPsnr(xvc::YuvComponent::kU, dst_pic), 40.0);
  ASSERT_GE(orig_pic.CalcPsnr(xvc::YuvComponent::kV, dst_pic), 40.0);
}

TEST_P(ResamplerTest, ToYuvBytesNormalResolutionWithCrop_40to40to40) {
  TestYuvPic orig_pic = CreateOrigPic(fmt_40x40_yuv420p8);
  std::vector<uint8_t> src_bytes = orig_pic.GetBytes();
  std::vector<uint8_t> out_bytes =
    ConvertFromAndBack(kModePadding, src_bytes, fmt_40x40_yuv420p8,
                       fmt_40x40_internal, fmt_40x40_yuv420p8);
  ASSERT_TRUE(TestYuvPic::SamePictureBytes(&src_bytes[0], src_bytes.size(),
                                           &out_bytes[0], out_bytes.size()));
}

TEST_P(ResamplerTest, ToYuvBytesNormalResolutionWithResample_40to40to40) {
  TestYuvPic orig_pic = CreateOrigPic(fmt_40x40_yuv420p8);
  std::vector<uint8_t> src_bytes = orig_pic.GetBytes();
  std::vector<uint8_t> out_bytes =
    ConvertFromAndBack(kModeResampling, src_bytes, fmt_40x40_yuv420p8,
                       fmt_40x40_internal, fmt_40x40_yuv420p8);
  ASSERT_TRUE(TestYuvPic::SamePictureBytes(&src_bytes[0], src_bytes.size(),
                                           &out_bytes[0], out_bytes.size()));
}

TEST_P(ResamplerTest, ToYuvBytesOddResolutionWithCrop_34to40to34) {
  TestYuvPic orig_pic = CreateOrigPic(fmt_34x34_yuv420p8);
  std::vector<uint8_t> src_bytes = orig_pic.GetBytes();
  std::vector<uint8_t> out_bytes =
    ConvertFromAndBack(kModePadding, src_bytes, fmt_34x34_yuv420p8,
                       fmt_40x40_internal, fmt_34x34_yuv420p8);
  ASSERT_TRUE(TestYuvPic::SamePictureBytes(&src_bytes[0], src_bytes.size(),
                                           &out_bytes[0], out_bytes.size()));
}

TEST_P(ResamplerTest, ToYuvBytesOddResolutionWithResample_34to40to34) {
  TestYuvPic orig_pic = CreateOrigPic(fmt_34x34_yuv420p8);
  std::vector<uint8_t> src_bytes = orig_pic.GetBytes();
  std::vector<uint8_t> out_bytes =
    ConvertFromAndBack(kModeResampling, src_bytes, fmt_34x34_yuv420p8,
                       fmt_40x40_internal, fmt_34x34_yuv420p8);
  double psnr_y = orig_pic.CalcPsnr(reinterpret_cast<char*>(&out_bytes[0]));
  ASSERT_GE(psnr_y, 40.0);
  ASSERT_LT(psnr_y, 45.0);
}

TEST_P(ResamplerTest, ToRgbvBytesNormalResolutionWithCrop_40to40to40) {
  TestYuvPic orig_pic = CreateOrigPic(fmt_40x40_yuv420p8);
  std::vector<uint8_t> out_rgba_bytes =
    ConvertFromAndBack(kModePadding, orig_pic.GetBytes(), fmt_40x40_yuv420p8,
                       fmt_40x40_internal, fmt_40x40_rgba8);
  std::vector<uint8_t> rec_luma =
    ConvertArgb601ToLuma(fmt_40x40_rgba8, out_rgba_bytes);
  double psnr_y = orig_pic.CalcPsnr(reinterpret_cast<char*>(&rec_luma[0]));
  ASSERT_GE(psnr_y, 50.0);
  ASSERT_LT(psnr_y, 55.0);
}

TEST_P(ResamplerTest, ToRgbBytesNormalResolutionWithResample_40to40to40) {
  TestYuvPic orig_pic = CreateOrigPic(fmt_40x40_yuv420p8);
  std::vector<uint8_t> out_rgba_bytes =
    ConvertFromAndBack(kModeResampling, orig_pic.GetBytes(), fmt_40x40_yuv420p8,
                       fmt_40x40_internal, fmt_40x40_rgba8);
  std::vector<uint8_t> rec_luma =
    ConvertArgb601ToLuma(fmt_40x40_rgba8, out_rgba_bytes);
  double psnr_y = orig_pic.CalcPsnr(reinterpret_cast<char*>(&rec_luma[0]));
  ASSERT_GE(psnr_y, 50.0);
  ASSERT_LT(psnr_y, 55.0);
}

TEST_P(ResamplerTest, ToRgbBytesOddResolutionWithCrop_34to40to34) {
  TestYuvPic orig_pic = CreateOrigPic(fmt_34x34_yuv420p8);
  std::vector<uint8_t> out_rgba_bytes =
    ConvertFromAndBack(kModePadding, orig_pic.GetBytes(), fmt_34x34_yuv420p8,
                       fmt_40x40_internal, fmt_34x34_rgba8);
  std::vector<uint8_t> rec_luma =
    ConvertArgb601ToLuma(fmt_34x34_rgba8, out_rgba_bytes);
  double psnr_y = orig_pic.CalcPsnr(reinterpret_cast<char*>(&rec_luma[0]));
  ASSERT_GE(psnr_y, 50.0);
  ASSERT_LT(psnr_y, 55.0);
}

TEST_P(ResamplerTest, ToRgbBytesOddResolutionWithResample_34to40to34) {
  TestYuvPic orig_pic = CreateOrigPic(fmt_34x34_yuv420p8);
  std::vector<uint8_t> out_rgba_bytes =
    ConvertFromAndBack(kModeResampling, orig_pic.GetBytes(), fmt_34x34_yuv420p8,
                       fmt_40x40_internal, fmt_34x34_rgba8);
  std::vector<uint8_t> rec_luma =
    ConvertArgb601ToLuma(fmt_34x34_rgba8, out_rgba_bytes);
  double psnr_y = orig_pic.CalcPsnr(reinterpret_cast<char*>(&rec_luma[0]));
  ASSERT_GE(psnr_y, 40.0);
  ASSERT_LT(psnr_y, 55.0);
}

INSTANTIATE_TEST_CASE_P(NormalBitdepth, ResamplerTest,
                        ::testing::Values(8));
#if XVC_HIGH_BITDEPTH
INSTANTIATE_TEST_CASE_P(HighBitdepth, ResamplerTest,
                        ::testing::Values(10));
#endif

}   // namespace
