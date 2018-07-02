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

#include <array>
#include <cassert>
#include <cmath>
#include <memory>

#include "googletest/include/gtest/gtest.h"

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/transform.h"
#include "xvc_test/yuv_helper.h"

namespace {

static const int kMaxCuSize = xvc::constants::kMaxBlockSize;

static const std::array<int, 4> kFullTransformSizes = { 4, 8, 16, 32 };
static const std::array<int, 1> kZeroOutTransformSizes = { 64 };
static const std::array<int, 5> kAllTransformSizes = { 4, 8, 16, 32, 64 };
static_assert(xvc::constants::kTransformZeroOutMinSize == 32,
              "Test assumes that full 64x64 tx is not used");
// Depending on if zero out happens in first or second transform
static const std::array<double, 7> kZeroOutThresholdWidth = {
  1000, 1000, 35.0, 29.5, 28.0, 29.0, 27.0    // 1, 2, 4, 8, 16, 32, 64
};
static const std::array<double, 7> kZeroOutThresholdHeight = {
  1000, 1000, 30.5, 29.5, 30.0, 30.0, 27.0    // 1, 2, 4, 8, 16, 32, 64
};
static_assert(xvc::constants::kMaxBlockSize <= 64,
              "Only upto 64x64 transform tested");

class TransformTest : public ::testing::TestWithParam<int> {
protected:
  void SetUp() override {
    bitdepth_ = GetParam();
  }

  void TestAllTransformTypes(int width, int height,
                             const xvc::ResidualBuffer &resi_input,
                             double error_threshold) {
    xvc::PictureData pic_data(chroma_format, width, height, bitdepth_);
    xvc::CodingUnit *cu = pic_data.CreateCu(cu_tree, 0, 0, 0, width, height);
    xvc::ResidualBufferStorage resi_tmp(width, height);
    xvc::CoeffBufferStorage coeff_buffer(width, height);

    for (int tx1 = 0; tx1 < xvc::kNbrTransformTypes; tx1++) {
      for (int tx2 = 0; tx2 < xvc::kNbrTransformTypes; tx2++) {
        cu->SetTransformType(comp, static_cast<xvc::TransformType>(tx1),
                             static_cast<xvc::TransformType>(tx2));
        xvc::ForwardTransform fwd_transform(bitdepth_);
        fwd_transform.Transform(*cu, comp, resi_input, &coeff_buffer);

        xvc::InverseTransform inv_transform(bitdepth_);
        inv_transform.Transform(*cu, comp, coeff_buffer, &resi_tmp);

        if (error_threshold == 0) {
          VerifyEqual(width, height, tx1, tx2, resi_input, resi_tmp);
        } else {
          VerifyPsnr(width, height, tx1, tx2, resi_input, resi_tmp,
                     error_threshold);
        }
      }
    }

    pic_data.ReleaseCu(cu);
  }

  void VerifyEqual(int width, int height, int tx_type1, int tx_type2,
                   const xvc::ResidualBuffer &resi_before,
                   const xvc::ResidualBuffer &resi_after) {
    const xvc::Residual *before = resi_before.GetDataPtr();
    const xvc::Residual *after = resi_after.GetDataPtr();
    const int err_threshold = 1 << (bitdepth_ - 8);
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int abs_err = std::abs(before[x] - after[x]);
        ASSERT_LE(abs_err, err_threshold) <<
          " tx1=" << tx_type1 << " tx2=" << tx_type2 <<
          " x=" << x << " y=" << y << " w=" << width << " h=" << height;
      }
      before += resi_before.GetStride();
      after += resi_after.GetStride();
    }
  }

  void VerifyPsnr(int width, int height, int tx_type1, int tx_type2,
                  const xvc::ResidualBuffer &resi_before,
                  const xvc::ResidualBuffer &resi_after,
                  double psnr_threshold) {
    const xvc::Residual *before = resi_before.GetDataPtr();
    const xvc::Residual *after = resi_after.GetDataPtr();
    int sum = 0;
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        int err = before[x] - after[x];
        sum += err*err;
      }
      before += resi_before.GetStride();
      after += resi_after.GetStride();
    }
    const int64_t max_val = (1 << bitdepth_) - 1;
    const int64_t max_sum = max_val * max_val * width * height;
    double psnr = sum > 0 ? 10.0 * std::log10(max_sum / sum) : 1000;
    ASSERT_GE(psnr, psnr_threshold) <<
      " tx1=" << tx_type1 << " tx2=" << tx_type2 <<
      " w=" << width << " h=" << height;
  }

  void GenerateDcPred(int width, int height, xvc::Sample val,
                      xvc::SampleBuffer *pred_buffer) const {
    xvc::Sample *pred_ptr = pred_buffer->GetDataPtr();
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        pred_ptr[x] = val;
      }
      pred_ptr += pred_buffer->GetStride();
    }
  }

  const xvc::YuvComponent comp = xvc::YuvComponent::kY;
  const xvc::CuTree cu_tree = xvc::CuTree::Primary;
  const xvc::PicturePredictionType pic_type = xvc::PicturePredictionType::kBi;
  const xvc::ChromaFormat chroma_format = xvc::ChromaFormat::k420;
  std::unique_ptr<xvc::ResidualBufferStorage> resi_input_;
  std::unique_ptr<xvc::ResidualBufferStorage> resi_tmp_;
  std::unique_ptr<xvc::CoeffBufferStorage> coef_buffer_;

  int bitdepth_;
};

TEST_P(TransformTest, PerfectTxDcPred) {
  for (int height : kFullTransformSizes) {
    for (int width : kFullTransformSizes) {
      xvc_test::TestYuvPic orig_yuv(width, height, bitdepth_);
      auto orig_buffer = orig_yuv.GetSampleBuffer();
      xvc::SampleBufferStorage pred_buffer(width, height);
      GenerateDcPred(width, height, orig_yuv.GetAverageSample(), &pred_buffer);

      xvc::ResidualBufferStorage resi_input(width, height);
      resi_input.Subtract(width, height, orig_buffer, pred_buffer);
      TestAllTransformTypes(width, height, resi_input, 0);
    }
  }
}

TEST_P(TransformTest, ZeroOutWidthWithDcPred) {
  static_assert(kMaxCuSize >= xvc_test::TestYuvPic::kInternalPicSize,
                "Cannot generate orig sample data for lage CU sizes");
  xvc::SampleBufferStorage orig_buffer(kMaxCuSize, kMaxCuSize);
  xvc_test::TestYuvPic orig_yuv(xvc_test::TestYuvPic::kInternalPicSize,
                                xvc_test::TestYuvPic::kInternalPicSize,
                                bitdepth_);
  orig_yuv.FillLargerBuffer(kMaxCuSize, kMaxCuSize, &orig_buffer);
  for (int height : kZeroOutTransformSizes) {
    for (int width : kAllTransformSizes) {
      double psnr_min = kZeroOutThresholdWidth[xvc::util::SizeToLog2(width)];
      xvc::SampleBufferStorage pred_buffer(width, height);
      GenerateDcPred(width, height, orig_yuv.GetAverageSample(), &pred_buffer);

      xvc::ResidualBufferStorage resi_input(width, height);
      resi_input.Subtract(width, height, orig_buffer, pred_buffer);
      TestAllTransformTypes(width, height, resi_input, psnr_min);
    }
  }
}

TEST_P(TransformTest, ZeroOutHeightWithDcPred) {
  static_assert(kMaxCuSize >= xvc_test::TestYuvPic::kInternalPicSize,
                "Cannot generate orig sample data for lage CU sizes");
  xvc::SampleBufferStorage orig_buffer(kMaxCuSize, kMaxCuSize);
  xvc_test::TestYuvPic orig_yuv(xvc_test::TestYuvPic::kInternalPicSize,
                                xvc_test::TestYuvPic::kInternalPicSize,
                                bitdepth_);
  orig_yuv.FillLargerBuffer(kMaxCuSize, kMaxCuSize, &orig_buffer);
  for (int height : kAllTransformSizes) {
    for (int width : kZeroOutTransformSizes) {
      double psnr_min = kZeroOutThresholdHeight[xvc::util::SizeToLog2(height)];
      xvc::SampleBufferStorage pred_buffer(width, height);
      GenerateDcPred(width, height, orig_yuv.GetAverageSample(), &pred_buffer);

      xvc::ResidualBufferStorage resi_input(width, height);
      resi_input.Subtract(width, height, orig_buffer, pred_buffer);
      TestAllTransformTypes(width, height, resi_input, psnr_min);
    }
  }
}

INSTANTIATE_TEST_CASE_P(NormalBitdepth, TransformTest,
                        ::testing::Values(8));
#if XVC_HIGH_BITDEPTH
INSTANTIATE_TEST_CASE_P(HighBitdepth, TransformTest,
                        ::testing::Values(10, 12));
#endif

}   // namespace
