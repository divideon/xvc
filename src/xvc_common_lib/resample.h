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

#ifndef XVC_COMMON_LIB_RESAMPLE_H_
#define XVC_COMMON_LIB_RESAMPLE_H_

#include <utility>
#include <vector>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/yuv_pic.h"

namespace xvc {

class Resampler {
public:
  using PicPlane = std::pair<const uint8_t *, ptrdiff_t>;
  using PicPlanes = std::array<PicPlane, constants::kMaxYuvComponents>;
  struct SimdFunc;

  explicit Resampler(const SimdFunc &simd) : simd_(simd) {}
  void ConvertFrom(const PictureFormat &src_format, const uint8_t *src_bytes,
                   YuvPicture *out_pic);
  void ConvertFrom(const PictureFormat &src_format,
                   const PicPlanes &input_planes,
                   YuvPicture *out_pic);
  void ConvertTo(const YuvPicture &src_pic, const PictureFormat &output_format,
                 std::vector<uint8_t> *out_bytes);

private:
  static const int kColorConversionBitdepth = 12;
  const uint8_t*
    CopyFromBytesFast(YuvComponent comp, const uint8_t *input_bytes,
                      ptrdiff_t input_stride, int input_bitdepth,
                      YuvPicture *out_pic) const;
  const uint8_t *
    CopyFromBytesWithPadding(YuvComponent comp, const uint8_t *src_bytes,
                             ptrdiff_t src_stride,
                             const PictureFormat &src_format,
                             YuvPicture *out_pic) const;
  void CopyFromBytesWithResampling(const uint8_t *src_bytes,
                                   const PictureFormat &src_format,
                                   YuvPicture *out_pic) const;
  uint8_t* CopyToBytesWithShift(YuvComponent comp, const YuvPicture &src_pic,
                                int out_bitdepth, bool dither,
                                uint8_t *out8) const;
  void CopyToWithResize(const YuvPicture &src_pic,
                        const PictureFormat &output_format,
                        int dst_bitdepth, uint8_t *out8) const;
  template <typename T>
  void ConvertColorSpace(uint8_t *dst, int width, int height,
                         const uint16_t *src, int bitdepth,
                         ColorMatrix color_matrix) const;
  void ConvertColorSpace8bit709(uint8_t *dst, int width, int height,
                                const uint16_t *src) const;

  const SimdFunc &simd_;
  std::vector<uint8_t> tmp_bytes_;
};

struct Resampler::SimdFunc {
  static const int kDither = 2;  // 0=off 1=on
  SimdFunc();
  void(*copy_sample_byte)(int width, int height,
                          const Sample *src, ptrdiff_t src_stride,
                          uint8_t *out, ptrdiff_t out_stride);
  void(*copy_sample_short)(int width, int height,
                           const Sample *src, ptrdiff_t src_stride,
                           uint16_t *out, ptrdiff_t out_stride);
  void(*downshift_sample_byte[kDither])(int width, int height, int shift,
                                        int out_bitdepth,
                                        const Sample *src, ptrdiff_t src_stride,
                                        uint8_t *out, ptrdiff_t out_stride);
  void(*downshift_sample_short[kDither])(int width, int height, int shift,
                                         int out_bitdepth, const Sample *src,
                                         ptrdiff_t src_stride, uint16_t *out,
                                         ptrdiff_t out_stride);
  void(*upshift_sample_short)(int width, int height, int shift,
                              const Sample *src, ptrdiff_t src_stride,
                              uint16_t *out, ptrdiff_t out_stride);
};

// TODO(PH) Refactor into Resampler class

namespace resample {

template <typename T, typename U>
void Resample(uint8_t *dst_start, int dst_width, int dst_height,
              ptrdiff_t dst_stride, int dst_bitdepth,
              const uint8_t *src_start, int src_width, int src_height,
              ptrdiff_t src_stride, int src_bitdepth);

template <typename T, typename U>
void BilinearResample(uint8_t *dst_start, int dst_width, int dst_height,
                      ptrdiff_t dst_stride, int dst_bitdepth,
                      const uint8_t *src_start, int src_width, int src_height,
                      ptrdiff_t src_stride, int src_bitdepth);

}   // namespace resample

}   // namespace xvc

#endif  // XVC_COMMON_LIB_RESAMPLE_H_
