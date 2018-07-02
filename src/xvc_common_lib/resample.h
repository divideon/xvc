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

#include <vector>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/yuv_pic.h"

namespace xvc {

class Resampler {
public:
  Resampler() {}
  void ConvertFrom(const PictureFormat &src_format, const uint8_t *src_bytes,
                   YuvPicture *out_picm);
  void ConvertTo(const YuvPicture &src_pic, const PictureFormat &output_format,
                 std::vector<uint8_t> *out_bytes);

private:
  static const int kColorConversionBitdepth = 12;
  void CopyFromBytesFast(const uint8_t *src_bytes, int input_bitdepth,
                         YuvPicture *out_pic) const;
  void CopyFromBytesWithPadding(const uint8_t *src_bytes,
                                const PictureFormat &src_format,
                                YuvPicture *out_pic) const;
  void CopyFromBytesWithResampling(const uint8_t *src_bytes,
                                   const PictureFormat &src_format,
                                   YuvPicture *out_pic) const;
  uint8_t* CopyToBytesWithShift(YuvComponent comp, const YuvPicture &src_pic,
                                int out_bitdepth, int dither,
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

  std::vector<uint8_t> tmp_bytes_;
};

// TODO(PH) Refactor into class above

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
