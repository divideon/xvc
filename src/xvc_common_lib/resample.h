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
