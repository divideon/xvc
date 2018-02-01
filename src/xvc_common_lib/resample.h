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
  explicit Resampler(const PictureFormat &output_format)
    : out_fmt_(output_format) {
  }
  void Convert(const YuvPicture &src_pic,
               std::vector<uint8_t> *out_bytes);

private:
  static const int kColorConversionBitdepth = 12;
  uint8_t* CopyWithShift(uint8_t *out8, int width,
                         int height, ptrdiff_t stride, int out_bitdepth,
                         const Sample *src, int bitdepth, int dither) const;
  template <typename T>
  void ConvertColorSpace(uint8_t *dst, int width, int height,
                         const uint16_t *src, int bitdepth,
                         ColorMatrix color_matrix) const;
  void ConvertColorSpace8bit709(uint8_t *dst, int width, int height,
                                const uint16_t *src) const;

  PictureFormat out_fmt_;
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
