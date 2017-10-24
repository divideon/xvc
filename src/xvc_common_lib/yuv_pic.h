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

#ifndef XVC_COMMON_LIB_YUV_PIC_H_
#define XVC_COMMON_LIB_YUV_PIC_H_

#include <vector>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/sample_buffer.h"

namespace xvc {

class YuvPicture {
public:
  YuvPicture(ChromaFormat chroma_format, int width, int height, int bitdepth,
             bool padding);

  int GetWidth(YuvComponent comp) const { return width_[comp]; }
  int GetHeight(YuvComponent comp) const { return height_[comp]; }
  ptrdiff_t GetStride(YuvComponent comp) const { return stride_[comp]; }
  int GetSizeShiftX(YuvComponent comp) const { return shiftx_[comp]; }
  int GetSizeShiftY(YuvComponent comp) const { return shifty_[comp]; }
  int GetBitdepth() const { return bitdepth_; }
  ChromaFormat GetChromaFormat() const { return chroma_format_; }

  Sample* GetSamplePtr(YuvComponent comp, int x, int y) {
    return comp_pel_[comp] + y * GetStride(comp) + x;
  }
  const Sample* GetSamplePtr(YuvComponent comp, int x, int y) const {
    return comp_pel_[comp] + y * GetStride(comp) + x;
  }
  SampleBuffer GetSampleBuffer(YuvComponent comp, int x, int y) {
    return SampleBuffer(GetSamplePtr(comp, x, y), GetStride(comp));
  }
  SampleBufferConst GetSampleBuffer(YuvComponent comp, int x, int y) const {
    return SampleBufferConst(GetSamplePtr(comp, x, y), GetStride(comp));
  }
  void CopyFrom(const uint8_t *picture_bytes, int input_bitdepth);
  void CopyFromWithPadding(const uint8_t *picture_bytes, int input_bitdepth);
  void CopyFromWithResampling(const uint8_t *picture_bytes, int input_bitdepth,
                              int orig_width, int orig_height);
  void CopyToSameBitdepth(std::vector<uint8_t> *pic_bytes) const;
  void CopyTo(std::vector<uint8_t> *out_bytes, int out_width,
              int out_height, ChromaFormat out_chroma_format,
              int out_bitdepth, ColorMatrix out_color_matrix);
  void PadBorder();

private:
  uint8_t* CopyWithShift(uint8_t *out8, int width,
                         int height, ptrdiff_t stride, int out_bitdepth,
                         const Sample *src, int bitdepth) const;
  template <typename T>
  void ConvertColorSpace(uint8_t *dst, int width, int height,
                         const uint16_t *src, int bitdepth,
                         ColorMatrix color_matrix) const;
  void ConvertColorSpace8bit709(uint8_t *dst, int width, int height,
                                const uint16_t *src) const;

  ChromaFormat chroma_format_;
  int width_[constants::kMaxYuvComponents];
  int height_[constants::kMaxYuvComponents];
  ptrdiff_t stride_[constants::kMaxYuvComponents];
  int total_height_[constants::kMaxYuvComponents];
  int shiftx_[constants::kMaxYuvComponents];
  int shifty_[constants::kMaxYuvComponents];
  int bitdepth_;
  std::vector<Sample> sample_buffer_;
  std::vector<uint8_t> tmp_bytes_;
  Sample *comp_pel_[constants::kMaxYuvComponents];
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_YUV_PIC_H_
