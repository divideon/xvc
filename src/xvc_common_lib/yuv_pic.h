/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
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

  Sample *GetSamplePtr(YuvComponent comp, int x, int y) {
    return comp_pel_[comp] + y * GetStride(comp) + x;
  }
  const Sample *GetSamplePtr(YuvComponent comp, int x, int y) const {
    return comp_pel_[comp] + y * GetStride(comp) + x;
  }
  SampleBuffer GetSampleBuffer(YuvComponent comp, int x, int y) {
    return SampleBuffer(GetSamplePtr(comp, x, y), GetStride(comp));
  }
  DataBuffer<const Sample> GetSampleBuffer(YuvComponent comp,
                                           int x, int y) const {
    return DataBuffer<const Sample>(GetSamplePtr(comp, x, y), GetStride(comp));
  }
  void CopyFrom(const uint8_t *picture_bytes, int input_bitdepth);
  void CopyToSameBitdepth(std::vector<uint8_t> *pic_bytes) const;
  void CopyTo(std::vector<uint8_t> *out_bytes, int out_width,
              int out_height, ChromaFormat out_chroma_format,
              int out_bitdepth) const;
  void PadBorder();

private:
  ChromaFormat chroma_format_;
  int width_[constants::kMaxYuvComponents];
  int height_[constants::kMaxYuvComponents];
  ptrdiff_t stride_[constants::kMaxYuvComponents];
  int total_height_[constants::kMaxYuvComponents];
  int shiftx_[constants::kMaxYuvComponents];
  int shifty_[constants::kMaxYuvComponents];
  int bitdepth_;
  std::vector<Sample> sample_buffer_;
  Sample *comp_pel_[constants::kMaxYuvComponents];
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_YUV_PIC_H_
