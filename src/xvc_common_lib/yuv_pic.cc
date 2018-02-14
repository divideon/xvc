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

#include "xvc_common_lib/yuv_pic.h"

#include <cassert>
#include <cstring>

#include "xvc_common_lib/resample.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

YuvPicture::YuvPicture(ChromaFormat chroma_fmt, int width, int height,
                       int bitdepth, bool padding, int crop_width,
                       int crop_height)
  : chroma_format_(chroma_fmt),
  bitdepth_(bitdepth),
  crop_width_(crop_width),
  crop_height_(crop_height) {
  int offset_x = padding ? (constants::kMaxBlockSize + 16) : 0;
  int offset_y = padding ? (constants::kMaxBlockSize + 16) : 0;
  width_[0] = width;
  height_[0] = height;
  shiftx_[0] = 0;
  shifty_[0] = 0;
  for (int c = kU; c < constants::kMaxYuvComponents; c++) {
    width_[c] = util::ScaleChromaX(width_[0], chroma_fmt);
    height_[c] = util::ScaleChromaY(height_[0], chroma_fmt);
    shiftx_[c] = util::GetChromaShiftX(chroma_fmt);
    shifty_[c] = util::GetChromaShiftY(chroma_fmt);
  }
  size_t total_size_w_padding = 0;
  for (int c = 0; c < constants::kMaxYuvComponents; c++) {
    stride_[c] = width_[c] +
      (util::ScaleSizeX(offset_x, chroma_fmt, YuvComponent(c)) << 1);
    total_height_[c] = height_[c] +
      (util::ScaleSizeY(offset_x, chroma_fmt, YuvComponent(c)) << 1);
    total_size_w_padding += stride_[c] * total_height_[c];
  }
  sample_buffer_.resize(total_size_w_padding);
  bool not_empty = width != 0 && height != 0;
  Sample *start_ptr = not_empty ? &sample_buffer_[0] : nullptr;
  for (int c = 0; c < constants::kMaxYuvComponents; c++) {
    int comp_offset_x = util::ScaleSizeX(offset_x, chroma_fmt, YuvComponent(c));
    int comp_offset_y = util::ScaleSizeY(offset_y, chroma_fmt, YuvComponent(c));
    comp_pel_[c] = start_ptr + stride_[c] * comp_offset_y + comp_offset_x;
    start_ptr += total_height_[c] * stride_[c];
  }
}

int YuvPicture::GetCropWidth(YuvComponent comp) const {
  return util::ScaleSizeX(crop_width_, chroma_format_, comp);
}

int YuvPicture::GetCropHeight(YuvComponent comp) const {
  return util::ScaleSizeY(crop_height_, chroma_format_, comp);
}

void YuvPicture::CopyToSameBitdepth(std::vector<uint8_t> *out_bytes) const {
  int num_samples = util::GetTotalNumSamples(width_[YuvComponent::kY],
                                             height_[YuvComponent::kY],
                                             chroma_format_);
  size_t pic_bytes = num_samples * (bitdepth_ > 8 ? 2 : 1);
  out_bytes->resize(pic_bytes);
  if (!num_samples) {
    return;
  }
  uint16_t *out16 = reinterpret_cast<uint16_t *>(&(*out_bytes)[0]);
  uint8_t *out8 = &(*out_bytes)[0];

  for (int c = 0; c < constants::kMaxYuvComponents; c++) {
    YuvComponent comp = YuvComponent(c);
    const Sample *src = comp_pel_[c];
    if (bitdepth_ > 8) {
      for (int y = 0; y < height_[comp]; y++) {
        memcpy(out16, src, width_[comp] * sizeof(Sample));
        out16 += width_[comp];
        src += stride_[c];
      }
    } else {
      if (sizeof(Sample) == 1) {
        for (int y = 0; y < height_[comp]; y++) {
          memcpy(out8, src, width_[comp] * sizeof(Sample));
          out8 += width_[comp];
          src += stride_[c];
        }
      } else {
        for (int y = 0; y < height_[comp]; y++) {
          for (int x = 0; x < width_[comp]; x++) {
            *out8++ = static_cast<uint8_t>(src[x]);
          }
          src += stride_[c];
        }
      }
    }
  }
}

void YuvPicture::PadBorder() {
  if (width_[0] == 0 && height_[0] == 0) {
    return;
  }
  for (int c = 0; c < constants::kMaxYuvComponents; c++) {
    int offset_x = static_cast<int>((stride_[c] - width_[c]) >> 1);
    int offset_y = static_cast<int>((total_height_[c] - height_[c]) >> 1);
    // Top
    Sample *row = comp_pel_[c];
    for (int y = -offset_y; y < 0; y++) {
      std::memcpy(row + y * stride_[c], row, width_[c] * sizeof(Sample));
    }
    // Bottom
    row += (height_[c] - 1) * stride_[c];
    for (int y = 1; y <= offset_y; y++) {
      std::memcpy(row + y * stride_[c], row, width_[c] * sizeof(Sample));
    }
    // Left & right
    row = comp_pel_[c] - offset_y * stride_[c];
    for (int y = 0; y < total_height_[c]; y++) {
      Sample left = row[0];
      // TODO(Dev) Replace with memset for bitdepth=8
      for (int x = -offset_x; x < 0; x++) {
        row[x] = left;
      }
      Sample right = row[width_[c] - 1];
      for (int x = 0; x < offset_x; x++) {
        row[width_[c] + x] = right;
      }
      row += stride_[c];
    }
  }
}

}   // namespace xvc
