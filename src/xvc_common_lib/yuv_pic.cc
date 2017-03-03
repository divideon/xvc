/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/yuv_pic.h"

#include <cassert>
#include <cstring>

#include "xvc_common_lib/resample.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

YuvPicture::YuvPicture(ChromaFormat chroma_fmt, int width, int height,
                       int bitdepth, bool padding)
  : chroma_format_(chroma_fmt), bitdepth_(bitdepth) {
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

void YuvPicture::CopyFrom(const uint8_t *pic8,
                          int input_bitdepth) {
  // TODO(Dev) Padding support not implemented
  assert(width_[0] == stride_[0]);
  assert(height_[0] == total_height_[0]);

  if (sample_buffer_.empty()) {
    return;
  }
  if (sizeof(*pic8) == sizeof(Sample)) {
    if (input_bitdepth == 8) {
      std::memcpy(&sample_buffer_[0], pic8,
                  sample_buffer_.size() * sizeof(Sample));
    } else {
      assert(0);  // not supported
    }
    return;
  }
  // High bitdepth combinations
  const uint16_t *pic16 = reinterpret_cast<const uint16_t *>(&pic8[0]);
  int bit_shift = bitdepth_ - input_bitdepth;
  if (input_bitdepth == 8) {
    for (size_t i = 0; i < sample_buffer_.size(); i++) {
      sample_buffer_[i] = pic8[i] << bit_shift;
    }
  } else if (input_bitdepth == bitdepth_) {
    // Assuming little ending
    std::memcpy(&sample_buffer_[0], pic8,
                sample_buffer_.size() * sizeof(Sample));
  } else if (input_bitdepth <= bitdepth_) {
    for (size_t i = 0; i < sample_buffer_.size(); i++) {
      sample_buffer_[i] = static_cast<Sample>(pic16[i] << bit_shift);
    }
  } else {
    assert(0);  // not supported
  }
}

void YuvPicture::CopyToSameBitdepth(std::vector<uint8_t> *out_bytes) const {
  int num_samples = util::GetTotalNumSamples(width_[YuvComponent::kY],
                                             height_[YuvComponent::kY],
                                             chroma_format_);
  size_t pic_bytes = num_samples * (bitdepth_ > 8 ? 2 : 1);
  out_bytes->resize(pic_bytes);
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

void YuvPicture::CopyTo(std::vector<uint8_t> *out_bytes, int out_width,
                        int out_height, ChromaFormat out_chroma_format,
                        int out_bitdepth) const {
  int num_samples_internal =
    util::GetTotalNumSamples(width_[YuvComponent::kY],
                             height_[YuvComponent::kY], chroma_format_);
  if (num_samples_internal == 0) {
    return;
  }
  int num_samples_out =
    util::GetTotalNumSamples(out_width, out_height, out_chroma_format);
  int num_components_out = util::GetNumComponents(out_chroma_format);

  size_t pic_bytes = num_samples_out * (out_bitdepth > 8 ? 2 : 1);
  out_bytes->resize(pic_bytes);
  uint8_t *out8 = &(*out_bytes)[0];
  uint16_t *out16 = reinterpret_cast<uint16_t*>(out8);

  // Resampling is performed if resolution or chroma format is different.
  if (out_width != width_[YuvComponent::kY] ||
      out_height != height_[YuvComponent::kY] ||
      (out_chroma_format != chroma_format_ &&
       out_chroma_format != ChromaFormat::kMonochrome)) {
    int sample_size = (out_bitdepth > 8 ? 2 : 1);
    for (int c = 0; c < num_components_out; c++) {
      YuvComponent comp = YuvComponent(c);
      uint8_t *src8 = reinterpret_cast<uint8_t*>(comp_pel_[c]);
      int dst_width = util::ScaleSizeX(out_width, out_chroma_format, comp);
      int dst_height = util::ScaleSizeY(out_height, out_chroma_format, comp);
      if (c < util::GetNumComponents(chroma_format_)) {
        int src_width = width_[c];
        int src_height = height_[c];
        ptrdiff_t src_stride = stride_[c];
        ptrdiff_t dst_stride = dst_width;
        if (out_bitdepth > 8) {
          resample::Resample<Sample, uint16_t>
            (out8, dst_width, dst_height, dst_stride, out_bitdepth,
             src8, src_width, src_height, src_stride, bitdepth_);
        } else {
          resample::Resample<Sample, uint8_t>
            (out8, dst_width, dst_height, dst_stride, out_bitdepth,
             src8, src_width, src_height, src_stride, bitdepth_);
        }
      } else {
        // When monochrome is converted to a chroma format with chroma
        // components, all chroma samples are set to 1 << (out_bitdepth - 1).
        std::memset(out8, 1 << (out_bitdepth - 1),
                    dst_width * dst_height * sample_size);
      }
      out8 += dst_width * dst_height * sample_size;
    }
    return;
  }

  for (int c = 0; c < num_components_out; c++) {
    YuvComponent comp = YuvComponent(c);
    const Sample *src = comp_pel_[c];
    if (out_bitdepth > 8) {
      if (out_bitdepth == bitdepth_) {
        for (int y = 0; y < height_[comp]; y++) {
          memcpy(out16, src, width_[comp] * sizeof(Sample));
          out16 += width_[comp];
          src += stride_[c];
        }
      } else if (out_bitdepth > bitdepth_) {
        int bit_shift = out_bitdepth - bitdepth_;
        for (int y = 0; y < height_[comp]; y++) {
          for (int x = 0; x < width_[comp]; x++) {
            *out16++ = src[x] << bit_shift;
          }
          src += stride_[c];
        }
      } else {
        int bit_shift = bitdepth_ - out_bitdepth;
        Sample sample_max = (1 << out_bitdepth) - 1;
        for (int y = 0; y < height_[comp]; y++) {
          for (int x = 0; x < width_[comp]; x++) {
            *out16++ = static_cast<uint16_t>(util::ClipBD(
              (src[x] + (1 << (bit_shift - 1))) >> bit_shift,
              sample_max));
          }
          src += stride_[c];
        }
      }
    } else {
      if (bitdepth_ <= 8) {
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
      } else {
        int bit_shift = bitdepth_ - out_bitdepth;
        for (int y = 0; y < height_[comp]; y++) {
          for (int x = 0; x < width_[comp]; x++) {
            *out8++ = static_cast<uint8_t>(util::Clip3(
              (src[x] + (1 << (bit_shift - 1))) >> bit_shift, 0, 255));
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
