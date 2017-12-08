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

const int kColorConversionBitdepth = 12;

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

void YuvPicture::CopyFromWithPadding(const uint8_t *pic8,
                                     int input_bitdepth) {
  if (sample_buffer_.empty()) {
    return;
  }

  if (input_bitdepth == 8) {
    for (int c = 0; c < constants::kMaxYuvComponents; c++) {
      YuvComponent comp = YuvComponent(c);
      for (int y = 0; y < height_[comp]; y++) {
        for (int x = 0; x < width_[comp]; x++) {
          Sample *dst = GetSamplePtr(comp, x, y);
          *dst = static_cast<Sample>(*pic8++);
        }
      }
    }
  } else {
    const uint16_t *pic16 = reinterpret_cast<const uint16_t *>(&pic8[0]);
    for (int c = 0; c < constants::kMaxYuvComponents; c++) {
      YuvComponent comp = YuvComponent(c);
      for (int y = 0; y < height_[comp]; y++) {
        for (int x = 0; x < width_[comp]; x++) {
          Sample *dst = GetSamplePtr(comp, x, y);
          *dst = static_cast<Sample>(*pic16++);
        }
      }
    }
  }
  PadBorder();
}


void YuvPicture::CopyFromWithResampling(const uint8_t *pic8, int input_bitdepth,
                                        int orig_width, int orig_height) {
  YuvPicture temp_pic(chroma_format_, orig_width, orig_height, input_bitdepth,
                      true);
  temp_pic.CopyFromWithPadding(pic8, input_bitdepth);
  for (int c = 0; c < constants::kMaxYuvComponents; c++) {
    YuvComponent comp = YuvComponent(c);
    uint8_t* dst = reinterpret_cast<uint8_t*>(GetSamplePtr(comp, 0, 0));
    uint8_t* src =
      reinterpret_cast<uint8_t*>(temp_pic.GetSamplePtr(comp, 0, 0));
    resample::Resample<Sample, Sample>
      (dst, width_[c], height_[c],
       stride_[c], bitdepth_,
       src, temp_pic.GetWidth(comp), temp_pic.GetHeight(comp),
       temp_pic.GetStride(comp), input_bitdepth);
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
                        int out_bitdepth, ColorMatrix out_color_matrix,
                        int dither) {
  int num_samples_internal =
    util::GetTotalNumSamples(width_[YuvComponent::kY],
                             height_[YuvComponent::kY], chroma_format_);
  if (num_samples_internal == 0) {
    return;
  }
  int num_samples_out =
    util::GetTotalNumSamples(out_width, out_height, out_chroma_format);
  int num_components_out = util::GetNumComponents(out_chroma_format);
  int dst_bitdepth = out_bitdepth;
  size_t pic_bytes = num_samples_out * (dst_bitdepth > 8 ? 2 : 1);
  out_bytes->resize(pic_bytes);
  if (out_chroma_format == ChromaFormat::kArgb) {
    dst_bitdepth = kColorConversionBitdepth;
    pic_bytes = num_samples_out * (dst_bitdepth > 8 ? 2 : 1);
  }

  uint8_t *out8 = &(*out_bytes)[0];
  uint16_t *out16 = reinterpret_cast<uint16_t*>(out8);
  if (out_chroma_format == ChromaFormat::kArgb) {
    if (tmp_bytes_.size() < pic_bytes) {
      tmp_bytes_.resize(pic_bytes);
    }
    out8 = &tmp_bytes_[0];
    out16 = reinterpret_cast<uint16_t*>(&tmp_bytes_[0]);
  }

  // Resampling is performed if resolution or chroma format is different.
  if (out_width != width_[YuvComponent::kY] ||
      out_height != height_[YuvComponent::kY] ||
      (out_chroma_format != chroma_format_ &&
       out_chroma_format != ChromaFormat::kMonochrome)) {
    int sample_size = (dst_bitdepth > 8 ? 2 : 1);
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
        if (dst_width == src_width && dst_height == src_height) {
          CopyWithShift(out8, width_[c], height_[c], stride_[c],
                        dst_bitdepth, comp_pel_[c], bitdepth_, dither);
        } else if (comp != YuvComponent::kY &&
                   dst_width == 2 * src_width &&
                   dst_height == 2 * src_height) {
          if (dst_bitdepth > 8) {
            resample::BilinearResample<Sample, uint16_t>
              (out8, dst_width, dst_height, dst_stride, dst_bitdepth,
               src8, src_width, src_height, src_stride, bitdepth_);
          } else {
            resample::BilinearResample<Sample, uint8_t>
              (out8, dst_width, dst_height, dst_stride, dst_bitdepth,
               src8, src_width, src_height, src_stride, bitdepth_);
          }
        } else if (dst_bitdepth > 8) {
          resample::Resample<Sample, uint16_t>
            (out8, dst_width, dst_height, dst_stride, dst_bitdepth,
             src8, src_width, src_height, src_stride, bitdepth_);
        } else {
          resample::Resample<Sample, uint8_t>
            (out8, dst_width, dst_height, dst_stride, dst_bitdepth,
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
    if (out_chroma_format == ChromaFormat::kArgb) {
      if (out_bitdepth > 8) {
        ConvertColorSpace<uint16_t>(&(*out_bytes)[0], out_width, out_height,
                                    out16, out_bitdepth, out_color_matrix);
      } else {
        if (out_color_matrix == ColorMatrix::kUndefinedColorMatrix ||
            out_color_matrix == ColorMatrix::k709) {
          ConvertColorSpace8bit709(&(*out_bytes)[0], out_width, out_height,
                                   out16);
        } else {
          ConvertColorSpace<uint8_t>(&(*out_bytes)[0], out_width, out_height,
                                     out16, out_bitdepth, out_color_matrix);
        }
      }
    }
    return;
  }

  for (int c = 0; c < num_components_out; c++) {
    const Sample *src = comp_pel_[c];
    out8 = CopyWithShift(out8, width_[c], height_[c], stride_[c], out_bitdepth,
                         src, bitdepth_, dither);
  }
}

uint8_t* YuvPicture::CopyWithShift(uint8_t *out8, int width, int height,
                                   ptrdiff_t stride, int out_bitdepth,
                                   const Sample *src, int bitdepth,
                                   int dither) const {
  if (out_bitdepth > 8) {
    uint16_t *out16 = reinterpret_cast<uint16_t*>(out8);
    if (out_bitdepth == bitdepth) {
      for (int y = 0; y < height; y++) {
        memcpy(out16, src, width * sizeof(Sample));
        out16 += width;
        src += stride;
      }
    } else if (out_bitdepth > bitdepth) {
      int bit_shift = out_bitdepth - bitdepth;
      for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
          *out16++ = src[x] << bit_shift;
        }
        src += stride;
      }
    } else {
      int bit_shift = bitdepth - out_bitdepth;
      Sample sample_max = (1 << out_bitdepth) - 1;
      if (dither) {
        int d = 0;
        int mask = (1 << bit_shift) - 1;
        for (int y = 0; y < height; y++) {
          for (int x = 0; x < width; x++) {
            d += src[x];
            *out16++ = static_cast<uint16_t>(util::ClipBD(d >> bit_shift,
              sample_max));
            d &= mask;
          }
          src += stride;
        }
      } else {
        for (int y = 0; y < height; y++) {
          for (int x = 0; x < width; x++) {
            *out16++ = static_cast<uint16_t>(util::ClipBD(
              (src[x] + (1 << (bit_shift - 1))) >> bit_shift,
              sample_max));
          }
          src += stride;
        }
      }
    }
    return reinterpret_cast<uint8_t*>(out16);
  } else {
    if (bitdepth <= 8) {
      if (sizeof(Sample) == 1) {
        for (int y = 0; y < height; y++) {
          memcpy(out8, src, width * sizeof(Sample));
          out8 += width;
          src += stride;
        }
      } else {
        for (int y = 0; y < height; y++) {
          for (int x = 0; x < width; x++) {
            *out8++ = static_cast<uint8_t>(src[x]);
          }
          src += stride;
        }
      }
    } else {
      int bit_shift = bitdepth - out_bitdepth;
      if (dither) {
        int d = 0;
        int mask = (1 << bit_shift) - 1;
        for (int y = 0; y < height; y++) {
          for (int x = 0; x < width; x++) {
            d += src[x];
            *out8++ = static_cast<uint8_t>(util::Clip3(d >> bit_shift, 0, 255));
            d &= mask;
          }
          src += stride;
        }
      } else {
        for (int y = 0; y < height; y++) {
          for (int x = 0; x < width; x++) {
            *out8++ = static_cast<uint8_t>(util::Clip3(
              (src[x] + (1 << (bit_shift - 1))) >> bit_shift, 0, 255));
          }
          src += stride;
        }
      }
    }
    return out8;
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

template <typename T>
void YuvPicture::ConvertColorSpace(uint8_t *out, int width, int height,
                                   const uint16_t *src, int bitdepth,
                                   ColorMatrix color_matrix) const {
  const int size = width * height;
  const uint16_t *s0 = src;
  const uint16_t *s1 = src + size;
  const uint16_t *s2 = src + 2 * size;
  const Sample sample_max = (1 << bitdepth) - 1;
  const int shift = 10 + kColorConversionBitdepth - bitdepth;
  T *dst = reinterpret_cast<T*>(out);

  static const std::array<std::array<std::array<int, 3>, 3>, 4> kM = { {
    { {  // Default, same as BT.709
      { 1192, 0, 1877 },
      { 1192, -223, -558 },
      { 1192, 2212, 0 }
    } },
    { {  // BT.601
      { 1192, 0, 1671 },
      { 1192, -410, -851 },
      { 1192, 2112, 0 }
    } },
    { {  // BT.709
      { 1192, 0, 1877 },
      { 1192, -223, -558 },
      { 1192, 2212, 0 }
    } },
    { {  // BT.2020
      { 1192, 0, 1758 },
      { 1192, -196, -681 },
      { 1192, 2243, 0 }
    } },
  } };
  const unsigned int k = static_cast<int>(color_matrix);
  assert(k < kM.size());

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      int c = *s0++ - (16 << (kColorConversionBitdepth - 8));
      int d = *s1++ - (128 << (kColorConversionBitdepth - 8));
      int e = *s2++ - (128 << (kColorConversionBitdepth - 8));
      dst[0] = static_cast<T>(
        util::ClipBD((kM[k][0][0] * c + kM[k][0][2] * e) >> shift, sample_max));
      dst[1] = static_cast<T>(
        util::ClipBD((kM[k][1][0] * c + kM[k][1][1] * d + kM[k][1][2] * e)
                     >> shift, sample_max));
      dst[2] = static_cast<T>(
        util::ClipBD((kM[k][2][0] * c + kM[k][2][1] * d) >> shift, sample_max));
      dst[3] = static_cast<T>(sample_max);
      dst += 4;
    }
  }
}

void YuvPicture::ConvertColorSpace8bit709(uint8_t *dst, int width, int height,
                                          const uint16_t *src) const {
  const int size = width * height;
  const uint16_t *s0 = src;
  const uint16_t *s1 = src + size;
  const uint16_t *s2 = src + 2 * size;
  const Sample sample_max = 255;
  const int shift = kColorConversionBitdepth + 2;

  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      const int c = 1192 * (*s0++ - (16 << (kColorConversionBitdepth - 8)));
      const int d = *s1++ - (128 << (kColorConversionBitdepth - 8));
      const int e = *s2++ - (128 << (kColorConversionBitdepth - 8));
      dst[0] = static_cast<uint8_t>(
        util::ClipBD((c + 1877 * e) >> shift, sample_max));
      dst[1] = static_cast<uint8_t>(
        util::ClipBD((c - 223 * d - 558 * e) >> shift, sample_max));
      dst[2] = static_cast<uint8_t>(
        util::ClipBD((c + 2212 * d) >> shift, sample_max));
      dst[3] = static_cast<uint8_t>(sample_max);
      dst += 4;
    }
  }
}

}   // namespace xvc
