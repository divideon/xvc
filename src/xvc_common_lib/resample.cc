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

#include "xvc_common_lib/resample.h"

#include <algorithm>
#include <cassert>
#include <vector>
#include <limits>

#include "xvc_common_lib/utils.h"

namespace xvc {

void Resampler::ConvertFrom(const PictureFormat &src_format,
                            const uint8_t *src_bytes, YuvPicture *out_pic) {
  const int num_components = util::GetNumComponents(out_pic->GetChromaFormat());
  const YuvComponent luma = YuvComponent::kY;
  assert(out_pic->GetChromaFormat() == src_format.chroma_format);
  if (out_pic->GetWidth(luma) == 0 ||
      out_pic->GetHeight(luma) == 0) {
    return;
  }
  if (out_pic->GetWidth(luma) == src_format.width &&
      out_pic->GetHeight(luma) == src_format.height) {
    for (int c = 0; c < num_components; c++) {
      const YuvComponent comp = static_cast<YuvComponent>(c);
      const ptrdiff_t src_stride = (src_format.bitdepth == 8 ? 1 : 2) *
        util::ScaleSizeX(src_format.width, src_format.chroma_format, comp);
      src_bytes = CopyFromBytesFast(comp, src_bytes, src_stride,
                                    src_format.bitdepth, out_pic);
    }
  } else if (out_pic->GetCropWidth() != 0 ||
             out_pic->GetCropHeight() != 0) {
    for (int c = 0; c < num_components; c++) {
      const YuvComponent comp = YuvComponent(c);
      const ptrdiff_t src_stride = (src_format.bitdepth == 8 ? 1 : 2) *
        util::ScaleSizeX(src_format.width, src_format.chroma_format, comp);
      src_bytes = CopyFromBytesWithPadding(comp, src_bytes, src_stride,
                                           src_format, out_pic);
    }
  } else {
    CopyFromBytesWithResampling(src_bytes, src_format, out_pic);
  }
}

void Resampler::ConvertFrom(const PictureFormat &src_format,
                            const PicPlanes &input_planes,
                            YuvPicture *out_pic) {
  const int num_components = util::GetNumComponents(out_pic->GetChromaFormat());
  const YuvComponent luma = YuvComponent::kY;
  assert(out_pic->GetChromaFormat() == src_format.chroma_format);
  if (out_pic->GetWidth(luma) == 0 ||
      out_pic->GetHeight(luma) == 0) {
    return;
  }
  if (out_pic->GetWidth(luma) == src_format.width &&
      out_pic->GetHeight(luma) == src_format.height) {
    for (int c = 0; c < num_components; c++) {
      const YuvComponent comp = static_cast<YuvComponent>(c);
      CopyFromBytesFast(comp, input_planes[c].first, input_planes[c].second,
                        src_format.bitdepth, out_pic);
    }
  } else if (out_pic->GetCropWidth() != 0 ||
             out_pic->GetCropHeight() != 0) {
    for (int c = 0; c < num_components; c++) {
      const YuvComponent comp = YuvComponent(c);
      CopyFromBytesWithPadding(comp, input_planes[c].first,
                               input_planes[c].second, src_format, out_pic);
    }
  } else {
    assert(0);  // not implemented
  }
}

void Resampler::ConvertTo(const YuvPicture &src_pic,
                          const PictureFormat &out_fmt,
                          std::vector<uint8_t> *out_bytes) {
  if (src_pic.GetWidth(YuvComponent::kY) == 0 ||
      src_pic.GetHeight(YuvComponent::kY) == 0) {
    out_bytes->clear();
    return;
  }
  const int num_components_out =
    util::GetNumComponents(out_fmt.chroma_format);
  const int num_samples_out =
    util::GetTotalNumSamples(out_fmt.width, out_fmt.height,
                             out_fmt.chroma_format);
  int dst_bitdepth = out_fmt.bitdepth;
  out_bytes->resize(num_samples_out * (dst_bitdepth > 8 ? 2 : 1));
  uint8_t *out8 = &(*out_bytes)[0];
  if (out_fmt.chroma_format == ChromaFormat::kArgb) {
    dst_bitdepth = kColorConversionBitdepth;
    tmp_bytes_.resize(num_samples_out * (dst_bitdepth > 8 ? 2 : 1));
    out8 = &tmp_bytes_[0];
  }

  const int src_width_nopad = src_pic.GetDisplayWidth(YuvComponent::kY);
  const int src_height_nopad = src_pic.GetDisplayHeight(YuvComponent::kY);
  if (out_fmt.width != src_width_nopad ||
      out_fmt.height != src_height_nopad ||
      (out_fmt.chroma_format != src_pic.GetChromaFormat() &&
       out_fmt.chroma_format != ChromaFormat::kMonochrome)) {
    CopyToWithResize(src_pic, out_fmt, dst_bitdepth, out8);
    if (out_fmt.chroma_format == ChromaFormat::kArgb) {
      uint16_t *out16 = reinterpret_cast<uint16_t*>(out8);
      if (out_fmt.bitdepth > 8) {
        ConvertColorSpace<uint16_t>(&(*out_bytes)[0], out_fmt.width,
                                    out_fmt.height, out16, out_fmt.bitdepth,
                                    out_fmt.color_matrix);
      } else {
        if (out_fmt.color_matrix == ColorMatrix::kUndefined ||
            out_fmt.color_matrix == ColorMatrix::k709) {
          ConvertColorSpace8bit709(&(*out_bytes)[0], out_fmt.width,
                                   out_fmt.height, out16);
        } else {
          ConvertColorSpace<uint8_t>(&(*out_bytes)[0], out_fmt.width,
                                     out_fmt.height, out16, out_fmt.bitdepth,
                                     out_fmt.color_matrix);
        }
      }
    }
  } else {
    // Basic conversion or copy without resolution or color space change
    for (int c = 0; c < num_components_out; c++) {
      const YuvComponent comp = YuvComponent(c);
      out8 = CopyToBytesWithShift(comp, src_pic, out_fmt.bitdepth,
                                  out_fmt.dither, out8);
    }
  }
}

const uint8_t*
Resampler::CopyFromBytesFast(YuvComponent comp, const uint8_t *input_bytes,
                             ptrdiff_t input_stride, int input_bitdepth,
                             YuvPicture *out_pic) const {
  assert(out_pic->GetCropWidth() == 0);
  assert(out_pic->GetCropHeight() == 0);
  const int width = out_pic->GetWidth(comp);
  const int height = out_pic->GetHeight(comp);
  const ptrdiff_t output_stride = out_pic->GetStride(comp);

  // Special case if compiled with low-bitdepth support only
  if (sizeof(uint8_t) == sizeof(Sample)) {
    if (input_bitdepth != 8) {
      assert(0);  // not supported
    }
    assert(width * sizeof(uint8_t) <= static_cast<size_t>(input_stride));
    if (input_stride == output_stride) {
      const size_t samples = static_cast<size_t>(input_stride * height);
      Sample *dst = out_pic->GetSamplePtr(comp, 0, 0);
      std::memcpy(dst, input_bytes, samples);
      input_bytes += samples;
    } else {
      Sample *dst = out_pic->GetSamplePtr(comp, 0, 0);
      for (int y = 0; y < height; y++) {
        std::memcpy(dst, input_bytes, width * sizeof(Sample));
        input_bytes += input_stride;
        dst += output_stride;
      }
    }
    return input_bytes;
  }

  // Normal high bitdepth case
  const int bit_shift = out_pic->GetBitdepth() - input_bitdepth;
  Sample *dst = out_pic->GetSamplePtr(comp, 0, 0);
  if (input_bitdepth > out_pic->GetBitdepth()) {
    assert(0);  // not supported
  } else if (input_bitdepth > 8 &&
             input_stride == output_stride &&
             input_bitdepth == out_pic->GetBitdepth()) {
    assert(width * sizeof(Sample) <= static_cast<size_t>(input_stride));
    const size_t samples = static_cast<size_t>(input_stride * height);
    std::memcpy(dst, input_bytes, samples);
    input_bytes += samples;
  } else if (input_bitdepth == 8) {
    assert(width * sizeof(uint8_t) <= static_cast<size_t>(input_stride));
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        dst[x] = static_cast<Sample>(input_bytes[x] << bit_shift);
      }
      input_bytes += input_stride;
      dst += output_stride;
    }
  } else {
    assert(width * sizeof(uint16_t) <= static_cast<size_t>(input_stride));
    for (int y = 0; y < height; y++) {
      const uint16_t *pic16 =
        reinterpret_cast<const uint16_t *>(&input_bytes[0]);
      for (int x = 0; x < width; x++) {
        dst[x] = static_cast<Sample>(pic16[x] << bit_shift);
      }
      input_bytes += input_stride;
      dst += output_stride;
    }
  }
  return input_bytes;
}

const uint8_t *
Resampler::CopyFromBytesWithPadding(YuvComponent comp,
                                    const uint8_t *src_bytes,
                                    ptrdiff_t src_stride,
                                    const PictureFormat &input_format,
                                    YuvPicture *out_pic) const {
  assert(input_format.width == out_pic->GetDisplayWidth(YuvComponent::kY));
  assert(input_format.height == out_pic->GetDisplayHeight(YuvComponent::kY));
  assert(input_format.chroma_format == out_pic->GetChromaFormat());
  assert(out_pic->GetWidth(YuvComponent::kY) >= 0);
  assert(out_pic->GetHeight(YuvComponent::kY) >= 0);
  assert(sizeof(Sample) == 2 || input_format.bitdepth == 8);
  assert(input_format.bitdepth <= out_pic->GetBitdepth());

  const int in_width =
    util::ScaleSizeX(input_format.width, input_format.chroma_format, comp);
  const int in_height =
    util::ScaleSizeY(input_format.height, input_format.chroma_format, comp);
  const int pad_x = out_pic->GetWidth(comp) - in_width;
  const int pad_y = out_pic->GetHeight(comp) - in_height;
  const int upshift = out_pic->GetBitdepth() - input_format.bitdepth;
  const ptrdiff_t dst_stride = out_pic->GetStride(comp);
  Sample *dst = out_pic->GetSamplePtr(comp, 0, 0);

  if (input_format.bitdepth == 8) {
    for (int y = 0; y < in_height; y++) {
      for (int x = 0; x < in_width; x++) {
        dst[x] = static_cast<Sample>(src_bytes[x]) << upshift;
      }
      for (int x2 = 0; x2 < pad_x; x2++) {
        dst[in_width + x2] = dst[in_width - 1];
      }
      src_bytes += src_stride;
      dst += dst_stride;
    }
  } else {
    for (int y = 0; y < in_height; y++) {
      const uint16_t *pic16 = reinterpret_cast<const uint16_t *>(&src_bytes[0]);
      for (int x = 0; x < in_width; x++) {
        dst[x] = static_cast<Sample>(pic16[x]) << upshift;
      }
      for (int x2 = 0; x2 < pad_x; x2++) {
        dst[in_width + x2] = dst[in_width - 1];
      }
      src_bytes += src_stride;
      dst += dst_stride;
    }
  }
  for (int y2 = 0; y2 < pad_y; y2++) {
    memcpy(dst + y2 * dst_stride, dst - dst_stride,
           dst_stride * sizeof(Sample));
  }
  return src_bytes;
}

void Resampler::CopyFromBytesWithResampling(const uint8_t *pic8,
                                            const PictureFormat &src_format,
                                            YuvPicture *out_pic) const {
  const int num_components = util::GetNumComponents(out_pic->GetChromaFormat());
  // 1. Create a padded version of original image
  YuvPicture temp_pic(src_format.chroma_format, src_format.width,
                      src_format.height, src_format.bitdepth,
                      true, 0, 0);
  for (int c = 0; c < num_components; c++) {
    const YuvComponent comp = static_cast<YuvComponent>(c);
    const ptrdiff_t src_stride = (src_format.bitdepth == 8 ? 1 : 2) *
      util::ScaleSizeX(src_format.width, src_format.chroma_format, comp);
    pic8 = CopyFromBytesFast(comp, pic8, src_stride, src_format.bitdepth,
                             &temp_pic);
  }
  temp_pic.PadBorder();
  // 2. Resample using padded temp image
  for (int c = 0; c < num_components; c++) {
    YuvComponent comp = YuvComponent(c);
    uint8_t* src =
      reinterpret_cast<uint8_t*>(temp_pic.GetSamplePtr(comp, 0, 0));
    uint8_t* dst =
      reinterpret_cast<uint8_t*>(out_pic->GetSamplePtr(comp, 0, 0));
    resample::Resample<Sample, Sample>
      (dst, out_pic->GetWidth(comp), out_pic->GetHeight(comp),
       out_pic->GetStride(comp), out_pic->GetBitdepth(),
       src, temp_pic.GetWidth(comp), temp_pic.GetHeight(comp),
       temp_pic.GetStride(comp), src_format.bitdepth);
  }
}

uint8_t*
Resampler::CopyToBytesWithShift(YuvComponent comp, const YuvPicture &src_pic,
                                int out_bitdepth, int dither,
                                uint8_t *out8) const {
  const int src_bitdepth = src_pic.GetBitdepth();
  const int out_width = src_pic.GetDisplayWidth(comp);
  const int out_height = src_pic.GetDisplayHeight(comp);
  const Sample *src = src_pic.GetSamplePtr(comp, 0, 0);
  const ptrdiff_t src_stride = src_pic.GetStride(comp);

  if (out_bitdepth > 8) {
    uint16_t *out16 = reinterpret_cast<uint16_t*>(out8);
    if (out_bitdepth == src_bitdepth) {
      for (int y = 0; y < out_height; y++) {
        memcpy(out16, src, out_width * sizeof(Sample));
        out16 += out_width;
        src += src_stride;
      }
    } else if (out_bitdepth > src_bitdepth) {
      int bit_shift = out_bitdepth - src_bitdepth;
      for (int y = 0; y < out_height; y++) {
        for (int x = 0; x < out_width; x++) {
          *out16++ = src[x] << bit_shift;
        }
        src += src_stride;
      }
    } else {
      int bit_shift = src_bitdepth - out_bitdepth;
      Sample sample_max = (1 << out_bitdepth) - 1;
      if (dither) {
        int d = 0;
        int mask = (1 << bit_shift) - 1;
        for (int y = 0; y < out_height; y++) {
          for (int x = 0; x < out_width; x++) {
            d += src[x];
            *out16++ = static_cast<uint16_t>(util::ClipBD(d >> bit_shift,
                                                          sample_max));
            d &= mask;
          }
          src += src_stride;
        }
      } else {
        for (int y = 0; y < out_height; y++) {
          for (int x = 0; x < out_width; x++) {
            *out16++ = static_cast<uint16_t>(util::ClipBD(
              (src[x] + (1 << (bit_shift - 1))) >> bit_shift,
              sample_max));
          }
          src += src_stride;
        }
      }
    }
    return reinterpret_cast<uint8_t*>(out16);
  } else {
    if (src_bitdepth <= 8) {
      if (sizeof(Sample) == 1) {
        for (int y = 0; y < out_height; y++) {
          memcpy(out8, src, out_width * sizeof(Sample));
          out8 += out_width;
          src += src_stride;
        }
      } else {
        for (int y = 0; y < out_height; y++) {
          for (int x = 0; x < out_width; x++) {
            *out8++ = static_cast<uint8_t>(src[x]);
          }
          src += src_stride;
        }
      }
    } else {
      int bit_shift = src_bitdepth - out_bitdepth;
      if (dither) {
        int d = 0;
        int mask = (1 << bit_shift) - 1;
        for (int y = 0; y < out_height; y++) {
          for (int x = 0; x < out_width; x++) {
            d += src[x];
            *out8++ = static_cast<uint8_t>(util::Clip3(d >> bit_shift, 0, 255));
            d &= mask;
          }
          src += src_stride;
        }
      } else {
        for (int y = 0; y < out_height; y++) {
          for (int x = 0; x < out_width; x++) {
            *out8++ = static_cast<uint8_t>(util::Clip3(
              (src[x] + (1 << (bit_shift - 1))) >> bit_shift, 0, 255));
          }
          src += src_stride;
        }
      }
    }
    return out8;
  }
}

void Resampler::CopyToWithResize(const YuvPicture &src_pic,
                                 const PictureFormat &out_fmt,
                                 int dst_bitdepth, uint8_t *out8) const {
  const ChromaFormat src_chroma_fmt = src_pic.GetChromaFormat();
  const int src_bitdepth = src_pic.GetBitdepth();
  const int num_components_out = util::GetNumComponents(out_fmt.chroma_format);

  // Resampling is performed if resolution or chroma format is different
  const int sample_size = (dst_bitdepth > 8 ? 2 : 1);
  for (int c = 0; c < num_components_out; c++) {
    const YuvComponent comp = YuvComponent(c);
    const Sample *src_sample = src_pic.GetSamplePtr(comp, 0, 0);
    const uint8_t *src8 = reinterpret_cast<const uint8_t*>(src_sample);
    const int dst_width =
      util::ScaleSizeX(out_fmt.width, out_fmt.chroma_format, comp);
    const int dst_height =
      util::ScaleSizeY(out_fmt.height, out_fmt.chroma_format, comp);
    if (c < util::GetNumComponents(src_chroma_fmt)) {
      const int src_width = src_pic.GetDisplayWidth(comp);
      const int src_height = src_pic.GetDisplayHeight(comp);
      const ptrdiff_t src_stride = src_pic.GetStride(comp);
      const ptrdiff_t dst_stride = dst_width;
      if (dst_width == src_width && dst_height == src_height) {
        CopyToBytesWithShift(comp, src_pic, dst_bitdepth, out_fmt.dither, out8);
      } else if (comp != YuvComponent::kY &&
                 dst_width == 2 * src_width &&
                 dst_height == 2 * src_height) {
        if (dst_bitdepth > 8) {
          resample::BilinearResample<Sample, uint16_t>
            (out8, dst_width, dst_height, dst_stride, dst_bitdepth,
             src8, src_width, src_height, src_stride, src_bitdepth);
        } else {
          resample::BilinearResample<Sample, uint8_t>
            (out8, dst_width, dst_height, dst_stride, dst_bitdepth,
             src8, src_width, src_height, src_stride, src_bitdepth);
        }
      } else if (dst_bitdepth > 8) {
        resample::Resample<Sample, uint16_t>
          (out8, dst_width, dst_height, dst_stride, dst_bitdepth,
           src8, src_width, src_height, src_stride, src_bitdepth);
      } else {
        resample::Resample<Sample, uint8_t>
          (out8, dst_width, dst_height, dst_stride, dst_bitdepth,
           src8, src_width, src_height, src_stride, src_bitdepth);
      }
    } else {
      // When monochrome is converted to a chroma format with chroma
      // components, all chroma samples are set to same value
      std::memset(out8, 1 << (out_fmt.bitdepth - 1),
                  dst_width * dst_height * sample_size);
    }
    out8 += dst_width * dst_height * sample_size;
  }
}

template <typename T>
void Resampler::ConvertColorSpace(uint8_t *out, int width, int height,
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

void Resampler::ConvertColorSpace8bit709(uint8_t *dst, int width, int height,
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

namespace resample {

static const int kFilterPrecision = 6;
static const int kInternalPrecision = 16;
static const int kPositionPrecision = 15;

static const int16_t kUpsampleFilter[16][8] = {
  { 0,  0,   0, 64,  0,   0,  0,  0 },
  { 0,  1,  -3, 63,  4,  -2,  1,  0 },
  { -1, 2,  -5, 62,  8,  -3,  1,  0 },
  { -1, 3,  -8, 60, 13,  -4,  1,  0 },
  { -1, 4, -10, 58, 17,  -5,  1,  0 },
  { -1, 4, -11, 52, 26,  -8,  3, -1 },
  { -1, 3,  -9, 47, 31, -10,  4, -1 },
  { -1, 4, -11, 45, 34, -10,  4, -1 },
  { -1, 4, -11, 40, 40, -11,  4, -1 },
  { -1, 4, -10, 34, 45, -11,  4, -1 },
  { -1, 4, -10, 31, 47,  -9,  3, -1 },
  { -1, 3,  -8, 26, 52, -11,  4, -1 },
  { 0,  1,  -5, 17, 58, -10,  4, -1 },
  { 0,  1,  -4, 13, 60,  -8,  3, -1 },
  { 0,  1,  -3,  8, 62,  -5,  2, -1 },
  { 0,  1,  -2,  4, 63,  -3,  1,  0 }
};

static const int16_t kDownsampleFilters[8][16][12] = {
  {
    {  0,   0,   0,   0,   0, 128,   0,   0,   0,   0,   0,   0 },
    {  0,   0,   0,   2,  -6, 127,   7,  -2,   0,   0,   0,   0 },
    {  0,   0,   0,   3, -12, 125,  16,  -5,   1,   0,   0,   0 },
    {  0,   0,   0,   4, -16, 120,  26,  -7,   1,   0,   0,   0 },
    {  0,   0,   0,   5, -18, 114,  36, -10,   1,   0,   0,   0 },
    {  0,   0,   0,   5, -20, 107,  46, -12,   2,   0,   0,   0 },
    {  0,   0,   0,   5, -21,  99,  57, -15,   3,   0,   0,   0 },
    {  0,   0,   0,   5, -20,  89,  68, -18,   4,   0,   0,   0 },
    {  0,   0,   0,   4, -19,  79,  79, -19,   4,   0,   0,   0 },
    {  0,   0,   0,   4, -18,  68,  89, -20,   5,   0,   0,   0 },
    {  0,   0,   0,   3, -15,  57,  99, -21,   5,   0,   0,   0 },
    {  0,   0,   0,   2, -12,  46, 107, -20,   5,   0,   0,   0 },
    {  0,   0,   0,   1, -10,  36, 114, -18,   5,   0,   0,   0 },
    {  0,   0,   0,   1,  -7,  26, 120, -16,   4,   0,   0,   0 },
    {  0,   0,   0,   1,  -5,  16, 125, -12,   3,   0,   0,   0 },
    {  0,   0,   0,   0,  -2,   7, 127,  -6,   2,   0,   0,   0 }
  },
  {
    {  0,   2,   0, -14,  33,  86,  33, -14,   0,   2,   0,   0 },
    {  0,   1,   1, -14,  29,  85,  38, -13,  -1,   2,   0,   0 },
    {  0,   1,   2, -14,  24,  84,  43, -12,  -2,   2,   0,   0 },
    {  0,   1,   2, -13,  19,  83,  48, -11,  -3,   2,   0,   0 },
    {  0,   0,   3, -13,  15,  81,  53, -10,  -4,   3,   0,   0 },
    {  0,   0,   3, -12,  11,  79,  57,  -8,  -5,   3,   0,   0 },
    {  0,   0,   3, -11,   7,  76,  62,  -5,  -7,   3,   0,   0 },
    {  0,   0,   3, -10,   3,  73,  65,  -2,  -7,   3,   0,   0 },
    {  0,   0,   3,  -9,   0,  70,  70,   0,  -9,   3,   0,   0 },
    {  0,   0,   3,  -7,  -2,  65,  73,   3, -10,   3,   0,   0 },
    {  0,   0,   3,  -7,  -5,  62,  76,   7, -11,   3,   0,   0 },
    {  0,   0,   3,  -5,  -8,  57,  79,  11, -12,   3,   0,   0 },
    {  0,   0,   3,  -4, -10,  53,  81,  15, -13,   3,   0,   0 },
    {  0,   0,   2,  -3, -11,  48,  83,  19, -13,   2,   1,   0 },
    {  0,   0,   2,  -2, -12,  43,  84,  24, -14,   2,   1,   0 },
    {  0,   0,   2,  -1, -13,  38,  85,  29, -14,   1,   1,   0 }
  },
  {
    {  0,   5,  -6, -10,  37,  76,  37, -10,  -6,   5,   0,   0 },
    {  0,   5,  -4, -11,  33,  76,  40,  -9,  -7,   5,   0,   0 },
    { -1,   5,  -3, -12,  29,  75,  45,  -7,  -8,   5,   0,   0 },
    { -1,   4,  -2, -13,  25,  75,  48,  -5,  -9,   5,   1,   0 },
    { -1,   4,  -1, -13,  22,  73,  52,  -3, -10,   4,   1,   0 },
    { -1,   4,   0, -13,  18,  72,  55,  -1, -11,   4,   2,  -1 },
    { -1,   4,   1, -13,  14,  70,  59,   2, -12,   3,   2,  -1 },
    { -1,   3,   1, -13,  11,  68,  62,   5, -12,   3,   2,  -1 },
    { -1,   3,   2, -13,   8,  65,  65,   8, -13,   2,   3,  -1 },
    { -1,   2,   3, -12,   5,  62,  68,  11, -13,   1,   3,  -1 },
    { -1,   2,   3, -12,   2,  59,  70,  14, -13,   1,   4,  -1 },
    { -1,   2,   4, -11,  -1,  55,  72,  18, -13,   0,   4,  -1 },
    {  0,   1,   4, -10,  -3,  52,  73,  22, -13,  -1,   4,  -1 },
    {  0,   1,   5,  -9,  -5,  48,  75,  25, -13,  -2,   4,  -1 },
    {  0,   0,   5,  -8,  -7,  45,  75,  29, -12,  -3,   5,  -1 },
    {  0,   0,   5,  -7,  -9,  40,  76,  33, -11,  -4,   5,   0 },
  },
  {
    {  2,  -3,  -9,   6,  39,  58,  39,  6,   -9,  -3,   2,   0 },
    {  2,  -3,  -9,   4,  38,  58,  43,  7,   -9,  -4,   1,   0 },
    {  2,  -2,  -9,   2,  35,  58,  44,  9,   -8,  -4,   1,   0 },
    {  1,  -2,  -9,   1,  34,  58,  46,  11,  -8,  -5,   1,   0 },
    {  1,  -1,  -8,  -1,  31,  57,  47,  13,  -7,  -5,   1,   0 },
    {  1,  -1,  -8,  -2,  29,  56,  49,  15,  -7,  -6,   1,   1 },
    {  1,   0,  -8,  -3,  26,  55,  51,  17,  -7,  -6,   1,   1 },
    {  1,   0,  -7,  -4,  24,  54,  52,  19,  -6,  -7,   1,   1 },
    {  1,   0,  -7,  -5,  22,  53,  53,  22,  -5,  -7,   0,   1 },
    {  1,   1,  -7,  -6,  19,  52,  54,  24,  -4,  -7,   0,   1 },
    {  1,   1,  -6,  -7,  17,  51,  55,  26,  -3,  -8,   0,   1 },
    {  1,   1,  -6,  -7,  15,  49,  56,  29,  -2,  -8,  -1,   1 },
    {  0,   1,  -5,  -7,  13,  47,  57,  31,  -1,  -8,  -1,   1 },
    {  0,   1,  -5,  -8,  11,  46,  58,  34,   1,  -9,  -2,   1 },
    {  0,   1,  -4,  -8,   9,  44,  58,  35,   2,  -9,  -2,   2 },
    {  0,   1,  -4,  -9,   7,  43,  58,  38,   4,  -9,  -3,   2 },
  },
  {
    { -2,  -7,   0,  17,  35,  43,  35,  17,   0,  -7,  -5,   2 },
    { -2,  -7,  -1,  16,  34,  43,  36,  18,   1,  -7,  -5,   2 },
    { -1,  -7,  -1,  14,  33,  43,  36,  19,   1,  -6,  -5,   2 },
    { -1,  -7,  -2,  13,  32,  42,  37,  20,   3,  -6,  -5,   2 },
    {  0,  -7,  -3,  12,  31,  42,  38,  21,   3,  -6,  -5,   2 },
    {  0,  -7,  -3,  11,  30,  42,  39,  23,   4,  -6,  -6,   1 },
    {  0,  -7,  -4,  10,  29,  42,  40,  24,   5,  -6,  -6,   1 },
    {  1,  -7,  -4,   9,  27,  41,  40,  25,   6,  -5,  -6,   1 },
    {  1,  -6,  -5,   7,  26,  41,  41,  26,   7,  -5,  -6,   1 },
    {  1,  -6,  -5,   6,  25,  40,  41,  27,   9,  -4,  -7,   1 },
    {  1,  -6,  -6,   5,  24,  40,  42,  29,  10,  -4,  -7,   0 },
    {  1,  -6,  -6,   4,  23,  39,  42,  30,  11,  -3,  -7,   0 },
    {  2,  -5,  -6,   3,  21,  38,  42,  31,  12,  -3,  -7,   0 },
    {  2,  -5,  -6,   3,  20,  37,  42,  32,  13,  -2,  -7,  -1 },
    {  2,  -5,  -6,   1,  19,  36,  43,  33,  14,  -1,  -7,  -1 },
    {  2,  -5,  -7,   1,  18,  36,  43,  34,  16,  -1,  -7,  -2 }
  },
  {
    { -6,  -3,   5,  19,  31,  36,  31,  19,   5,  -3,  -6,   0 },
    { -6,  -4,   4,  18,  31,  37,  32,  20,   6,  -3,  -6,  -1 },
    { -6,  -4,   4,  17,  30,  36,  33,  21,   7,  -3,  -6,  -1 },
    { -5,  -5,   3,  16,  30,  36,  33,  22,   8,  -2,  -6,  -2 },
    { -5,  -5,   2,  15,  29,  36,  34,  23,   9,  -2,  -6,  -2 },
    { -5,  -5,   2,  15,  28,  36,  34,  24,  10,  -2,  -6,  -3 },
    { -4,  -5,   1,  14,  27,  36,  35,  24,  10,  -1,  -6,  -3 },
    { -4,  -5,   0,  13,  26,  35,  35,  25,  11,   0,  -5,  -3 },
    { -4,  -6,   0,  12,  26,  36,  36,  26,  12,   0,  -6,  -4 },
    { -3,  -5,   0,  11,  25,  35,  35,  26,  13,   0,  -5,  -4 },
    { -3,  -6,  -1,  10,  24,  35,  36,  27,  14,   1,  -5,  -4 },
    { -3,  -6,  -2,  10,  24,  34,  36,  28,  15,   2,  -5,  -5 },
    { -2,  -6,  -2,   9,  23,  34,  36,  29,  15,   2,  -5,  -5 },
    { -2,  -6,  -2,   8,  22,  33,  36,  30,  16,   3,  -5,  -5 },
    { -1,  -6,  -3,   7,  21,  33,  36,  30,  17,   4,  -4,  -6 },
    { -1,  -6,  -3,   6,  20,  32,  37,  31,  18,   4,  -4,  -6 }
  },
  {
    { -9,   0,   9,  20,  28,  32,  28,  20,   9,   0,  -9,   0 },
    { -9,   0,   8,  19,  28,  32,  29,  20,  10,   0,  -4,  -5 },
    { -9,  -1,   8,  18,  28,  32,  29,  21,  10,   1,  -4,  -5 },
    { -9,  -1,   7,  18,  27,  32,  30,  22,  11,   1,  -4,  -6 },
    { -8,  -2,   6,  17,  27,  32,  30,  22,  12,   2,  -4,  -6 },
    { -8,  -2,   6,  16,  26,  32,  31,  23,  12,   2,  -4,  -6 },
    { -8,  -2,   5,  16,  26,  31,  31,  23,  13,   3,  -3,  -7 },
    { -8,  -3,   5,  15,  25,  31,  31,  24,  14,   4,  -3,  -7 },
    { -7,  -3,   4,  14,  25,  31,  31,  25,  14,   4,  -3,  -7 },
    { -7,  -3,   4,  14,  24,  31,  31,  25,  15,   5,  -3,  -8 },
    { -7,  -3,   3,  13,  23,  31,  31,  26,  16,   5,  -2,  -8 },
    { -6,  -4,   2,  12,  23,  31,  32,  26,  16,   6,  -2,  -8 },
    { -6,  -4,   2,  12,  22,  30,  32,  27,  17,   6,  -2,  -8 },
    { -6,  -4,   1,  11,  22,  30,  32,  27,  18,   7,  -1,  -9 },
    { -5,  -4,   1,  10,  21,  29,  32,  28,  18,   8,  -1,  -9 },
    { -5,  -4,   0,  10,  20,  29,  32,  28,  19,   8,   0,  -9 }
  },
  {
    { -8,   7,  13,  18,  22,  24,  22,  18,  13,   7,   2, -10 },
    { -8,   7,  13,  18,  22,  23,  22,  19,  13,   7,   2, -10 },
    { -8,   6,  12,  18,  22,  23,  22,  19,  14,   8,   2, -10 },
    { -9,   6,  12,  17,  22,  23,  23,  19,  14,   8,   3, -10 },
    { -9,   6,  12,  17,  21,  23,  23,  19,  14,   9,   3, -10 },
    { -9,   5,  11,  17,  21,  23,  23,  20,  15,   9,   3, -10 },
    { -9,   5,  11,  16,  21,  23,  23,  20,  15,   9,   4, -10 },
    { -9,   5,  10,  16,  21,  23,  23,  20,  15,  10,   4, -10 },
    {-10,   5,  10,  16,  20,  23,  23,  20,  16,  10,   5, -10 },
    {-10,   4,  10,  15,  20,  23,  23,  21,  16,  10,   5,  -9 },
    {-10,   4,   9,  15,  20,  23,  23,  21,  16,  11,   5,  -9 },
    {-10,   3,   9,  15,  20,  23,  23,  21,  17,  11,   5,  -9 },
    {-10,   3,   9,  14,  19,  23,  23,  21,  17,  12,   6,  -9 },
    {-10,   3,   8,  14,  19,  23,  23,  22,  17,  12,   6,  -9 },
    {-10,   2,   8,  14,  19,  22,  23,  22,  18,  12,   6,  -8 },
    {-10,   2,   7,  13,  19,  22,  23,  22,  18,  13,   7,  -8 }
  }
};

static int GetFilterFromScale(int scale) {
  int filter = 0;
  if (scale > 245760) {
    filter = 7;
  } else if (scale > 187245) {
    filter = 6;
  } else if (scale > 163840) {
    filter = 5;
  } else if (scale > 131072) {
    filter = 4;
  } else if (scale > 109226) {
    filter = 3;
  } else if (scale > 81920) {
    filter = 2;
  } else if (scale > 68985) {
    filter = 1;
  }
  return filter;
}

template <typename T>
static uint16_t FilterHor(const T* src, int sub_pel, int shift,
                          int scale_factor) {
  int sum = 0;
  if (scale_factor < 65536) {
    // Upsampling.
    for (int i = 0; i < 8; i++) {
      sum += src[i - 3] * kUpsampleFilter[sub_pel][i];
    }
  } else if (scale_factor == 65536) {
    // No resampling.
    sum += src[0] << 6;
  } else {
    // Downsampling.
    int filter = GetFilterFromScale(scale_factor);
    for (int i = 0; i < 12; i++) {
      sum += src[i - 5] * kDownsampleFilters[filter][sub_pel][i];
    }
    sum >>= 1;
  }
  uint16_t max_value = std::numeric_limits<uint16_t>::max();
  return util::Clip3<uint16_t>(sum >> shift, 0, max_value);
}

template <typename T>
static T FilterVer(const uint16_t* src, int sub_pel, int shift, int stride,
                   T max, int scale_factor) {
  int sum = 0;
  if (scale_factor < 65536) {
    // Upsampling.
    for (int i = 0; i < 8; i++) {
      sum += src[(i - 3) * stride] * kUpsampleFilter[sub_pel][i];
    }
  } else if (scale_factor == 65536) {
    // No resampling.
    sum += src[0] << 6;
  } else {
    // Downsampling.
    int filter = GetFilterFromScale(scale_factor);
    for (int i = 0; i < 12; i++) {
      sum += src[(i - 5) * stride] * kDownsampleFilters[filter][sub_pel][i];
    }
    sum >>= 1;
  }
  return util::Clip3<T>(sum >> shift, 0, max);
}

template <typename T, typename U>
void Resample(uint8_t *dst_start, int dst_width, int dst_height,
              ptrdiff_t dst_stride, int dst_bitdepth,
              const uint8_t *src_start, int src_width, int src_height,
              ptrdiff_t src_stride, int src_bitdepth) {
  const T* src = reinterpret_cast<const T*>(src_start);
  U* dst = reinterpret_cast<U *>(dst_start);

  int tmp_pad = 8;
  int tmp_width = dst_width;
  int tmp_height = src_height;
  std::vector<uint16_t> tmp_bytes;
  tmp_bytes.resize((tmp_height + 2 * tmp_pad) * tmp_width);
  uint16_t* tmp = &tmp_bytes[0];

  int scale_x =
    ((src_width << kPositionPrecision) + (dst_width >> 1)) / dst_width;
  int shift_hor =
    std::max(src_bitdepth - (kInternalPrecision - kFilterPrecision), 0);

  // Horizontal filtering from src to tmp.
  for (int i = -tmp_pad; i < tmp_height + tmp_pad; i++) {
    for (int j = 0; j < tmp_width; j++) {
      int pos_x = (j * scale_x) >> (kPositionPrecision - 4);
      int sub_pel = pos_x & 15;
      int full_pel = pos_x >> 4;
      *tmp++ = FilterHor<T>(&src[i * src_stride + full_pel], sub_pel, shift_hor,
                            scale_x);
    }
  }

  int scale_y =
    ((src_height << kPositionPrecision) + (dst_height >> 1)) / dst_height;
  int shift_ver =
    2 * kFilterPrecision - shift_hor + src_bitdepth - dst_bitdepth;
  U max = (1 << dst_bitdepth) - 1;

  // Vertical filtering from tmp to dst.
  for (int i = 0; i < dst_height; i++) {
    int pos_y = (i * scale_y) >> (kPositionPrecision - 4);
    int sub_pel = pos_y & 15;
    int full_pel = pos_y >> 4;
    tmp = &tmp_bytes[(tmp_pad + full_pel) * tmp_width];
    for (int j = 0; j < dst_width; j++) {
      dst[j] = FilterVer<U>(tmp++, sub_pel, shift_ver, tmp_width, max, scale_y);
    }
    dst += dst_stride;
  }
}

template void Resample<uint8_t, uint8_t>(uint8_t *dst_start, int dst_width,
                                         int dst_height, ptrdiff_t dst_stride,
                                         int dst_bitdepth,
                                         const uint8_t *src_start,
                                         int src_width, int src_height,
                                         ptrdiff_t src_stride,
                                         int src_bitdepth);

template void Resample<uint16_t, uint8_t>(uint8_t *dst_start, int dst_width,
                                          int dst_height, ptrdiff_t dst_stride,
                                          int dst_bitdepth,
                                          const uint8_t *src_start,
                                          int src_width, int src_height,
                                          ptrdiff_t src_stride,
                                          int src_bitdepth);

template void Resample<uint8_t, uint16_t>(uint8_t *dst_start, int dst_width,
                                          int dst_height, ptrdiff_t dst_stride,
                                          int dst_bitdepth,
                                          const uint8_t *src_start,
                                          int src_width, int src_height,
                                          ptrdiff_t src_stride,
                                          int src_bitdepth);

template void Resample<uint16_t, uint16_t>(uint8_t *dst_start, int dst_width,
                                           int dst_height, ptrdiff_t dst_stride,
                                           int dst_bitdepth,
                                           const uint8_t *src_start,
                                           int src_width, int src_height,
                                           ptrdiff_t src_stride,
                                           int src_bitdepth);

template <typename T, typename U>
void BilinearResample(uint8_t *dst_start, int dst_width, int dst_height,
                      ptrdiff_t dst_stride, int dst_bitdepth,
                      const uint8_t *src_start, int src_width, int src_height,
                      ptrdiff_t src_stride, int src_bitdepth) {
  const T* src = reinterpret_cast<const T*>(src_start);
  U* dst = reinterpret_cast<U *>(dst_start);

  int shift = dst_bitdepth - src_bitdepth;
  if (shift > 0) {
    for (int i = 0; i < src_height; i++) {
      for (int j = 0; j < src_width; j++) {
        dst[2 * j] = static_cast<U>(src[j] << shift);
        dst[2 * j + 1] = static_cast<U>((src[j] + src[j + 1]) << (shift - 1));
        dst[2 * j + dst_stride] =
          static_cast<U>((src[j] + src[j + src_stride]) << (shift - 1));
        dst[2 * j + dst_stride + 1] =
          static_cast<U>((src[j] + src[j + 1] + src[j + src_stride] + src[j +
                          src_stride + 1] + 2) << (shift - 2));
      }
      dst += 2 * dst_stride;
      src += src_stride;
    }
  } else {
    shift = -shift;
    for (int i = 0; i < src_height; i++) {
      for (int j = 0; j < src_width; j++) {
        dst[2 * j] = static_cast<U>(src[j] >> shift);
        dst[2 * j + 1] = static_cast<U>((src[j] + src[j + 1]) >> (shift + 1));
        dst[2 * j + dst_stride] =
          static_cast<U>((src[j] + src[j + src_stride]) >> (shift + 1));
        dst[2 * j + dst_stride + 1] =
          static_cast<U>((src[j] + src[j + 1] + src[j + src_stride] + src[j +
                          src_stride + 1] + 2) >> (shift + 2));
      }
      dst += 2 * dst_stride;
      src += src_stride;
    }
  }
}

template void BilinearResample<Sample, uint8_t>(uint8_t *dst_start,
                                                int dst_width,
                                                int dst_height,
                                                ptrdiff_t dst_stride,
                                                int dst_bitdepth,
                                                const uint8_t *src_start,
                                                int src_width, int src_height,
                                                ptrdiff_t src_stride,
                                                int src_bitdepth);

template void BilinearResample<Sample, uint16_t>(uint8_t *dst_start,
                                                 int dst_width,
                                                 int dst_height,
                                                 ptrdiff_t dst_stride,
                                                 int dst_bitdepth,
                                                 const uint8_t *src_start,
                                                 int src_width, int src_height,
                                                 ptrdiff_t src_stride,
                                                 int src_bitdepth);

}   // namespace resample

}   // namespace xvc
