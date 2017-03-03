/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/checksum.h"

#include <cassert>

#include "xvc_common_lib/utils_md5.h"

namespace xvc {

void Checksum::HashPicture(const YuvPicture &pic) {
  switch (method_) {
    case Method::kCRC:
      for (int c = 0; c < constants::kMaxYuvComponents; c++) {
        YuvComponent comp = YuvComponent(c);
        CalculateCRC(pic.GetSamplePtr(comp, 0, 0), pic.GetBitdepth(),
                     pic.GetWidth(comp), pic.GetHeight(comp),
                     pic.GetStride(comp));
      }
      break;

    case Method::kMD5:
      for (int c = 0; c < constants::kMaxYuvComponents; c++) {
        YuvComponent comp = YuvComponent(c);
        CalculateMD5(pic.GetSamplePtr(comp, 0, 0), pic.GetBitdepth(),
                     pic.GetWidth(comp), pic.GetHeight(comp),
                     pic.GetStride(comp));
      }
      break;

    default:
      assert(0);
      break;
  }
}

void Checksum::HashComp(const Sample *src, int width, int height,
                        ptrdiff_t stride, int bitdepth) {
  switch (method_) {
    case Method::kCRC:
      CalculateCRC(src, bitdepth, width, height, stride);
      break;

    case Method::kMD5:
      CalculateMD5(src, bitdepth, width, height, stride);
      break;

    default:
      assert(0);
      break;
  }
}

void Checksum::CalculateCRC(const Sample *src, int bitdepth, int width,
                            int height, ptrdiff_t stride) {
  uint32_t crcVal = 0xffff;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      for (int bit = 0; bit < 8; bit++) {
        uint32_t crcMsb = (crcVal >> 15) & 1;
        uint32_t bitVal = (src[x] >> (7 - bit)) & 1;
        crcVal = (((crcVal << 1) + bitVal) & 0xffff) ^ (crcMsb * 0x1021);
      }
      if (bitdepth > 8) {
        for (int bit = 0; bit < 8; bit++) {
          uint32_t crcMsb = (crcVal >> 15) & 1;
          uint32_t bitVal = (src[x] >> (15 - bit)) & 1;
          crcVal = (((crcVal << 1) + bitVal) & 0xffff) ^ (crcMsb * 0x1021);
        }
      }
    }
    src += stride;
  }

  for (int bit = 0; bit < 16; bit++) {
    uint32_t crcMsb = (crcVal >> 15) & 1;
    crcVal = ((crcVal << 1) & 0xffff) ^ (crcMsb * 0x1021);
  }
  hash_.push_back((crcVal >> 8) & 0xff);
  hash_.push_back(crcVal & 0xff);
}

void Checksum::CalculateMD5(const Sample *src, int bitdepth, int width,
                            int height, ptrdiff_t stride) {
  util::MD5 md5;
  int row = bitdepth == 8 ? width : width << 1;
  const uint8_t* samples;
  std::vector<uint8_t> row_buffer;
  if (bitdepth == 8 && sizeof(Sample) == 2) {
    row_buffer.resize(width);
  }
  for (int y = 0; y < height; y++) {
    if (bitdepth == 8 && sizeof(Sample) == 2) {
      for (int x = 0; x < width; x++) {
        row_buffer[x] = static_cast<uint8_t>(src[x]);
      }
      samples = reinterpret_cast<const uint8_t*>(&row_buffer[0]);
    } else {
      samples = reinterpret_cast<const uint8_t*>(src);
    }
    // TODO(Dev) Consider enforcing little endian for high bitdepth yuv
    md5.Update(samples, row);
    src += stride;
  }
  size_t size = hash_.size();
  hash_.resize(size + 16);
  md5.Final(&hash_[size]);
}

}   // namespace xvc
