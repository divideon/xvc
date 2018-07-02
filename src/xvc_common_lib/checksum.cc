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

#include "xvc_common_lib/checksum.h"

#include <cassert>

#include "xvc_common_lib/utils_md5.h"

namespace xvc {

void Checksum::HashPicture(const YuvPicture &pic) {
  switch (method_) {
    case Method::kCrc:
      CalculateCrc(pic, mode_);
      break;

    case Method::kMd5:
      CalculateMd5(pic, mode_);
      break;

    default:
      assert(0);
      break;
  }
}

void Checksum::CalculateCrc(const YuvPicture &pic, Mode mode) {
  assert(mode == Mode::kMinOverhead || mode == Mode::kMaxRobust);
  int num_components = util::GetNumComponents(pic.GetChromaFormat());
  uint32_t crcVal = 0xffff;

  for (int c = 0; c < num_components; c++) {
    YuvComponent comp = YuvComponent(c);
    if (mode == Mode::kMaxRobust) {
      crcVal = 0xffff;
    }
    auto src = pic.GetSamplePtr(comp, 0, 0);
    for (int y = 0; y < pic.GetHeight(comp); y++) {
      for (int x = 0; x < pic.GetWidth(comp); x++) {
        for (int bit = 0; bit < 8; bit++) {
          uint32_t crcMsb = (crcVal >> 15) & 1;
          uint32_t bitVal = (src[x] >> (7 - bit)) & 1;
          crcVal = (((crcVal << 1) + bitVal) & 0xffff) ^ (crcMsb * 0x1021);
        }
        if (pic.GetBitdepth() > 8) {
          for (int bit = 0; bit < 8; bit++) {
            uint32_t crcMsb = (crcVal >> 15) & 1;
            uint32_t bitVal = (src[x] >> (15 - bit)) & 1;
            crcVal = (((crcVal << 1) + bitVal) & 0xffff) ^ (crcMsb * 0x1021);
          }
        }
      }
      src += pic.GetStride(comp);
    }
    // For MaxRobust, one checksum value is calculated for each component.
    if (mode == Mode::kMaxRobust) {
      for (int bit = 0; bit < 16; bit++) {
        uint32_t crcMsb = (crcVal >> 15) & 1;
        crcVal = ((crcVal << 1) & 0xffff) ^ (crcMsb * 0x1021);
      }
      hash_.push_back((crcVal >> 8) & 0xff);
      hash_.push_back(crcVal & 0xff);
    }
  }
  // For MinOverhead, a single checksum value is calculated for the picture.
  if (mode == Mode::kMinOverhead) {
    for (int bit = 0; bit < 16; bit++) {
      uint32_t crcMsb = (crcVal >> 15) & 1;
      crcVal = ((crcVal << 1) & 0xffff) ^ (crcMsb * 0x1021);
    }
    hash_.push_back((crcVal >> 8) & 0xff);
    hash_.push_back(crcVal & 0xff);
  }
}

void Checksum::CalculateMd5(const YuvPicture &pic, Mode mode) {
  assert(mode == Mode::kMinOverhead || mode == Mode::kMaxRobust);
  int num_components = util::GetNumComponents(pic.GetChromaFormat());
  util::MD5 md5;

  for (int c = 0; c < num_components; c++) {
    YuvComponent comp = YuvComponent(c);
    if (mode == Mode::kMaxRobust) {
      md5.Reset();
    }
    auto src = pic.GetSamplePtr(comp, 0, 0);
    int bitdepth = pic.GetBitdepth();
    int width = pic.GetWidth(comp);
    int row = bitdepth == 8 ? width : width << 1;
    const uint8_t* samples;
    std::vector<uint8_t> row_buffer;
    if (bitdepth == 8 && sizeof(Sample) == 2) {
      row_buffer.resize(width);
    }
    for (int y = 0; y < pic.GetHeight(comp); y++) {
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
      src += pic.GetStride(comp);
    }
    // For MaxRobust, one checksum value is calculated for each component.
    if (mode == Mode::kMaxRobust) {
      size_t size = hash_.size();
      hash_.resize(size + 16);
      md5.Final(&hash_[size]);
    }
  }
  // For MinOverhead, a single checksum value is calculated for the picture.
  if (mode == Mode::kMinOverhead) {
    hash_.resize(16);
    md5.Final(&hash_[0]);
  }
}

}   // namespace xvc
