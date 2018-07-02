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

#include "xvc_dec_lib/bit_reader.h"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <stdexcept>

namespace xvc {

size_t BitReader::GetPosition() const {
  assert(bit_mask_ == 0x80);
  return consumed_;
}

int BitReader::ReadBit() {
  int val = buffer_[consumed_] & bit_mask_;
  bit_mask_ >>= 1;
  if (!bit_mask_) {
    bit_mask_ = 0x80;
    assert(consumed_ < length_);
    if (consumed_ < length_) {
      consumed_++;
    }
  }
  return val ? 1 : 0;
}

uint32_t BitReader::ReadBits(int num_bits) {
  uint32_t bits = 0;
  while (num_bits) {
    bits |= ReadBit() << (num_bits - 1);
    num_bits--;
  }
  return bits;
}

void BitReader::SkipBits() {
  if (bit_mask_ != 0x80) {
    bit_mask_ = 0x80;
    assert(consumed_ < length_);
    if (consumed_ < length_) {
      consumed_++;
    }
  }
}

uint8_t BitReader::ReadByte() {
  assert(consumed_ < length_);
  if (consumed_ >= length_) {
    throw std::runtime_error("corrupt bitstream");
  }
  return buffer_[consumed_++];
}

void BitReader::ReadBytes(uint8_t *bytes, size_t len) {
  assert(consumed_ < length_);
  assert(consumed_ + len <= length_);
  size_t tocopy = std::min(len, length_ - consumed_);
  std::memcpy(bytes, &buffer_[consumed_], tocopy);
  consumed_ += tocopy;
}

void BitReader::Rewind(int num_bits) {
  while (num_bits--) {
    bit_mask_ <<= 1;
    if (bit_mask_ == 0x100) {
      bit_mask_ = 0x1;
      assert(consumed_ > 0);
      --consumed_;
    }
  }
}

}   // namespace xvc
