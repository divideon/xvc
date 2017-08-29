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
