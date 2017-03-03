/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/bit_writer.h"

#include <cassert>

namespace xvc {

void BitWriter::WriteBit(uint32_t bit_val) {
  if (!shift_) {
    buffer_.push_back(0);
    shift_ = 8;
  }
  buffer_.back() |= (bit_val << --shift_);
}

void BitWriter::WriteBits(uint32_t bits, int num_bits) {
  uint32_t mask = 1 << (num_bits - 1);
  while (mask) {
    WriteBit((bits & (mask)) ? 1 : 0);
    mask >>= 1;
  }
}

void BitWriter::PadZeroBits() {
  if (shift_) {
    shift_ = 0;
  }
}

void BitWriter::WriteByte(uint8_t byte) {
  assert(!shift_);
  buffer_.push_back(byte);
}

void BitWriter::WriteBytes(const uint8_t *byte, size_t num_bytes) {
  buffer_.insert(buffer_.end(), byte, byte + num_bytes);
}

}   // namespace xvc
