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
