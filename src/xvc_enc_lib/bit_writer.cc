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
