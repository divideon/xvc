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

#ifndef XVC_ENC_LIB_BIT_WRITER_H_
#define XVC_ENC_LIB_BIT_WRITER_H_

#include <vector>
#include <cassert>

#include "xvc_common_lib/common.h"

namespace xvc {

class BitWriter {
public:
  BitWriter() : shift_(0) {
  }

  std::vector<uint8_t>* GetBytes() { return &buffer_; }
  void Clear() {
    buffer_.clear();
    assert(!shift_);
  }
  void WriteBit(uint32_t bitval);
  void WriteBits(uint32_t bits, int num_bits);
  void PadZeroBits();
  void WriteByte(uint8_t byte);
  void WriteBytes(const uint8_t *byte, size_t num_bytes);

private:
  int shift_;
  std::vector<uint8_t> buffer_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_BIT_WRITER_H_
