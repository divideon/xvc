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

#ifndef XVC_DEC_LIB_BIT_READER_H_
#define XVC_DEC_LIB_BIT_READER_H_

#include <fstream>
#include <vector>

#include "xvc_common_lib/common.h"

namespace xvc {

class BitReader {
public:
  BitReader(const uint8_t *buffer, size_t length)
    : buffer_(buffer), length_(length) {
  }

  size_t GetPosition() const;
  int ReadBit();
  uint32_t ReadBits(int num_bits);
  void SkipBits();
  uint8_t ReadByte();
  void ReadBytes(uint8_t *bytes, size_t len);
  void Rewind(int num_bits);

private:
  void Refill();

  int bit_mask_ = 0x80;
  size_t consumed_ = 0;
  const uint8_t *buffer_ = nullptr;
  size_t length_ = 0;
};

}   // namespace xvc

#endif    // XVC_DEC_LIB_BIT_READER_H_
