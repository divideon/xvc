/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
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
