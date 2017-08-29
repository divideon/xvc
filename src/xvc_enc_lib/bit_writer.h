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
