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
