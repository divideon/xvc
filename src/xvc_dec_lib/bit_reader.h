/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
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
