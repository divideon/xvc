/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_UTILS_MD5_H_
#define XVC_COMMON_LIB_UTILS_MD5_H_

#include <stdint.h>

namespace xvc {
namespace util {

class MD5 {
public:
  MD5();
  void Update(const uint8_t *buf, uint32_t len);
  void Final(uint8_t digest[16]);

private:
  uint32_t buf_[4];
  uint32_t bits_[2];
  uint32_t state_[16];
};

}   // namespace util
}   // namespace xvc

#endif  // XVC_COMMON_LIB_UTILS_MD5_H_
