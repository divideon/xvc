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

#ifndef XVC_COMMON_LIB_UTILS_MD5_H_
#define XVC_COMMON_LIB_UTILS_MD5_H_

#include <stdint.h>

namespace xvc {
namespace util {

class MD5 {
public:
  MD5();
  void Reset();
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
