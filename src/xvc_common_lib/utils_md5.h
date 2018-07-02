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
