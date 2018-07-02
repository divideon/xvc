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

#ifndef XVC_COMMON_LIB_CHECKSUM_H_
#define XVC_COMMON_LIB_CHECKSUM_H_

#include <vector>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_common_lib/restrictions.h"

namespace xvc {

class Checksum {
public:
  enum class Method {
    kNone = 0,
    kCrc = 1,
    kMd5 = 2,
  };
  enum class Mode {
    kMinOverhead = 0,
    kMaxRobust = 1,
    kTotalNumber = 2,
    kInvalid = 99,
  };
  static const Method kDefaultMethod = Method::kMd5;
  static const Method kFallbackMethod = Method::kCrc;

  Checksum(Method method, Mode mode) : method_(method), mode_(mode) {}
  void HashPicture(const YuvPicture &pic);
  std::vector<uint8_t> GetHash() const { return hash_; }

private:
  void CalculateCrc(const YuvPicture &pic, Mode mode);
  void CalculateMd5(const YuvPicture &pic, Mode mode);

  const Method method_;
  const Mode mode_;
  std::vector<uint8_t> hash_;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_CHECKSUM_H_
