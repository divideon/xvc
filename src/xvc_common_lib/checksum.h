/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
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

  Checksum() = default;
  explicit Checksum(const std::vector<uint8_t> &hash)
    : hash_(hash) {}

  void Clear() { hash_.clear(); }
  void HashPicture(const YuvPicture &pic, Method method, Mode mode);
  std::vector<uint8_t> GetHash() const { return hash_; }

private:
  void CalculateCrc(const YuvPicture &pic, Mode mode);
  void CalculateMd5(const YuvPicture &pic, Mode mode);

  std::vector<uint8_t> hash_;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_CHECKSUM_H_
