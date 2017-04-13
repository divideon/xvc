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

namespace xvc {

class Checksum {
public:
  enum class Method {
    kNone = 0,
    kCRC = 1,
    kMD5 = 2,
  };
  enum class Mode {
    kMinOverhead = 0,
    kMaxRobust = 1,
    kTotalNumber = 2,
    kInvalid = 99,
  };
  static const Method kDefaultMethod = Method::kMD5;
  static const Method kFallbackMethod = Method::kCRC;

  explicit Checksum(Method method) : method_(method) {}
  Checksum(Method method, const std::vector<uint8_t> &hash)
    : hash_(hash), method_(method) {}

  void Clear() { hash_.clear(); }
  void HashPicture(const YuvPicture &pic, Mode mode);
  Method GetMethod() const { return method_; }
  std::vector<uint8_t> GetHash() const { return hash_; }

  bool operator==(const Checksum &other) const {
    return hash_ == other.hash_;
  }
  bool operator!=(const Checksum &other) const {
    return !(*this == other);
  }

private:
  void CalculateCRC(const YuvPicture &pic, Mode mode);
  void CalculateMD5(const YuvPicture &pic, Mode mode);

  std::vector<uint8_t> hash_;
  Method method_;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_CHECKSUM_H_
