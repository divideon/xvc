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
