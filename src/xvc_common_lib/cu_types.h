/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_CU_TYPES_H_
#define XVC_COMMON_LIB_CU_TYPES_H_

namespace xvc {

enum class SplitType {
  kNone,
  kQuad,
  kHorizontal,
  kVertical,
};

enum class PredictionMode {
  kIntra = 0,
  kInter = 1,
};

enum class PartitionType {
  kSize2Nx2N,
  kSize2NxN,
  kSizeNx2N,
  kSizeNxN,
  kSize2NxnU,
  kSize2NxnD,
  kSizenLx2N,
  kSizenRx2N,
  kSizeNONE = 15,
};

enum IntraMode : int {
  kPlanar = 0,
  kDC = 1,
  kHorizontal = 10,
  kVertical = 26,

  kTotalNumber = 35,
  kInvalid = 99,
};

enum class IntraChromaMode : int {
  kPlanar = 0,
  kDC = 1,
  kHorizontal = 10,
  kVertical = 26,
  kVerticalPlus8 = 34,
  kDMChroma = 36,
  kInvalid = 99,
};

enum class InterDir {
  kL0 = 0,
  kL1 = 1,
  kBi = 2,
};

struct MotionVector {
  MotionVector() = default;
  MotionVector(int mv_x, int mv_y) : x(mv_x), y(mv_y) {}
  bool operator==(const MotionVector &other) const {
    return x == other.x && y == other.y;
  }
  bool operator!=(const MotionVector &other) const {
    return !(*this == other);
  }
  int x = 0;
  int y = 0;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_CU_TYPES_H_
