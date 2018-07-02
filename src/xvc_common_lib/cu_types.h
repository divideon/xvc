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

#ifndef XVC_COMMON_LIB_CU_TYPES_H_
#define XVC_COMMON_LIB_CU_TYPES_H_

namespace xvc {

enum class NeighborDir {
  kAboveLeft,
  kAbove,
  kAboveCorner,
  kAboveRight,
  kLeft,
  kLeftCorner,
  kLeftBelow,
};

enum class SplitType {
  kNone,
  kQuad,
  kHorizontal,
  kVertical,
};

enum class SplitRestriction {
  kNone,
  kNoHorizontal,
  kNoVertical,
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
  kSizeNone = 15,
};

enum class TransformType {
  kDefault,
  kDct2,
  kDct5,
  kDct8,
  kDst1,
  kDst7,
  kTotalNumber,
};
static const int kNbrTransformTypes =
static_cast<int>(TransformType::kTotalNumber);

enum IntraMode : int8_t {
  kLmChroma = -2,
  kInvalid = -1,
  kPlanar = 0,
  kDc = 1,
};
static const int kNbrIntraModes = 35;
static const int kNbrIntraModesExt = 67;

enum class IntraAngle {
  kPlanar = 0,
  kDc = 1,
  kFirst = 2,
  kHorizontal = 10,
  kDiagonal = 18,
  kVertical = 26,
};
static const int kNbrIntraDirs = kNbrIntraModes;

// chroma modes maps directly to intra modes except for kDmChroma
enum class IntraChromaMode {
  kLmChroma = -2,
  kDmChroma = -1,
  kPlanar = 0,
  kDc = 1,
  kInvalid = 99,
};
const int kMaxNumIntraChromaModes = 6;

enum class IntraChromaAngle {
  kDmChroma = -1,
  kPlanar = 0,
  kDc = 1,
  kHorizontal = 10,
  kVertical = 26,
  kVerticalPlus8 = 34,
  kInvalid = 99,
};

enum class InterDir {
  kL0 = 0,
  kL1 = 1,
  kBi = 2,
};

struct MvDelta {
  static const int kPrecisionShift = 2;
  MvDelta() = default;
  MvDelta(int mvd_x, int mvd_y, int prec = kPrecisionShift)
    : x(prec < kPrecisionShift ? mvd_x * (1 << (kPrecisionShift - prec)) :
        mvd_x >> (prec - kPrecisionShift)),
    y(prec < kPrecisionShift ? mvd_y * (1 << (kPrecisionShift - prec)) :
      mvd_y >> (prec - kPrecisionShift)) {
  }
  bool operator==(const MvDelta &other) const {
    return x == other.x && y == other.y;
  }
  bool operator!=(const MvDelta &other) const {
    return !(*this == other);
  }
  bool IsZero() const {
    return x == 0 && y == 0;
  }
  int x = 0;
  int y = 0;
};

using MvDelta2 = std::array<MvDelta, 2>;

struct MvFullpel {
  static const int kPrecisionShift = 0;
  MvFullpel() = default;
  MvFullpel(int mv_x, int mv_y) : x(mv_x), y(mv_y) {}
  bool operator==(const MvFullpel &other) const {
    return x == other.x && y == other.y;
  }
  bool operator!=(const MvFullpel &other) const {
    return !(*this == other);
  }
  int x = 0;
  int y = 0;
};

struct MotionVector {
  static const int kPrecisionShift = 4;
  static const int kNormalPrecision = 2;
  static const int kHighToNormalShiftDelta = kPrecisionShift - kNormalPrecision;
  static const int kScale = 1 << kPrecisionShift;
  MotionVector() = default;
  MotionVector(int mv_x, int mv_y) : x(mv_x), y(mv_y) {}
  explicit MotionVector(const MvFullpel mv_fullpel) :
    x(mv_fullpel.x * kScale),
    y(mv_fullpel.y * kScale) {
  }
  bool operator==(const MotionVector &other) const {
    return x == other.x && y == other.y;
  }
  bool operator!=(const MotionVector &other) const {
    return !(*this == other);
  }
  operator MvFullpel() const {
    return MvFullpel(x >> kPrecisionShift, y >> kPrecisionShift);
  }
  MotionVector& operator+=(const MvDelta &mvd) {
    x += mvd.x * (1 << (kPrecisionShift - MvDelta::kPrecisionShift));
    y += mvd.y * (1 << (kPrecisionShift - MvDelta::kPrecisionShift));
    return *this;
  }
  friend MotionVector operator+(const MotionVector &mv, const MvDelta &mvd) {
    return MotionVector(
      mv.x + mvd.x * (1 << (kPrecisionShift - MvDelta::kPrecisionShift)),
      mv.y + mvd.y * (1 << (kPrecisionShift - MvDelta::kPrecisionShift)));
  }
  friend MvDelta operator-(const MotionVector &mv1, const MotionVector &mv2) {
    return MvDelta(mv1.x - mv2.x, mv1.y - mv2.y, MotionVector::kPrecisionShift);
  }
  void RoundToFullpel() {
    x = ((x + (1 << (kPrecisionShift - 1))) >> kPrecisionShift) * kScale;
    y = ((y + (1 << (kPrecisionShift - 1))) >> kPrecisionShift) * kScale;
  }
  void RoundToNormalPrecision() {
    const int down_shift = kHighToNormalShiftDelta;
    const int up_scale = 1 << kHighToNormalShiftDelta;
    const int offset = 1 << (down_shift - 1);
    x = (x < 0) ? -(((-x + offset) >> down_shift) * up_scale) :
      ((x + offset) >> down_shift) * up_scale;
    y = (y < 0) ? -(((-y + offset) >> down_shift) * up_scale) :
      ((y + offset) >> down_shift) * up_scale;
  }
  int x = 0;
  int y = 0;
};

using MotionVector3 = std::array<MotionVector, 3>;

enum class MvCorner {
  kDefault = 0,
  kUpLeft = 0,
  kUpRight = 1,
  kDownLeft = 2,
  kDownRight = 3,
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_CU_TYPES_H_
