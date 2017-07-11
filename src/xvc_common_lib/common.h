/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_COMMON_H_
#define XVC_COMMON_LIB_COMMON_H_

#include <stdint.h>
#include <cstddef>

#ifndef XVC_HIGH_BITDEPTH
#define XVC_HIGH_BITDEPTH 1
#endif

namespace xvc {

#if !XVC_HIGH_BITDEPTH
typedef uint8_t Sample;
#else
typedef uint16_t Sample;
#endif
typedef int16_t Coeff;
typedef int16_t Residual;
typedef uint64_t Cost;
typedef uint64_t Distortion;
typedef uint32_t Bits;
typedef uint64_t PicNum;
typedef uint8_t SegmentNum;

enum class ChromaFormat : uint8_t {
  kMonochrome = 0,
  k420 = 1,
  k422 = 2,
  k444 = 3,
  kArgb = 4,
  kUndefinedChromaFormat = 255,
};

enum class ColorMatrix : uint8_t {
  kUndefinedColorMatrix = 0,
  k601 = 1,
  k709 = 2,
  k2020 = 3,
};

enum YuvComponent {
  kY = 0,
  kU = 1,
  kV = 2,
};

enum class CuTree {
  Primary = 0,
  Secondary = 1,
};

namespace constants {

// xvc version
const uint32_t kXvcCodecIdentifier = 7894627;
const uint32_t kXvcMajorVersion = 1;
const uint32_t kXvcMinorVersion = 0;

// Picture
const int kMaxYuvComponents = 3;
const int kMaxNumCuTrees = 2;

// CU limits
const int kCtuSize = 64;
// CU size and depth for luma
const int kMaxCuDepth = 3;
const int kMaxCuDepthChroma = kMaxCuDepth + 1;
const int kMinCuSize = (kCtuSize >> kMaxCuDepth);
// Binary split
const int kMaxBinarySplitDepth = 3;
const int kMaxBinarySplitSizeInter = 128;
const int kMaxBinarySplitSizeIntra1 = 32;
const int kMaxBinarySplitSizeIntra2 = 16;
const int kMinBinarySplitSize = 4;

// Actual storage required (to allow for deeper chroma CU trees)
const int kMaxBlockSize = kCtuSize;
const int kMaxBlockDepthLuma = kMaxCuDepth + kMaxBinarySplitDepth;
const int kMaxBlockDepthChroma = kMaxCuDepthChroma + kMaxBinarySplitDepth;
const int kMaxBlockDepth = kMaxBlockDepthLuma > kMaxBlockDepthChroma ?
kMaxBlockDepthLuma : kMaxBlockDepthChroma;
const int kMinBlockSize = 4;
const int kMaxBlockSamples = kMaxBlockSize * kMaxBlockSize;

const int kQuadSplit = 4;

// Transform
const int kTransformExtendedPrecision = 2;
const bool kZeroOutHighFreqLargeTransforms = true;

// Prediction
const int kNumIntraMpm = 3;
const int kNumIntraChromaModes = 5;
const int kNumInterMvPredictors = 2;
const int kNumInterMergeCandidates = 5;
const int kMvPrecisionShift = 2;
const bool kTemporalMvPrediction = true;

// Quant
const int kMaxTrDynamicRange = 15;
const int kMinAllowedQp = -64;
const int kMaxAllowedQp = 63;
const int kMaxQpDiff = 16;
const int kQpSignalBase = 64;

// Residual coding
const int kMaxNumC1Flags = 8;
const int kMaxNumC2Flags = 1;
const int kSubblockShift = 2;
const uint32_t kCoeffRemainBinReduction = 3;
const int SignHidingThreshold = 3;

// Deblocking
const int kDeblockOffsetBits = 6;

// Maximum number of reference pictures per reference picture list
const int kMaxNumRefPics = 5;

// High-level syntax
const int kTimeScale = 90000;
const int kMaxTid = 8;
const int kFrameRateBitDepth = 24;
const PicNum kMaxSubGopLength = 64;

// Min and Max
const int16_t kInt16Max = INT16_MAX;
const int16_t kInt16Min = INT16_MIN;

}   // namespace constants

}   // namespace xvc

#endif  // XVC_COMMON_LIB_COMMON_H_
