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

#ifndef XVC_COMMON_LIB_UTILS_H_
#define XVC_COMMON_LIB_UTILS_H_

#include <algorithm>
#include <cassert>

#include "xvc_common_lib/common.h"

namespace xvc {

namespace util {

constexpr bool IsLuma(YuvComponent comp) {
  return comp == YuvComponent::kY;
}

constexpr bool IsFirstChroma(YuvComponent comp) {
  return comp == YuvComponent::kU;
}

template <typename T, typename U>
static T Clip3(U value, T min, T max) {
  return static_cast<T>(std::min(std::max(value, static_cast<U>(min)),
                                 static_cast<U>(max)));
}

template <typename T>
static Sample ClipBD(T value, Sample max) {
  if (value < 0) return 0;
  if (value > max) return max;
  return static_cast<Sample>(value);
}

int SizeToLog2(int size);
int SizeLog2Bits(int size);

int GetChromaShiftX(ChromaFormat chroma_format);
int GetChromaShiftY(ChromaFormat chroma_format);

int ScaleChromaX(int size, ChromaFormat chroma_format);
int ScaleChromaY(int size, ChromaFormat chroma_format);

inline int ScaleSizeX(int size, ChromaFormat chroma_format,
                      YuvComponent comp) {
  return IsLuma(comp) ? size : ScaleChromaX(size, chroma_format);
}

inline int ScaleSizeY(int size, ChromaFormat chroma_format,
                      YuvComponent comp) {
  return IsLuma(comp) ? size : ScaleChromaY(size, chroma_format);
}

inline int GetLumaNumSamples(int width, int height) {
  return width * height;
}

inline int GetChromaNumSamples(int width, int height, ChromaFormat chroma_fmt) {
  return ScaleChromaX(width, chroma_fmt) * ScaleChromaY(height, chroma_fmt);
}

inline int GetTotalNumSamples(int width, int height, ChromaFormat chroma_fmt) {
  if (chroma_fmt == ChromaFormat::kArgb) {
    return 4 * GetLumaNumSamples(width, height);
  }
  return GetLumaNumSamples(width, height) +
    (GetChromaNumSamples(width, height, chroma_fmt) << 1);
}

inline int GetNumComponents(ChromaFormat chroma_fmt) {
  return chroma_fmt == ChromaFormat::kMonochrome ? 1 : 3;
}

}   // namespace util

}   // namespace xvc

#endif  // XVC_COMMON_LIB_UTILS_H_
