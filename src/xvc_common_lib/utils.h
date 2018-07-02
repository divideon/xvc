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

constexpr int CompToPlane(YuvComponent comp) {
  return IsLuma(comp) ? 0 : 1;
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
int Log2Floor(int x);

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
