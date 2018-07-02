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

#include "xvc_common_lib/utils.h"

namespace xvc {

namespace util {

// TODO(dev) this should really be a lookup table
int SizeToLog2(int size) {
  int log2 = 1;
  while ((1 << log2) < size) {
    log2++;
  }
  return log2;
}

// TODO(dev) this should really be a lookup table
int SizeLog2Bits(int size) {
  int log2 = 1;
  while ((1 << log2) < size) {
    log2++;
  }
  return log2 - 2;
}

int Log2Floor(int x) {
  int log2 = 0;
  while (x > 1) {
    log2++;
    x >>= 1;
  }
  return log2;
}

int GetChromaShiftX(ChromaFormat chroma_format) {
  switch (chroma_format) {
    case ChromaFormat::kMonochrome:
      return 8;
      break;
    case ChromaFormat::k420:
      return 1;
      break;
    case ChromaFormat::k422:
      return 1;
      break;
    case ChromaFormat::k444:
      return 0;
      break;
    case ChromaFormat::kUndefined:
    default:
      assert(0);
      return 0;
      break;
  }
}

int GetChromaShiftY(ChromaFormat chroma_format) {
  switch (chroma_format) {
    case ChromaFormat::kMonochrome:
      return 8;
      break;
    case ChromaFormat::k420:
      return 1;
      break;
    case ChromaFormat::k422:
      return 0;
      break;
    case ChromaFormat::k444:
      return 0;
      break;
    case ChromaFormat::kUndefined:
    default:
      assert(0);
      return 0;
      break;
  }
}

int ScaleChromaX(int size, ChromaFormat chroma_format) {
  switch (chroma_format) {
    case ChromaFormat::kMonochrome:
      return 0;
      break;
    case ChromaFormat::k420:
      return size >> 1;
      break;
    case ChromaFormat::k422:
      return size >> 1;
      break;
    case ChromaFormat::k444:
      return size;
      break;
    case ChromaFormat::kArgb:
      return size;
      break;
    case ChromaFormat::kUndefined:
    default:
      assert(0);
      return 0;
      break;
  }
}

int ScaleChromaY(int size, ChromaFormat chroma_format) {
  switch (chroma_format) {
    case ChromaFormat::kMonochrome:
      return 0;
      break;
    case ChromaFormat::k420:
      return size >> 1;
      break;
    case ChromaFormat::k422:
      return size;
      break;
    case ChromaFormat::k444:
      return size;
      break;
    case ChromaFormat::kArgb:
      return size;
      break;
    case ChromaFormat::kUndefined:
    default:
      assert(0);
      return 0;
      break;
  }
}

}  // namespace util

}  // namespace xvc
