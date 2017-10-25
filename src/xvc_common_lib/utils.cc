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
    case ChromaFormat::kUndefinedChromaFormat:
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
    case ChromaFormat::kUndefinedChromaFormat:
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
    case ChromaFormat::kUndefinedChromaFormat:
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
    case ChromaFormat::kUndefinedChromaFormat:
    default:
      assert(0);
      return 0;
      break;
  }
}

}  // namespace util

}  // namespace xvc
