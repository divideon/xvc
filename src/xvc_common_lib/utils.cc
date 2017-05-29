/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/utils.h"

namespace xvc {

namespace util {

// TODO(dev) this should really be a lookup table
int SizeToLog2(int size) {
  int log2 = 1;
  while ((1 << log2) < size)
    log2++;
  return log2;
}

// TODO(dev) this should really be a lookup table
int SizeLog2Bits(int size) {
  int log2 = 1;
  while ((1 << log2) < size)
    log2++;
  return log2 - 2;
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
