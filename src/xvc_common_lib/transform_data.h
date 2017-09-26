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

#ifndef XVC_COMMON_LIB_TRANSFORM_DATA_H_
#define XVC_COMMON_LIB_TRANSFORM_DATA_H_

#include <array>

namespace xvc {

class TransformData {
public:
  // DCT-2 (6 bits precision)
  static const int16_t kDct2Transform4[4][4];
  static const int16_t kDct2Transform8[8][8];
  static const int16_t kDct2Transform16[16][16];
  static const int16_t kDct2Transform32[32][32];
  // DCT-2 (8 bits precision)
  static const int16_t kDct2Transform2High[2][2];
  static const int16_t kDct2Transform4High[4][4];
  static const int16_t kDct2Transform8High[8][8];
  static const int16_t kDct2Transform16High[16][16];
  static const int16_t kDct2Transform32High[32][32];
  static const int16_t kDct2Transform64High[64][64];
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_TRANSFORM_DATA_H_
