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

#ifndef XVC_COMMON_LIB_TRANSFORM_DATA_H_
#define XVC_COMMON_LIB_TRANSFORM_DATA_H_

#include <stdint.h>
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
  // DCT-5 (8 bits precision)
  static const int16_t kDct5Transform4High[4 * 4];
  static const int16_t kDct5Transform8High[8 * 8];
  static const int16_t kDct5Transform16High[16 * 16];
  static const int16_t kDct5Transform32High[32 * 32];
  static const int16_t kDct5Transform64High[64 * 64];
  // DCT-8 (8 bits precision)
  static const int16_t kDct8Transform4High[4 * 4];
  static const int16_t kDct8Transform8High[8 * 8];
  static const int16_t kDct8Transform16High[16 * 16];
  static const int16_t kDct8Transform32High[32 * 32];
  static const int16_t kDct8Transform64High[64 * 64];
  // DST-1 (8 bits precision)
  static const int16_t kDst1Transform4High[4 * 4];
  static const int16_t kDst1Transform8High[8 * 8];
  static const int16_t kDst1Transform16High[16 * 16];
  static const int16_t kDst1Transform32High[32 * 32];
  static const int16_t kDst1Transform64High[64 * 64];
  // DST-7 (8 bits precision)
  static const int16_t kDst7Transform4High[4 * 4];
  static const int16_t kDst7Transform8High[8 * 8];
  static const int16_t kDst7Transform16High[16 * 16];
  static const int16_t kDst7Transform32High[32 * 32];
  static const int16_t kDst7Transform64High[64 * 64];
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_TRANSFORM_DATA_H_
