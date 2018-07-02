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

#include "xvc_common_lib/context_model.h"

#include <algorithm>

#include "xvc_common_lib/restrictions.h"

namespace xvc {

void ContextModel::Init(int qp, int init_value) {
  int slope = (init_value >> 4) * 5 - 45;
  int offset = ((init_value & 15) << 3) - 16;
  int init_state = std::min(std::max(1, (((slope * qp) >> 4) + offset)), 126);
  uint32_t mps_state = (init_state >= 64);
  state_ = static_cast<uint8_t>(
    ((mps_state ? (init_state - 64) : (63 - init_state)) << 1) + mps_state);
}

void ContextModel::UpdateLPS() {
  if (!Restrictions::Get().disable_cabac_ctx_update) {
    state_ = kNextStateLps_[state_];
  }
}

void ContextModel::UpdateMPS() {
  if (!Restrictions::Get().disable_cabac_ctx_update) {
    state_ = kNextStateMps_[state_];
  }
}

const std::array<uint8_t, 2 * ContextModel::kNumCtxStates>
ContextModel::kNextStateMps_ = { {
  2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17,
  18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33,
  34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49,
  50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65,
  66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81,
  82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97,
  98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113,
  114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 124, 125, 126, 127
} };

const std::array<uint8_t, 2 * ContextModel::kNumCtxStates>
ContextModel::kNextStateLps_ = { {
  1, 0, 0, 1, 2, 3, 4, 5, 4, 5, 8, 9, 8, 9, 10, 11,
  12, 13, 14, 15, 16, 17, 18, 19, 18, 19, 22, 23, 22, 23, 24, 25,
  26, 27, 26, 27, 30, 31, 30, 31, 32, 33, 32, 33, 36, 37, 36, 37,
  38, 39, 38, 39, 42, 43, 42, 43, 44, 45, 44, 45, 46, 47, 48, 49,
  48, 49, 50, 51, 52, 53, 52, 53, 54, 55, 54, 55, 56, 57, 58, 59,
  58, 59, 60, 61, 60, 61, 60, 61, 62, 63, 64, 65, 64, 65, 66, 67,
  66, 67, 66, 67, 68, 69, 68, 69, 70, 71, 70, 71, 70, 71, 72, 73,
  72, 73, 72, 73, 74, 75, 74, 75, 74, 75, 76, 77, 76, 77, 126, 127
} };

const std::array<uint32_t, 2 * ContextModel::kNumCtxStates>
ContextModel::kEntropyBits_ = { {
  0x07b23, 0x085f9, 0x074a0, 0x08cbc, 0x06ee4, 0x09354, 0x067f4, 0x09c1b,
  0x060b0, 0x0a62a, 0x05a9c, 0x0af5b, 0x0548d, 0x0b955, 0x04f56, 0x0c2a9,
  0x04a87, 0x0cbf7, 0x045d6, 0x0d5c3, 0x04144, 0x0e01b, 0x03d88, 0x0e937,
  0x039e0, 0x0f2cd, 0x03663, 0x0fc9e, 0x03347, 0x10600, 0x03050, 0x10f95,
  0x02d4d, 0x11a02, 0x02ad3, 0x12333, 0x0286e, 0x12cad, 0x02604, 0x136df,
  0x02425, 0x13f48, 0x021f4, 0x149c4, 0x0203e, 0x1527b, 0x01e4d, 0x15d00,
  0x01c99, 0x166de, 0x01b18, 0x17017, 0x019a5, 0x17988, 0x01841, 0x18327,
  0x016df, 0x18d50, 0x015d9, 0x19547, 0x0147c, 0x1a083, 0x0138e, 0x1a8a3,
  0x01251, 0x1b418, 0x01166, 0x1bd27, 0x01068, 0x1c77b, 0x00f7f, 0x1d18e,
  0x00eda, 0x1d91a, 0x00e19, 0x1e254, 0x00d4f, 0x1ec9a, 0x00c90, 0x1f6e0,
  0x00c01, 0x1fef8, 0x00b5f, 0x208b1, 0x00ab6, 0x21362, 0x00a15, 0x21e46,
  0x00988, 0x2285d, 0x00934, 0x22ea8, 0x008a8, 0x239b2, 0x0081d, 0x24577,
  0x007c9, 0x24ce6, 0x00763, 0x25663, 0x00710, 0x25e8f, 0x006a0, 0x26a26,
  0x00672, 0x26f23, 0x005e8, 0x27ef8, 0x005ba, 0x284b5, 0x0055e, 0x29057,
  0x0050c, 0x29bab, 0x004c1, 0x2a674, 0x004a7, 0x2aa5e, 0x0046f, 0x2b32f,
  0x0041f, 0x2c0ad, 0x003e7, 0x2ca8d, 0x003ba, 0x2d323, 0x0010c, 0x3bfbb,
} };

const std::array<uint8_t, 32> ContextModel::kRenormTable_ = { {
  6, 5, 4, 4, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2,
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
} };

const std::array<std::array<uint8_t, 4>, ContextModel::kNumCtxStates>
ContextModel::kRangeTable_ = { {
    { 128, 176, 208, 240 },
    { 128, 167, 197, 227 },
    { 128, 158, 187, 216 },
    { 123, 150, 178, 205 },
    { 116, 142, 169, 195 },
    { 111, 135, 160, 185 },
    { 105, 128, 152, 175 },
    { 100, 122, 144, 166 },
    { 95, 116, 137, 158 },
    { 90, 110, 130, 150 },
    { 85, 104, 123, 142 },
    { 81, 99, 117, 135 },
    { 77, 94, 111, 128 },
    { 73, 89, 105, 122 },
    { 69, 85, 100, 116 },
    { 66, 80, 95, 110 },
    { 62, 76, 90, 104 },
    { 59, 72, 86, 99 },
    { 56, 69, 81, 94 },
    { 53, 65, 77, 89 },
    { 51, 62, 73, 85 },
    { 48, 59, 69, 80 },
    { 46, 56, 66, 76 },
    { 43, 53, 63, 72 },
    { 41, 50, 59, 69 },
    { 39, 48, 56, 65 },
    { 37, 45, 54, 62 },
    { 35, 43, 51, 59 },
    { 33, 41, 48, 56 },
    { 32, 39, 46, 53 },
    { 30, 37, 43, 50 },
    { 29, 35, 41, 48 },
    { 27, 33, 39, 45 },
    { 26, 31, 37, 43 },
    { 24, 30, 35, 41 },
    { 23, 28, 33, 39 },
    { 22, 27, 32, 37 },
    { 21, 26, 30, 35 },
    { 20, 24, 29, 33 },
    { 19, 23, 27, 31 },
    { 18, 22, 26, 30 },
    { 17, 21, 25, 28 },
    { 16, 20, 23, 27 },
    { 15, 19, 22, 25 },
    { 14, 18, 21, 24 },
    { 14, 17, 20, 23 },
    { 13, 16, 19, 22 },
    { 12, 15, 18, 21 },
    { 12, 14, 17, 20 },
    { 11, 14, 16, 19 },
    { 11, 13, 15, 18 },
    { 10, 12, 15, 17 },
    { 10, 12, 14, 16 },
    { 9, 11, 13, 15 },
    { 9, 11, 12, 14 },
    { 8, 10, 12, 14 },
    { 8, 9, 11, 13 },
    { 7, 9, 11, 12 },
    { 7, 9, 10, 12 },
    { 7, 8, 10, 11 },
    { 6, 8, 9, 11 },
    { 6, 7, 9, 10 },
    { 6, 7, 8, 9 },
    { 2, 2, 2, 2 }
} };

}   // namespace xvc
