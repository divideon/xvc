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

#ifndef XVC_COMMON_LIB_CONTEXT_MODEL_H_
#define XVC_COMMON_LIB_CONTEXT_MODEL_H_

#include <array>

#include "xvc_common_lib/common.h"

namespace xvc {

class ContextModel {
public:
  static const int kFracBitsPrecision = 15;
  static const uint32_t kEntropyBypassBits = 1 << kFracBitsPrecision;

  void Init(int qp, int init_value);
  void UpdateLPS();
  void UpdateMPS();
  uint32_t GetState() const { return (state_ >> 1); }
  uint32_t GetMps() const { return (state_ & 1); }
  uint32_t GetLps(int range) const {
    return kRangeTable_[state_ >> 1][(range >> 6) & 3];
  }
  uint32_t GetEntropyBits(uint32_t bin) const {
    return kEntropyBits_[state_ ^ bin];
  }
  uint8_t GetRenormBitsLps(uint32_t lps) const {
    return kRenormTable_[lps >> 3];
  }
  static uint32_t GetEntropyBitsTrm(uint32_t bin) {
    return kEntropyBits_[126 ^ bin];
  }

protected:
  static const int kNumCtxStates = 64;
  static const std::array<uint8_t, 2 * kNumCtxStates> kNextStateMps_;
  static const std::array<uint8_t, 2 * kNumCtxStates> kNextStateLps_;
  static const std::array<uint32_t, 2 * kNumCtxStates>  kEntropyBits_;
  static const std::array<uint8_t, 32> kRenormTable_;
  static const std::array<std::array<uint8_t, 4>, kNumCtxStates> kRangeTable_;
  uint8_t state_ = 0;
};

// TODO(PH) Following classes gives performance boost to decoder by not having
// to check restriction flags for every bin but are not used by encoder due to
// adding extra code complexity together with rdo

class ContextModelDynamic : public ContextModel {
public:
  // Override parent methods without virtual to avoid restriction check
  void UpdateLPS() {
    state_ = kNextStateLps_[state_];
  }
  void UpdateMPS() {
    state_ = kNextStateMps_[state_];
  }
};

class ContextModelStatic : public ContextModel {
public:
  // Override parent methods without virtual to avoid restriction check
  void UpdateLPS() {}
  void UpdateMPS() {}
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_CONTEXT_MODEL_H_
