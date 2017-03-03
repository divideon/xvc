/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_CONTEXT_MODEL_H_
#define XVC_COMMON_LIB_CONTEXT_MODEL_H_

#include "xvc_common_lib/common.h"

namespace xvc {

class ContextModel {
public:
  static const int CONTEXT_STATE_BITS = 6;
  static uint32_t GetEntropyBitsTrm(uint32_t bin) {
    return kEntropyBits_[126 ^ bin];
  }

  ContextModel() : state_(0) {}
  void SetState(uint8_t state, uint8_t mps) { state_ = (state << 1) + mps; }
  void Init(int qp, int init_value);

  uint32_t GetState() { return (state_ >> 1); }
  uint32_t GetMps() { return (state_ & 1); }
  uint32_t GetEntropyBits(uint32_t bin) { return kEntropyBits_[state_ ^ bin]; }

  void UpdateLPS();
  void UpdateMPS();

private:
  static const int kNumTotalStates_ = 1 << (CONTEXT_STATE_BITS + 1);
  static const uint8_t kNextStateMps_[kNumTotalStates_];
  static const uint8_t kNextStateLps_[kNumTotalStates_];
  static const uint32_t kEntropyBits_[ContextModel::kNumTotalStates_];

  uint8_t state_;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_CONTEXT_MODEL_H_
