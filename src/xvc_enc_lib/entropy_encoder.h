/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_LIB_ENTROPY_ENCODER_H_
#define XVC_ENC_LIB_ENTROPY_ENCODER_H_

#include "xvc_common_lib/context_model.h"
#include "xvc_enc_lib/bit_writer.h"

namespace xvc {

class EntropyEncoder {
public:
  explicit EntropyEncoder(BitWriter *bit_writer);
  EntropyEncoder(BitWriter *bit_writer, uint32_t written_bits,
                 uint32_t fractional_bits);

  void EncodeBin(uint32_t binval, ContextModel *ctx);
  void EncodeBypass(uint32_t binval);
  void EncodeBypassBins(uint32_t binvals, int num_bins);
  void EncodeBinTrm(uint32_t binval);

  void ResetBitCounting() { frac_bits_ &= 32767; }
  void Start();
  void Finish();
  Bits GetNumWrittenBits() {
    return static_cast<uint32_t>(frac_bits_ >> 15);
  }
  Bits GetFractionalBits() {
    return static_cast<uint32_t>(frac_bits_ & 32767);
  }

private:
  void WriteOut();
  void WriteIfPossible();

  uint32_t low_;
  uint32_t range_;
  uint32_t buffered_byte_;
  int num_buffered_bytes_;
  int bits_left_;
  uint64_t frac_bits_ = 0;
  BitWriter *bit_writer_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_ENTROPY_ENCODER_H_
