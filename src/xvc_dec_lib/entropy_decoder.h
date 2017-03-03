/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_DEC_LIB_ENTROPY_DECODER_H_
#define XVC_DEC_LIB_ENTROPY_DECODER_H_

#include "xvc_common_lib/context_model.h"
#include "xvc_dec_lib/bit_reader.h"

namespace xvc {

class EntropyDecoder {
public:
  explicit EntropyDecoder(BitReader *bit_reader);

  uint32_t DecodeBin(ContextModel *ctx);
  uint32_t DecodeBypass();
  uint32_t DecodeBypassBins(int num_bins);
  uint32_t DecodeBinTrm();

  void Start();
  void Finish();

private:
  uint32_t range_;
  uint32_t value_;
  int bits_needed_;
  BitReader *bit_reader_;
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_ENTROPY_DECODER_H_
