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
