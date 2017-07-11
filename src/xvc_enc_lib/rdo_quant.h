/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_LIB_RDO_QUANT_H_
#define XVC_ENC_LIB_RDO_QUANT_H_

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_enc_lib/syntax_writer.h"

namespace xvc {

class RdoQuant {
public:
  explicit RdoQuant(int bitdepth) : bitdepth_(bitdepth) {}
  int QuantFast(const CodingUnit &cu, YuvComponent comp, const Qp &qp,
                int width, int height, PicturePredictionType pic_type,
                const Coeff *in, ptrdiff_t in_stride,
                Coeff *out, ptrdiff_t out_stride);

private:
  void AdjustCoeffsForSignHiding(const CodingUnit &cu, YuvComponent comp,
                                 int width, int height,
                                 const Coeff *in, ptrdiff_t in_stride,
                                 const Coeff *delta, ptrdiff_t delta_stride,
                                 Coeff *out, ptrdiff_t out_stride);

  int bitdepth_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_RDO_QUANT_H_
