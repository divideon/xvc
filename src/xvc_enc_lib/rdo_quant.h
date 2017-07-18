/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_LIB_RDO_QUANT_H_
#define XVC_ENC_LIB_RDO_QUANT_H_

#include <functional>

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_enc_lib/syntax_writer.h"

namespace xvc {

class RdoQuant {
public:
  explicit RdoQuant(int bitdepth) : bitdepth_(bitdepth) {}
  int QuantFast(const CodingUnit &cu, YuvComponent comp, const Qp &qp,
                PicturePredictionType pic_type,
                const Coeff *in, ptrdiff_t in_stride,
                Coeff *out, ptrdiff_t out_stride);
  int QuantRdo(const CodingUnit &cu, YuvComponent comp, const Qp &qp,
               PicturePredictionType pic_type, const SyntaxWriter &writer,
               const Coeff *in, ptrdiff_t in_stride,
               Coeff *out, ptrdiff_t out_stride);

private:
  static const int kLambdaPrecision = 16;
  struct CoeffCodingState;
  template<int SubBlockShift>
  int QuantRdo(const CodingUnit &cu, YuvComponent comp, const Qp &qp,
               PicturePredictionType pic_type, const SyntaxWriter &writer,
               const Coeff *in, ptrdiff_t in_stride,
               Coeff *out, ptrdiff_t out_stride);
  void CoeffSignHideFast(const CodingUnit &cu, YuvComponent comp,
                         int width, int height,
                         const Coeff *in, ptrdiff_t in_stride,
                         const Coeff *delta, ptrdiff_t delta_stride,
                         Coeff *out, ptrdiff_t out_stride);
  void CoeffSignHideRdo(const CodingUnit &cu, YuvComponent comp,
                        const Qp &qp,
                        const Coeff *src, ptrdiff_t src_stride,
                        const Coeff *delta, const int *sig_rate,
                        const int *rate_up, const int *rate_down,
                        Coeff *out, ptrdiff_t out_stride);
  Coeff QuantCoeffRdo(YuvComponent comp, Coeff orig_coeff, Coeff level,
                      const CoeffCodingState &code_state, Bits sig1_bits,
                      int64_t lambda, int cost_scale, CabacContexts *contexts,
                      const std::function<Coeff(Coeff)> &inv_quant,
                      int64_t *out_cost);
  bool EvalZeroSubblock(int subblock_index, int size, bool subblock_csbf,
                        const ContextModel &csbf_ctx, int last_pos_index,
                        int64_t subblock_zero_dist, int64_t lambda,
                        Bits *csbf_bits_to_zero, int64_t *subblock_code_cost);
  template<int SubBlockShift>
  int EvalLastPos(const CodingUnit &cu, YuvComponent comp,
                  ScanOrder scan_order, CabacContexts *contexts,
                  int last_pos_index, int64_t lambda,
                  int64_t comp_code_cost, int64_t comp_zero_dist,
                  const Coeff *out, ptrdiff_t out_stride,
                  const uint8_t *subblock_csbf, const Bits *csbf_bits_to_zero,
                  const int64_t *coeff_cost_zero, const Bits *coeff_sig_bits);
  Bits GetAbsLevelBits(YuvComponent comp, Coeff quant_level,
                       CabacContexts *contexts, const CoeffCodingState &state);
  void UpdateCodeState(YuvComponent comp, Coeff quant_level,
                       CoeffCodingState *state);
  Bits GetLastPosBits(int width, int height, YuvComponent comp,
                      ScanOrder scan_order, CabacContexts *contexts,
                      int last_pos_x, int last_pos_y);
  std::function<Coeff(Coeff)> GetFwdQuantFunc(YuvComponent comp, const Qp &qp,
                                              int width, int height);
  std::function<Coeff(Coeff)> GetInvQuantFunc(YuvComponent comp, const Qp &qp,
                                              int width, int height);
  int64_t BitCost(Bits bits, int64_t lambda) {
    return (bits * lambda) >> kLambdaPrecision;
  }

  int bitdepth_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_RDO_QUANT_H_
