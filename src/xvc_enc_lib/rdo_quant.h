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
  static const int kStorageSize =
    constants::kMaxBlockSize * constants::kMaxBlockSize;
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
                         Coeff *out, ptrdiff_t out_stride) const;
  void CoeffSignHideRdo(const CodingUnit &cu, YuvComponent comp,
                        const Qp &qp,
                        const Coeff *src, ptrdiff_t src_stride,
                        Coeff *out, ptrdiff_t out_stride) const;
  Coeff QuantCoeffRdo(YuvComponent comp, Coeff orig_coeff, Coeff level,
                      const CoeffCodingState &code_state, Bits sig1_bits,
                      int64_t lambda, int cost_scale, CabacContexts *contexts,
                      const std::function<Coeff(Coeff)> &inv_quant,
                      int64_t *out_cost) const;
  bool EvalZeroSubblock(int subblock_index, int size, bool subblock_csbf,
                        const ContextModel &csbf_ctx, int last_pos_index,
                        int64_t subblock_zero_dist, int64_t lambda,
                        Bits *csbf_bits_to_zero,
                        int64_t *subblock_code_cost) const;
  template<int SubBlockShift>
  int EvalLastPos(const CodingUnit &cu, YuvComponent comp,
                  ScanOrder scan_order, CabacContexts *contexts,
                  int last_pos_index, int64_t lambda,
                  int64_t comp_code_cost, int64_t comp_zero_dist,
                  const Coeff *out, ptrdiff_t out_stride,
                  const uint8_t *subblock_csbf,
                  const Bits *csbf_bits_to_zero) const;
  Bits GetAbsLevelBits(YuvComponent comp, Coeff quant_level,
                       CabacContexts *contexts,
                       const CoeffCodingState &state) const;
  void UpdateCodeState(YuvComponent comp, Coeff quant_level,
                       CoeffCodingState *state) const;
  Bits GetLastPosBits(int width, int height, YuvComponent comp,
                      ScanOrder scan_order, CabacContexts *contexts,
                      int last_pos_x, int last_pos_y) const;
  std::function<Coeff(Coeff)> GetFwdQuantFunc(YuvComponent comp, const Qp &qp,
                                              int width, int height);
  std::function<Coeff(Coeff)> GetInvQuantFunc(YuvComponent comp, const Qp &qp,
                                              int width, int height);
  int64_t BitCost(Bits bits, int64_t lambda) const {
    return (bits * lambda) >> kLambdaPrecision;
  }

  int bitdepth_;
  // Last position eval state
  std::array<int64_t, kStorageSize> coeff_cost_to_zero_;
  std::array<Bits, kStorageSize> coeff_sig_bits_;
  // Sign hide eval state
  std::array<Coeff, kStorageSize> err_dist_;
  std::array<int, kStorageSize> sig_rate_;
  std::array<int, kStorageSize> rate_up_;
  std::array<int, kStorageSize> rate_down_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_RDO_QUANT_H_
