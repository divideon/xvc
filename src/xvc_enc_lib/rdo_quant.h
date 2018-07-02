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

#ifndef XVC_ENC_LIB_RDO_QUANT_H_
#define XVC_ENC_LIB_RDO_QUANT_H_

#include <functional>

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_enc_lib/encoder_settings.h"
#include "xvc_enc_lib/syntax_writer.h"

namespace xvc {

class RdoQuant {
public:
  RdoQuant(int bitdepth, const EncoderSettings &encoder_settings)
    : bitdepth_(bitdepth),
    encoder_settings_(encoder_settings) {
  }
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
  int CoeffSignHideFast(const CodingUnit &cu, YuvComponent comp,
                        int width, int height,
                        const Coeff *in, ptrdiff_t in_stride,
                        const Coeff *delta, ptrdiff_t delta_stride,
                        Coeff *out, ptrdiff_t out_stride) const;
  int CoeffSignHideRdo(const CodingUnit &cu, YuvComponent comp,
                       const Qp &qp,
                       const Coeff *src, ptrdiff_t src_stride,
                       Coeff *out, ptrdiff_t out_stride) const;
  Coeff QuantCoeffRdo(YuvComponent comp, Coeff orig_coeff, Coeff level,
                      const CoeffCodingState &code_state, Bits sig1_bits,
                      int64_t lambda, int cost_scale,
                      const ContextModel &c1_ctx, const ContextModel &c2_ctx,
                      const std::function<Coeff(Coeff)> &inv_quant,
                      int64_t *out_cost) const;
  bool EvalZeroSubblock(int subblock_index, int size, bool subblock_csbf,
                        const ContextModel &csbf_ctx, int last_pos_index,
                        int64_t subblock_zero_dist, int64_t lambda,
                        Bits *csbf_bits_to_zero,
                        int64_t *subblock_code_cost) const;
  template<int SubBlockShift>
  int EvalLastPos(const CodingUnit &cu, YuvComponent comp,
                  ScanOrder scan_order, Contexts *contexts,
                  int last_pos_index, int64_t lambda,
                  int64_t comp_code_cost, int64_t comp_zero_dist,
                  const Coeff *out, ptrdiff_t out_stride,
                  const uint8_t *subblock_csbf,
                  const Bits *csbf_bits_to_zero) const;
  Bits GetAbsLevelBits(YuvComponent comp, Coeff quant_level,
                       const ContextModel &c1_ctx,
                       const ContextModel &c2_ctx,
                       const CoeffCodingState &state) const;
  void UpdateCodeState(YuvComponent comp, Coeff quant_level,
                       CoeffCodingState *state) const;
  Bits GetLastPosBits(int width, int height, YuvComponent comp,
                      ScanOrder scan_order, Contexts *contexts,
                      int last_pos_x, int last_pos_y) const;
  std::function<Coeff(Coeff)> GetFwdQuantFunc(YuvComponent comp, const Qp &qp,
                                              int width, int height);
  std::function<Coeff(Coeff)> GetInvQuantFunc(YuvComponent comp, const Qp &qp,
                                              int width, int height);
  int64_t BitCost(Bits bits, int64_t lambda) const {
    return (bits * lambda) >> kLambdaPrecision;
  }

  const int bitdepth_;
  const EncoderSettings &encoder_settings_;
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
