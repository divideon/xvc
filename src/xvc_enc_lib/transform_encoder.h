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

#ifndef XVC_ENC_LIB_TRANSFORM_ENCODER_H_
#define XVC_ENC_LIB_TRANSFORM_ENCODER_H_

#include <array>

#include "xvc_common_lib/sample_buffer.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/transform.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_enc_lib/encoder_settings.h"
#include "xvc_enc_lib/encoder_simd_functions.h"
#include "xvc_enc_lib/rdo_quant.h"
#include "xvc_enc_lib/sample_metric.h"
#include "xvc_enc_lib/syntax_writer.h"
#include "xvc_enc_lib/cu_writer.h"

namespace xvc {

enum class TxSearchFlags {
  kNone = 0,
  kNormalTx = 1 << 0,
  kCbfZero = 1 << 1,
  kTransformTskip = 1 << 2,
  kTransformSelect = 1 << 3,
  kFullEval = 0xFFFFFF,
};

class TransformEncoder {
public:
  struct RdCost {
    bool operator==(const RdCost &other) { return cost == other.cost; }
    bool operator<(const RdCost &other) { return cost < other.cost; }
    Cost cost;
    Distortion dist_reco;
    Distortion dist_resi;
  };
  TransformEncoder(const EncoderSimdFunctions &simd, int bitdepth,
                   int num_components, const YuvPicture &orig_pic,
                   const EncoderSettings &encoder_settings);

  SampleBuffer& GetPredBuffer(YuvComponent comp) {
    return temp_pred_[static_cast<int>(comp)];
  }
  RdCost
    CompressAndEvalTransform(CodingUnit *cu, YuvComponent comp,
                             const Qp &qp, const SyntaxWriter &writer,
                             const YuvPicture &orig_pic,
                             TxSearchFlags search_flags, const Cost *orig_cost,
                             Distortion *out_dist_zero, CuWriter *cu_writer,
                             YuvPicture *rec_pic);
  Distortion TransformAndReconstruct(CodingUnit *cu, YuvComponent comp,
                                     const Qp &qp, const SyntaxWriter &writer,
                                     const YuvPicture &orig_pic,
                                     YuvPicture *rec_pic);
  Bits GetCuBitsResidual(const CodingUnit &cu, const SyntaxWriter &writer,
                         CuWriter *cu_writer);
  Bits GetCuBitsFull(const CodingUnit &cu, const SyntaxWriter &writer,
                     CuWriter *cu_writer);

private:
  void ReconstructZeroCbf(CodingUnit *cu, YuvComponent comp,
                          const YuvPicture &orig_pic, YuvPicture *rec_pic);

  static const ptrdiff_t kBufferStride_ = constants::kMaxBlockSize;
  const EncoderSettings &encoder_settings_;
  const Sample min_pel_;
  const Sample max_pel_;
  const int num_components_;
  SampleMetric cu_metric_;
  InverseTransform inv_transform_;
  ForwardTransform fwd_transform_;
  Quantize inv_quant_;
  RdoQuant fwd_quant_;
  CodingUnit::ResidualState best_cu_state_;
  std::array<SampleBufferStorage, constants::kMaxYuvComponents> temp_pred_;
  ResidualBufferStorage temp_resi_orig_;
  ResidualBufferStorage temp_resi_;
  CoeffBufferStorage temp_coeff_;
};

inline TxSearchFlags operator~ (TxSearchFlags a) {
  return static_cast<TxSearchFlags>(~static_cast<int>(a));
}
inline TxSearchFlags operator| (TxSearchFlags a, TxSearchFlags b) {
  return static_cast<TxSearchFlags>(static_cast<int>(a) | static_cast<int>(b));
}
inline TxSearchFlags operator& (TxSearchFlags a, TxSearchFlags b) {
  return static_cast<TxSearchFlags>(static_cast<int>(a) & static_cast<int>(b));
}
inline TxSearchFlags& operator|= (TxSearchFlags& a, TxSearchFlags b) {  // NOLINT
  a = static_cast<TxSearchFlags>(static_cast<int>(a) | static_cast<int>(b));
  return a;
}
inline TxSearchFlags& operator&= (TxSearchFlags& a, TxSearchFlags b) {  // NOLINT
  a = static_cast<TxSearchFlags>(static_cast<int>(a) & static_cast<int>(b));
  return a;
}

}   // namespace xvc

#endif  // XVC_ENC_LIB_TRANSFORM_ENCODER_H_
