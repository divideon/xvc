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

#ifndef XVC_ENC_LIB_TRANSFORM_ENCODER_H_
#define XVC_ENC_LIB_TRANSFORM_ENCODER_H_

#include <array>

#include "xvc_common_lib/sample_buffer.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/transform.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_enc_lib/encoder_settings.h"
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
  TransformEncoder(int bitdepth, int num_components,
                   const YuvPicture &orig_pic,
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
