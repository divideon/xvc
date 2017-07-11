/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_LIB_TRANSFORM_ENCODER_H_
#define XVC_ENC_LIB_TRANSFORM_ENCODER_H_

#include "xvc_common_lib/sample_buffer.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/transform.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_enc_lib/encoder_settings.h"
#include "xvc_enc_lib/rdo_quant.h"
#include "xvc_enc_lib/sample_metric.h"
#include "xvc_enc_lib/syntax_writer.h"

namespace xvc {

class TransformEncoder {
public:
  TransformEncoder(int bitdepth, int num_components,
                   const YuvPicture &orig_pic,
                   const EncoderSettings &encoder_settings);

  SampleBuffer& GetPredBuffer() { return temp_pred_; }
  Distortion TransformAndReconstruct(CodingUnit *cu, YuvComponent comp,
                                     const Qp &qp, const YuvPicture &orig_pic,
                                     YuvPicture *rec_pic);
  bool EvalCbfZero(CodingUnit *cu, const Qp &qp, YuvComponent comp,
                   const SyntaxWriter &bitstream_writer,
                   Distortion dist_non_zero, Distortion dist_zero);
  bool EvalRootCbfZero(CodingUnit *cu, const Qp &qp,
                       const SyntaxWriter &bitstream_writer,
                       Distortion sum_dist_non_zero,
                       Distortion sum_dist_zero);
  Distortion GetResidualDist(const CodingUnit &cu, YuvComponent comp,
                             SampleMetric *metric);

private:
  static const ptrdiff_t kBufferStride_ = constants::kMaxBlockSize;
  const EncoderSettings &encoder_settings_;
  const Sample min_pel_;
  const Sample max_pel_;
  const int num_components_;
  InverseTransform inv_transform_;
  ForwardTransform fwd_transform_;
  Quantize inv_quant_;
  RdoQuant fwd_quant_;
  SampleBufferStorage temp_pred_;
  ResidualBufferStorage temp_resi_orig_;
  ResidualBufferStorage temp_resi_;
  CoeffBufferStorage temp_coeff_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_TRANSFORM_ENCODER_H_
