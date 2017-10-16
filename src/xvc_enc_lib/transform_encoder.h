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

class TransformEncoder {
public:
  TransformEncoder(int bitdepth, int num_components,
                   const YuvPicture &orig_pic,
                   const EncoderSettings &encoder_settings);

  SampleBuffer& GetPredBuffer(YuvComponent comp) {
    return temp_pred_[static_cast<int>(comp)];
  }
  Distortion CompressAndEvalTransform(CodingUnit *cu, YuvComponent comp,
                                      const Qp &qp, const SyntaxWriter &writer,
                                      const YuvPicture &orig_pic,
                                      CuWriter *cu_writer, YuvPicture *rec_pic);
  bool EvalCbfZero(CodingUnit *cu, const Qp &qp, YuvComponent comp,
                   const SyntaxWriter &bitstream_writer, CuWriter *cu_writer,
                   Distortion dist_non_zero, Distortion dist_zero);
  bool EvalRootCbfZero(CodingUnit *cu, const Qp &qp,
                       const SyntaxWriter &bitstream_writer,
                       CuWriter *cu_writer, Distortion sum_dist_non_zero,
                       Distortion sum_dist_zero);
  Distortion GetResidualDist(const CodingUnit &cu, YuvComponent comp,
                             SampleMetric *metric);

private:
  Distortion TransformAndReconstruct(CodingUnit *cu, YuvComponent comp,
                                     const Qp &qp, const SyntaxWriter &writer,
                                     const YuvPicture &orig_pic,
                                     bool skip_transform, YuvPicture *rec_pic);
  MetricType GetTransformMetric(YuvComponent comp) const {
    return encoder_settings_.structural_ssd > 0 && comp == YuvComponent::kY ?
      MetricType::kStructuralSsd : MetricType::kSsd;
  }

  static const ptrdiff_t kBufferStride_ = constants::kMaxBlockSize;
  const EncoderSettings &encoder_settings_;
  const Sample min_pel_;
  const Sample max_pel_;
  const int num_components_;
  InverseTransform inv_transform_;
  ForwardTransform fwd_transform_;
  Quantize inv_quant_;
  RdoQuant fwd_quant_;
  std::array<SampleBufferStorage, constants::kMaxYuvComponents> temp_pred_;
  ResidualBufferStorage temp_resi_orig_;
  ResidualBufferStorage temp_resi_;
  CoeffBufferStorage temp_coeff_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_TRANSFORM_ENCODER_H_
