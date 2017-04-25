/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/transform_encoder.h"

#include "xvc_common_lib/restrictions.h"

namespace xvc {

TransformEncoder::TransformEncoder(int bitdepth, int num_components,
                                   const YuvPicture &orig_pic)
  : min_pel_(0),
  max_pel_((1 << bitdepth) - 1),
  num_components_(num_components),
  inv_transform_(bitdepth),
  fwd_transform_(bitdepth),
  quantize_(),
  temp_pred_(kBufferStride_, constants::kMaxBlockSize),
  temp_resi_orig_(kBufferStride_, constants::kMaxBlockSize),
  temp_resi_(kBufferStride_, constants::kMaxBlockSize),
  temp_coeff_(kBufferStride_, constants::kMaxBlockSize) {
}

Distortion
TransformEncoder::TransformAndReconstruct(CodingUnit *cu, YuvComponent comp,
                                          const QP & qp,
                                          const YuvPicture &orig_pic,
                                          YuvPicture *rec_pic) {
  int cu_x = cu->GetPosX(comp);
  int cu_y = cu->GetPosY(comp);
  int width = cu->GetWidth(comp);
  int height = cu->GetHeight(comp);
  Coeff *cu_coeff = cu->GetCoeff(comp);
  ptrdiff_t cu_coeff_stride = cu->GetCoeffStride();

  // Calculate residual
  auto orig_buffer = orig_pic.GetSampleBuffer(comp, cu_x, cu_y);
  temp_resi_orig_.Subtract(width, height, orig_buffer, temp_pred_);

  // Transform
  const bool is_luma_intra = util::IsLuma(comp) && cu->IsIntra();
  fwd_transform_.Transform(width, height, is_luma_intra,
                           temp_resi_orig_.GetDataPtr(),
                           temp_resi_orig_.GetStride(),
                           temp_coeff_.GetDataPtr(), temp_coeff_.GetStride());

  // Quant
  int non_zero =
    quantize_.Forward(*cu, comp, qp, width, height, rec_pic->GetBitdepth(),
                      cu->GetPicType(), temp_coeff_.GetDataPtr(),
                      temp_coeff_.GetStride(), cu_coeff, cu_coeff_stride);
  bool cbf = non_zero != 0;
  if (Restrictions::Get().disable_transform_cbf) {
    cbf = true;
  }
  cu->SetCbf(comp, cbf);

  SampleBuffer reco_buffer = rec_pic->GetSampleBuffer(comp, cu_x, cu_y);
  if (cbf) {
    // Dequant
    quantize_.Inverse(comp, qp, width, height, rec_pic->GetBitdepth(),
                      cu_coeff, cu_coeff_stride, temp_coeff_.GetDataPtr(),
                      temp_coeff_.GetStride());

    // Inv transform
    inv_transform_.Transform(width, height, is_luma_intra,
                             temp_coeff_.GetDataPtr(), temp_coeff_.GetStride(),
                             temp_resi_.GetDataPtr(), temp_resi_.GetStride());

    // Reconstruct
    reco_buffer.AddClip(width, height, temp_pred_, temp_resi_,
                        min_pel_, max_pel_);
  } else {
    reco_buffer.CopyFrom(width, height, temp_pred_);
  }

  SampleMetric metric(MetricType::kSSE, qp, rec_pic->GetBitdepth());
  return metric.CompareSample(*cu, comp, orig_pic, reco_buffer);
}

bool TransformEncoder::EvalCbfZero(CodingUnit *cu, const QP &qp,
                                   YuvComponent comp,
                                   const SyntaxWriter &rdo_writer,
                                   Distortion dist_non_zero,
                                   Distortion dist_zero) {
  if (!cu->GetCbf(comp)) {
    return false;
  }
  RdoSyntaxWriter non_zero_writer(rdo_writer, 0);
  non_zero_writer.WriteCbf(*cu, comp, true);
  non_zero_writer.WriteCoefficients(*cu, comp, cu->GetCoeff(comp),
                                    cu->GetCoeffStride());
  Bits non_zero_bits = non_zero_writer.GetNumWrittenBits();

  RdoSyntaxWriter zero_writer(rdo_writer, 0);
  zero_writer.WriteCbf(*cu, comp, false);
  Bits bits_zero = zero_writer.GetNumWrittenBits();

  Cost cost_non_zero = dist_non_zero +
    static_cast<Cost>(non_zero_bits * qp.GetLambda() + 0.5);
  Cost cost_zero = dist_zero +
    static_cast<Cost>(bits_zero * qp.GetLambda() + 0.5);
  if (cost_zero < cost_non_zero) {
    cu->SetCbf(comp, false);
    return true;
  }
  return false;
}

bool TransformEncoder::EvalRootCbfZero(CodingUnit *cu, const QP &qp,
                                       const SyntaxWriter &bitstream_writer,
                                       Distortion sum_dist_non_zero,
                                       Distortion sum_dist_zero) {
  RdoSyntaxWriter rdo_writer_nonzero(bitstream_writer, 0);
  // TODO(Dev) Investigate gains of correct root cbf signaling
  for (int c = 0; c < num_components_; c++) {
    const YuvComponent comp = YuvComponent(c);
    bool cbf = cu->GetCbf(comp);
    rdo_writer_nonzero.WriteCbf(*cu, comp, cbf);
    if (cbf) {
      rdo_writer_nonzero.WriteCoefficients(*cu, comp, cu->GetCoeff(comp),
                                           cu->GetCoeffStride());
    }
  }
  Bits bits_non_zero = rdo_writer_nonzero.GetNumWrittenBits();

  // TODO(Dev) Investigate gains of correct start state
#if HM_STRICT
  RdoSyntaxWriter rdo_writer_zero(rdo_writer_nonzero, 0);
#else
  RdoSyntaxWriter rdo_writer_zero(bitstream_writer, 0);
#endif
  // TODO(Dev) Investigate gains of using correct skip syntax
#if HM_STRICT
  rdo_writer_zero.WriteRootCbf(false);
#else
  if (cu->GetSkipFlag()) {
    rdo_writer_zero.WriteSkipFlag(*cu, true);
  } else {
    rdo_writer_zero.WriteRootCbf(false);
  }
#endif
  Bits bits_zero = rdo_writer_zero.GetNumWrittenBits();

  Cost cost_zero = sum_dist_zero +
    static_cast<Cost>(bits_zero * qp.GetLambda() + 0.5);
  Cost cost_non_zero = sum_dist_non_zero +
    static_cast<Cost>(bits_non_zero * qp.GetLambda() + 0.5);
  return cost_zero < cost_non_zero;
}

Distortion
TransformEncoder::GetResidualDist(const CodingUnit &cu, YuvComponent comp,
                                  SampleMetric *metric) {
  return metric->CompareShort(comp, cu.GetWidth(comp), cu.GetHeight(comp),
                              temp_resi_orig_, temp_resi_);
}

}   // namespace xvc
