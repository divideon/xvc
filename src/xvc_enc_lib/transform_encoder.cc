/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/transform_encoder.h"

#include "xvc_common_lib/restrictions.h"
#include "xvc_enc_lib/sample_metric.h"

namespace xvc {

TransformEncoder::TransformEncoder(int bitdepth, const YuvPicture &orig_pic)
  : temp_pred_(kBufferStride_, constants::kMaxBlockSize),
  temp_resi_orig_(kBufferStride_, constants::kMaxBlockSize),
  temp_resi_(kBufferStride_, constants::kMaxBlockSize),
  min_pel_(0),
  max_pel_((1 << bitdepth) - 1),
  inv_transform_(bitdepth),
  fwd_transform_(bitdepth),
  quantize_(),
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
  fwd_transform_.Transform(width, height, temp_resi_orig_.GetDataPtr(),
                           temp_resi_orig_.GetStride(),
                           temp_coeff_.GetDataPtr(),
                           temp_coeff_.GetStride());

  // Quant
  int transform_shift = constants::kMaxTrDynamicRange
    - rec_pic->GetBitdepth() - util::SizeToLog2(width);
  int shift_quant = constants::kQuantShift + qp.GetQpPer(comp)
    + transform_shift;
  bool is_intra_pic = cu->GetPicType() == PicturePredictionType::kIntra;
  const int offset_quant = QP::GetOffsetQuant(is_intra_pic, shift_quant);
  int non_zero = quantize_.Forward(comp, qp, shift_quant, offset_quant, width,
                                   height, temp_coeff_.GetDataPtr(),
                                   temp_coeff_.GetStride(), cu_coeff,
                                   cu_coeff_stride);
  bool cbf = non_zero != 0;
  if (Restrictions::Get().disable_transform_cbf) {
    cbf = true;
  }
  cu->SetCbf(comp, cbf);

  SampleBuffer reco_buffer = rec_pic->GetSampleBuffer(comp, cu_x, cu_y);
  if (cbf) {
    // Dequant
    const int shift_iquant = constants::kIQuantShift
      - constants::kQuantShift - transform_shift;
    quantize_.Inverse(comp, qp, shift_iquant, width, height, cu_coeff,
                      cu_coeff_stride, temp_coeff_.GetDataPtr(),
                      temp_coeff_.GetStride());

    // Inv transform
    inv_transform_.Transform(width, height, temp_coeff_.GetDataPtr(),
                             temp_coeff_.GetStride(), temp_resi_.GetDataPtr(),
                             temp_resi_.GetStride());

    // Reconstruct
    reco_buffer.AddClip(width, height, temp_pred_, temp_resi_,
                        min_pel_, max_pel_);
  } else {
    reco_buffer.CopyFrom(width, height, temp_pred_);
  }

  SampleMetric metric(MetricType::kSSE, qp, rec_pic->GetBitdepth());
  return metric.CompareSample(*cu, comp, orig_pic, reco_buffer);
}

}   // namespace xvc
