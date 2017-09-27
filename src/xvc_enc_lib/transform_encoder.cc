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

#include "xvc_enc_lib/transform_encoder.h"

#include "xvc_common_lib/restrictions.h"

namespace xvc {

TransformEncoder::TransformEncoder(int bitdepth, int num_components,
                                   const YuvPicture &orig_pic,
                                   const EncoderSettings &encoder_settings)
  : encoder_settings_(encoder_settings),
  min_pel_(0),
  max_pel_((1 << bitdepth) - 1),
  num_components_(num_components),
  inv_transform_(bitdepth),
  fwd_transform_(bitdepth),
  inv_quant_(),
  fwd_quant_(bitdepth),
  temp_pred_(kBufferStride_, constants::kMaxBlockSize),
  temp_resi_orig_(kBufferStride_, constants::kMaxBlockSize),
  temp_resi_(kBufferStride_, constants::kMaxBlockSize),
  temp_coeff_(kBufferStride_, constants::kMaxBlockSize) {
}

Distortion
TransformEncoder::CompressAndEvalTransform(CodingUnit *cu, YuvComponent comp,
                                           const Qp &qp,
                                           const SyntaxWriter &writer,
                                           const YuvPicture &orig_pic,
                                           CuWriter *cu_writer,
                                           YuvPicture *rec_pic) {
  const auto get_transform_cost = [&](Distortion dist) {
    if (encoder_settings_.fast_inter_transform_dist &&
        cu->IsInter() && cu->GetCbf(comp)) {
      // Set new distortion based on residual instead of reconstructed samples
      // TODO(PH) Consider removing this case (it only adds extra complexity)
      SampleMetric metric(GetTransformMetric(comp), qp, rec_pic->GetBitdepth());
      dist = GetResidualDist(*cu, comp, &metric);
    }
    RdoSyntaxWriter rdo_writer(writer, 0);
    if (cu->IsIntra() && util::IsLuma(comp)) {
      // TODO(PH) Consider remove this case (intra mode signaling is same)
      cu_writer->WriteComponent(*cu, comp, &rdo_writer);
    } else {
      cu_writer->WriteResidualDataRdoCbf(*cu, comp, &rdo_writer);
    }
    Bits bits = rdo_writer.GetNumWrittenBits();
    return dist + static_cast<Cost>(bits * qp.GetLambda() + 0.5);
  };

  if (Restrictions::Get().disable_transform_skip ||
      !cu->CanTransformSkip(comp)) {
    // Most common case: no transform skip evaluated
    return TransformAndReconstruct(cu, comp, qp, writer, orig_pic, false,
                                   rec_pic);
  }

  // Evaluate transform skip
  Distortion dist_txskip =
    TransformAndReconstruct(cu, comp, qp, writer, orig_pic, true, rec_pic);
  if (!cu->GetCbf(comp)) {
    // Transform skip requires cbf
    return TransformAndReconstruct(cu, comp, qp, writer, orig_pic, false,
                                   rec_pic);
  }
  Cost cost_txskip = get_transform_cost(dist_txskip);

  // Evaluate normal transform
  Distortion dist_normal =
    TransformAndReconstruct(cu, comp, qp, writer, orig_pic, false, rec_pic);
  Cost cost_normal = get_transform_cost(dist_normal);

  if (cost_txskip < cost_normal) {
    // TODO(PH) Evaluate saving coefficients instead of re-calculating tx skip
    return TransformAndReconstruct(cu, comp, qp, writer, orig_pic, true,
                                   rec_pic);
  }
  return dist_normal;
}

Distortion
TransformEncoder::TransformAndReconstruct(CodingUnit *cu, YuvComponent comp,
                                          const Qp & qp,
                                          const SyntaxWriter &syntax_writer,
                                          const YuvPicture &orig_pic,
                                          bool skip_transform,
                                          YuvPicture *rec_pic) {
  int cu_x = cu->GetPosX(comp);
  int cu_y = cu->GetPosY(comp);
  int width = cu->GetWidth(comp);
  int height = cu->GetHeight(comp);
  CoeffBuffer cu_coeff = cu->GetCoeff(comp);

  // Calculate residual
  auto orig_buffer = orig_pic.GetSampleBuffer(comp, cu_x, cu_y);
  temp_resi_orig_.Subtract(width, height, orig_buffer, temp_pred_);

  // Transform
  if (!skip_transform) {
    fwd_transform_.Transform(*cu, comp, temp_resi_orig_, &temp_coeff_);
  } else {
    fwd_transform_.TransformSkip(width, height, temp_resi_orig_, &temp_coeff_);
  }

  // Quant
  int non_zero;
  if (encoder_settings_.rdo_quant) {
    non_zero =
      fwd_quant_.QuantRdo(*cu, comp, qp, cu->GetPicType(), syntax_writer,
                          temp_coeff_.GetDataPtr(), temp_coeff_.GetStride(),
                          cu_coeff.GetDataPtr(), cu_coeff.GetStride());
  } else {
    non_zero =
      fwd_quant_.QuantFast(*cu, comp, qp, cu->GetPicType(),
                           temp_coeff_.GetDataPtr(), temp_coeff_.GetStride(),
                           cu_coeff.GetDataPtr(), cu_coeff.GetStride());
  }
  bool cbf = non_zero != 0;
  if (!cbf && Restrictions::Get().disable_transform_cbf) {
    cu_coeff.ZeroOut(width, height);
    cbf = true;
  }
  cu->SetCbf(comp, cbf);
  cu->SetTransformSkip(comp, skip_transform);

  SampleBuffer reco_buffer = rec_pic->GetSampleBuffer(comp, cu_x, cu_y);
  if (cbf) {
    // Dequant
    inv_quant_.Inverse(comp, qp, width, height, rec_pic->GetBitdepth(),
                       cu_coeff.GetDataPtr(), cu_coeff.GetStride(),
                       temp_coeff_.GetDataPtr(), temp_coeff_.GetStride());

    // Inv transform
    if (!skip_transform) {
      inv_transform_.Transform(*cu, comp, temp_coeff_, &temp_resi_);
    } else {
      inv_transform_.TransformSkip(width, height, temp_coeff_, &temp_resi_);
    }

    // Reconstruct
    reco_buffer.AddClip(width, height, temp_pred_, temp_resi_,
                        min_pel_, max_pel_);
  } else {
    reco_buffer.CopyFrom(width, height, temp_pred_);
  }

  SampleMetric metric(GetTransformMetric(comp), qp, rec_pic->GetBitdepth());
  return metric.CompareSample(*cu, comp, orig_pic, reco_buffer);
}

bool TransformEncoder::EvalCbfZero(CodingUnit *cu, const Qp &qp,
                                   YuvComponent comp,
                                   const SyntaxWriter &rdo_writer,
                                   CuWriter *cu_writer,
                                   Distortion dist_non_zero,
                                   Distortion dist_zero) {
  if (!cu->GetCbf(comp)) {
    return false;
  }
  RdoSyntaxWriter non_zero_writer(rdo_writer, 0);
  cu_writer->WriteResidualDataRdoCbf(*cu, comp, &non_zero_writer);
  Bits non_zero_bits = non_zero_writer.GetNumWrittenBits();

  RdoSyntaxWriter zero_writer(rdo_writer, 0);
  zero_writer.WriteCbf(*cu, comp, false);
  Bits bits_zero = zero_writer.GetNumWrittenBits();

  Cost cost_non_zero = dist_non_zero +
    static_cast<Cost>(non_zero_bits * qp.GetLambda() + 0.5);
  Cost cost_zero = dist_zero +
    static_cast<Cost>(bits_zero * qp.GetLambda() + 0.5);
  if (cost_zero < cost_non_zero ||
    (cost_zero == cost_non_zero && cu->GetTransformSkip(comp))) {
    cu->SetCbf(comp, false);
    return true;
  }
  return false;
}

bool TransformEncoder::EvalRootCbfZero(CodingUnit *cu, const Qp &qp,
                                       const SyntaxWriter &bitstream_writer,
                                       CuWriter *cu_writer,
                                       Distortion sum_dist_non_zero,
                                       Distortion sum_dist_zero) {
  RdoSyntaxWriter rdo_writer_nonzero(bitstream_writer, 0);
  for (int c = 0; c < num_components_; c++) {
    const YuvComponent comp = YuvComponent(c);
    // TODO(Dev) Investigate gains of correct root cbf signaling
    cu_writer->WriteResidualDataRdoCbf(*cu, comp, &rdo_writer_nonzero);
  }
  Bits bits_non_zero = rdo_writer_nonzero.GetNumWrittenBits();

  Bits bits_zero;
  if (encoder_settings_.fast_inter_root_cbf_zero_bits) {
    // Note that root cbf is not used in case of skip, ok both are 1 bin...
    rdo_writer_nonzero.WriteRootCbf(false);
    bits_zero = rdo_writer_nonzero.GetNumWrittenBits() - bits_non_zero;
  } else {
    RdoSyntaxWriter rdo_writer_zero(bitstream_writer, 0);
    // Note that root cbf is not used in case of skip, ok both are 1 bin...
    rdo_writer_zero.WriteRootCbf(false);
    bits_zero = rdo_writer_zero.GetNumWrittenBits();
  }

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
