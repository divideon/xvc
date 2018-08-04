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

#include "xvc_enc_lib/transform_encoder.h"

#include <limits>

#include "xvc_common_lib/restrictions.h"

namespace xvc {

TransformEncoder::TransformEncoder(const EncoderSimdFunctions &simd,
                                   int bitdepth, int num_components,
                                   const YuvPicture &orig_pic,
                                   const EncoderSettings &encoder_settings)
  : encoder_settings_(encoder_settings),
  min_pel_(0),
  max_pel_((1 << bitdepth) - 1),
  num_components_(num_components),
  cu_metric_(simd.sample_metric, bitdepth, encoder_settings_.structural_ssd ?
             MetricType::kStructuralSsd : MetricType::kSsd,
             encoder_settings_.structural_strength),
  inv_transform_(bitdepth),
  fwd_transform_(bitdepth),
  inv_quant_(),
  fwd_quant_(bitdepth, encoder_settings),
  temp_pred_({ { { constants::kMaxBlockSize, constants::kMaxBlockSize },
  { constants::kMaxBlockSize, constants::kMaxBlockSize },
  { constants::kMaxBlockSize, constants::kMaxBlockSize } } }),
  temp_resi_orig_(kBufferStride_, constants::kMaxBlockSize),
  temp_resi_(kBufferStride_, constants::kMaxBlockSize),
  temp_coeff_(kBufferStride_, constants::kMaxBlockSize) {
}

TransformEncoder::RdCost
TransformEncoder::CompressAndEvalTransform(CodingUnit *cu, YuvComponent comp,
                                           const Qp &qp,
                                           const SyntaxWriter &writer,
                                           const YuvPicture &orig_pic,
                                           TxSearchFlags search_flags,
                                           const Cost *prev_cost,
                                           Distortion *out_dist_zero,
                                           CuWriter *cu_writer,
                                           YuvPicture *rec_pic) {
  const auto get_transform_cost = [&](Distortion dist) {
    if (dist == std::numeric_limits<Distortion>::max()) {
      return RdCost{ std::numeric_limits<Cost>::max(), dist, dist };
    }
    Distortion dist_resi = dist;
    if (encoder_settings_.fast_inter_transform_dist &&
        !encoder_settings_.structural_ssd &&
        cu->IsInter() && cu->GetCbf(comp)) {
      // Set new distortion based on residual instead of reconstructed samples
      // TODO(PH) Consider removing this case (it only adds extra complexity)
      const int width = cu->GetWidth(comp);
      const int height = cu->GetHeight(comp);
      dist_resi = cu_metric_.CompareShort(qp, comp, width, height,
                                          temp_resi_orig_, temp_resi_);
    }
    RdoSyntaxWriter rdo_writer(writer, 0);
    if (cu->IsIntra() && util::IsLuma(comp)) {
      // TODO(PH) Consider remove this case (intra mode signaling is same)
      cu_writer->WriteComponent(*cu, comp, &rdo_writer);
    } else {
      cu_writer->WriteResidualDataRdoCbf(*cu, comp, &rdo_writer);
    }
    Bits bits = rdo_writer.GetNumWrittenBits();
    Cost cost = dist_resi + static_cast<Cost>(bits * qp.GetLambda() + 0.5);
    return RdCost{ cost, dist, dist_resi };
  };

  RdCost best_cost = { std::numeric_limits<Cost>::max(), 0 , 0 };
  if (prev_cost) {
    best_cost.cost = *prev_cost;
  }
  // best state is either applied (to rec_pic) or saved (best_cu_state_)
  bool best_is_applied = prev_cost != nullptr;

  // Evaluate default transform
  if ((search_flags & TxSearchFlags::kNormalTx) != TxSearchFlags::kNone) {
    if (best_is_applied) {
      best_is_applied = false;
      cu->SaveStateTo(&best_cu_state_, *rec_pic, comp);
    }
    cu->SetTransformSkip(comp, false);
    cu->SetTransformFromSelectIdx(comp, -1);
    Distortion dist_normal =
      TransformAndReconstruct(cu, comp, qp, writer, orig_pic, rec_pic);
    RdCost cost = get_transform_cost(dist_normal);
    if (cost < best_cost) {
      best_cost = cost;
      best_is_applied = true;
    }
  }

  // Evaluate cbf zero
  if ((search_flags & TxSearchFlags::kCbfZero) != TxSearchFlags::kNone) {
    Distortion dist_zero =
      cu_metric_.CompareSample(*cu, comp, orig_pic, GetPredBuffer(comp));
    if (out_dist_zero) {
      *out_dist_zero = dist_zero;
    }
    if (cu->GetCbf(comp)) {
      RdoSyntaxWriter zero_writer(writer, 0);
      if (!Restrictions::Get().disable_transform_cbf) {
        zero_writer.WriteCbf(*cu, comp, false);
      } else {
        // Slow method, perform full coeff writing of zero coefficients
        if (best_is_applied) {
          best_is_applied = false;
          cu->SaveStateTo(&best_cu_state_, *rec_pic, comp);
        }
        cu->SetRootCbf(true);
        cu->ClearCbf(comp);
        ReconstructZeroCbf(cu, comp, orig_pic, rec_pic);
        cu_writer->WriteResidualDataRdoCbf(*cu, comp, &zero_writer);
      }
      Bits bits_zero = zero_writer.GetNumWrittenBits();
      Cost cost = dist_zero +
        static_cast<Cost>(bits_zero * qp.GetLambda() + 0.5);
      if (cost < best_cost.cost) {
        cu->ClearCbf(comp);
        ReconstructZeroCbf(cu, comp, orig_pic, rec_pic);
        best_cost = { cost, dist_zero, dist_zero };
        best_is_applied = true;
      }
    }
  }

  // Evaluate transform skip
  if ((search_flags & TxSearchFlags::kTransformTskip) != TxSearchFlags::kNone &&
      cu->CanTransformSkip(comp) &&
      !Restrictions::Get().disable_ext2_transform_skip) {
    if (best_is_applied) {
      best_is_applied = false;
      cu->SaveStateTo(&best_cu_state_, *rec_pic, comp);
    }
    cu->SetTransformSkip(comp, true);
    cu->SetTransformFromSelectIdx(comp, -1);
    Distortion dist_txskip =
      TransformAndReconstruct(cu, comp, qp, writer, orig_pic, rec_pic);
    RdCost cost = get_transform_cost(dist_txskip);
    if (cost < best_cost) {
      best_cost = cost;
      best_is_applied = true;
    }
  }

  // Evaluate transform select
  bool best_has_coeff = best_is_applied ? cu->GetCbf(comp) :
    best_cu_state_.tx.cbf[static_cast<int>(comp)];
  int nbr_tx_select_idx = 0;
  if ((search_flags & TxSearchFlags::kTransformSelect) != TxSearchFlags::kNone
      && util::IsLuma(comp)
      && !Restrictions::Get().disable_ext2_transform_select) {
    nbr_tx_select_idx = constants::kMaxTransformSelectIdx;
  }
  if (encoder_settings_.fast_transform_select_eval &&
    (search_flags & TxSearchFlags::kCbfZero) != TxSearchFlags::kNone &&
      !best_has_coeff) {
    nbr_tx_select_idx = 0;
  }
  for (int tx_select = 0; tx_select < nbr_tx_select_idx; tx_select++) {
    if (best_is_applied) {
      best_is_applied = false;
      cu->SaveStateTo(&best_cu_state_, *rec_pic, comp);
    }
    cu->SetTransformSkip(comp, false);
    cu->SetTransformFromSelectIdx(comp, tx_select);
    Distortion dist =
      TransformAndReconstruct(cu, comp, qp, writer, orig_pic, rec_pic);
    RdCost cost = get_transform_cost(dist);
    if (cost < best_cost) {
      best_cost = cost;
      best_is_applied = true;
    }
  }

  if (!best_is_applied) {
    cu->LoadStateFrom(best_cu_state_, rec_pic, comp);
  }
  return{ best_cost.cost, best_cost.dist_reco, best_cost.dist_resi };
}

Distortion
TransformEncoder::TransformAndReconstruct(CodingUnit *cu, YuvComponent comp,
                                          const Qp & qp,
                                          const SyntaxWriter &syntax_writer,
                                          const YuvPicture &orig_pic,
                                          YuvPicture *rec_pic) {
  const int cu_x = cu->GetPosX(comp);
  const int cu_y = cu->GetPosY(comp);
  const int width = cu->GetWidth(comp);
  const int height = cu->GetHeight(comp);
  const bool skip_transform = cu->GetTransformSkip(comp);
  CoeffBuffer cu_coeff = cu->GetCoeff(comp);

  // Calculate residual
  auto orig_buffer = orig_pic.GetSampleBuffer(comp, cu_x, cu_y);
  SampleBuffer &pred_buffer = GetPredBuffer(comp);
  temp_resi_orig_.Subtract(width, height, orig_buffer, pred_buffer);

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
  if (util::IsLuma(comp) && cu->GetTransformSelectIdx() > 0 &&
      cu->IsIntra() && non_zero < constants::kTransformSelectMinSigCoeffs) {
    // enforce transform select idx signaling invariant for intra
    return std::numeric_limits<Distortion>::max();
  }
  if (util::IsLuma(comp) && cu->HasTransformSelectIdx() &&
      cu->IsInter() && !non_zero) {
    // enforce transform select idx signaling invariant for inter
    return std::numeric_limits<Distortion>::max();
  }
  if (skip_transform && !non_zero) {
    // prevent transform skip without coefficients
    return std::numeric_limits<Distortion>::max();
  }
  bool cbf = non_zero != 0;
  if (!cbf && Restrictions::Get().disable_transform_cbf) {
    cu_coeff.ZeroOut(width, height);
    cbf = true;
  }
  cu->SetCbf(comp, cbf);

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
    reco_buffer.AddClip(width, height, pred_buffer, temp_resi_,
                        min_pel_, max_pel_);
  } else {
    reco_buffer.CopyFrom(width, height, pred_buffer);
  }

  return cu_metric_.CompareSample(*cu, comp, orig_pic, reco_buffer);
}

Bits TransformEncoder::GetCuBitsResidual(const CodingUnit &cu,
                                         const SyntaxWriter &bitstream_writer,
                                         CuWriter *cu_writer) {
  RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
  for (int c = 0; c < num_components_; c++) {
    const YuvComponent comp = YuvComponent(c);
    cu_writer->WriteResidualDataRdoCbf(cu, comp, &rdo_writer);
  }
  return rdo_writer.GetNumWrittenBits();
}

Bits TransformEncoder::GetCuBitsFull(const CodingUnit &cu,
                                     const SyntaxWriter &bitstream_writer,
                                     CuWriter *cu_writer) {
  RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
  for (int c = 0; c < num_components_; c++) {
    const YuvComponent comp = YuvComponent(c);
    cu_writer->WriteComponent(cu, comp, &rdo_writer);
  }
  return rdo_writer.GetNumWrittenBits();
}

void TransformEncoder::ReconstructZeroCbf(CodingUnit *cu, YuvComponent comp,
                                          const YuvPicture &orig_pic,
                                          YuvPicture *rec_pic) {
  const int cu_x = cu->GetPosX(comp);
  const int cu_y = cu->GetPosY(comp);
  const int width = cu->GetWidth(comp);
  const int height = cu->GetHeight(comp);
  const SampleBuffer &pred_buffer = GetPredBuffer(comp);
  SampleBuffer reco_buffer = rec_pic->GetSampleBuffer(comp, cu_x, cu_y);
  reco_buffer.CopyFrom(width, height, pred_buffer);
}

}   // namespace xvc
