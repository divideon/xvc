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

#include "xvc_enc_lib/intra_search.h"

#include <limits>

#include "xvc_common_lib/restrictions.h"
#include "xvc_enc_lib/sample_metric.h"

namespace xvc {

IntraSearch::IntraSearch(const EncoderSimdFunctions &simd, int bitdepth,
                         const PictureData &pic_data,
                         const YuvPicture &orig_pic,
                         const EncoderSettings &encoder_settings)
  : IntraPrediction(bitdepth),
  pic_data_(pic_data),
  orig_pic_(orig_pic),
  encoder_settings_(encoder_settings),
  satd_metric_(simd.sample_metric, bitdepth, MetricType::kSatd),
  cu_writer_(pic_data, this) {
}

Distortion
IntraSearch::CompressIntraLuma(CodingUnit *cu, const Qp &qp,
                               const SyntaxWriter &bitstream_writer,
                               TransformEncoder *encoder, YuvPicture *rec_pic) {
  const YuvComponent comp = YuvComponent::kY;
  IntraPrediction::RefState ref_state;
  IntraModeSet intra_modes;
  FillReferenceState(*cu, comp, *rec_pic, &ref_state);
  int num_modes_for_slow_rdo =
    DetermineSlowIntraModes(cu, qp, bitstream_writer, ref_state, encoder,
                            rec_pic, &intra_modes);

  IntraMode best_mode = IntraMode::kInvalid;
  Cost best_cost = std::numeric_limits<Cost>::max();
  Distortion best_dist = std::numeric_limits<Distortion>::max();
  bool best_is_applied = false;
  bool best_uses_tx_select = false;
  for (int i = 0; i < num_modes_for_slow_rdo; i++) {
    IntraMode intra_mode = intra_modes[i].first;
    cu->SetIntraModeLuma(intra_mode);
    best_is_applied = false;

    // Full reconstruction
    RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
    Distortion ssd = PredictAndTransform(cu, comp, qp, rdo_writer, ref_state,
                                         encoder, rec_pic);
    cu_writer_.WriteComponent(*cu, comp, &rdo_writer);

    Bits bits = rdo_writer.GetNumWrittenBits();
    Cost cost = ssd + static_cast<Cost>(bits * qp.GetLambda() + 0.5);
    const bool bias_normal_tx_type = cost == best_cost &&
      best_uses_tx_select && !cu->HasTransformSelectIdx();
    if (cost < best_cost || bias_normal_tx_type) {
      best_cost = cost;
      best_dist = ssd;
      best_mode = intra_mode;
      best_uses_tx_select = cu->HasTransformSelectIdx();
      best_is_applied = true;
      cu->SaveStateTo(&best_cu_state_, *rec_pic, YuvComponent::kY);
    }
  }

  cu->SetIntraModeLuma(best_mode);
  if (!best_is_applied) {
    cu->LoadStateFrom(best_cu_state_, rec_pic, YuvComponent::kY);
  }
  return best_dist;
}

Distortion
IntraSearch::CompressIntraChroma(CodingUnit *cu, const Qp &qp,
                                 const SyntaxWriter &bitstream_writer,
                                 TransformEncoder *enc, YuvPicture *rec_pic) {
  const CodingUnit *luma_cu = pic_data_.GetLumaCu(cu);
  IntraMode luma_mode = luma_cu->GetIntraMode(YuvComponent::kY);
  IntraPredictorChroma chroma_modes = GetPredictorsChroma(luma_mode);
  IntraPrediction::RefState ref_state_u;
  IntraPrediction::RefState ref_state_v;
  Distortion best_dist = 0;

  FillReferenceState(*cu, YuvComponent::kU, *rec_pic, &ref_state_u);
  FillReferenceState(*cu, YuvComponent::kV, *rec_pic, &ref_state_v);
  if (Restrictions::Get().disable_intra_chroma_predictor) {
    cu->SetIntraModeChroma(IntraChromaMode::kDmChroma);
    best_dist += PredictAndTransform(cu, YuvComponent::kU, qp, bitstream_writer,
                                     ref_state_u, enc, rec_pic);
    best_dist += PredictAndTransform(cu, YuvComponent::kV, qp, bitstream_writer,
                                     ref_state_v, enc, rec_pic);
    return best_dist;
  }

  Cost best_cost = std::numeric_limits<Cost>::max();
  IntraChromaMode best_mode = IntraChromaMode::kInvalid;
  bool best_is_applied = false;

  for (int i = 0; i < static_cast<int>(chroma_modes.size()); i++) {
    IntraChromaMode chroma_mode = chroma_modes[i];
    if (chroma_mode == IntraChromaMode::kInvalid) {
      continue;
    }
    cu->SetIntraModeChroma(chroma_mode);
    best_is_applied = false;

    // Full reconstruction
    RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
    Distortion dist = 0;
    dist += PredictAndTransform(cu, YuvComponent::kU, qp, rdo_writer,
                                ref_state_u, enc, rec_pic);
    cu_writer_.WriteResidualData(*cu, YuvComponent::kU, &rdo_writer);
    dist += PredictAndTransform(cu, YuvComponent::kV, qp, rdo_writer,
                                ref_state_v, enc, rec_pic);
    cu_writer_.WriteResidualData(*cu, YuvComponent::kV, &rdo_writer);

    // TODO(PH) Should optimally be done before rdo quant
    cu_writer_.WriteIntraPrediction(*cu, YuvComponent::kU, &rdo_writer);
    cu_writer_.WriteIntraPrediction(*cu, YuvComponent::kV, &rdo_writer);
    Bits bits = rdo_writer.GetNumWrittenBits();

    Cost cost = dist + static_cast<Cost>(bits * qp.GetLambda() + 0.5);
    if (cost < best_cost) {
      best_cost = cost;
      best_dist = dist;
      best_mode = chroma_modes[i];
      best_is_applied = true;
      cu->SaveStateTo(&best_cu_state_, *rec_pic, YuvComponent::kU);
      cu->SaveStateTo(&best_cu_state_, *rec_pic, YuvComponent::kV);
    }
  }
  assert(best_mode != IntraChromaMode::kInvalid);
  cu->SetIntraModeChroma(best_mode);
  if (!best_is_applied) {
    cu->LoadStateFrom(best_cu_state_, rec_pic, YuvComponent::kU);
    cu->LoadStateFrom(best_cu_state_, rec_pic, YuvComponent::kV);
  }
  return best_dist;
}

Distortion
IntraSearch::CompressIntraFast(CodingUnit *cu, YuvComponent comp,
                               const Qp & qp, const SyntaxWriter &writer,
                               TransformEncoder *encoder,
                               YuvPicture *rec_pic) {
  IntraPrediction::RefState ref_state;
  FillReferenceState(*cu, comp, *rec_pic, &ref_state);
  // TODO(PH) Make faster method without transform evaluation
  return PredictAndTransform(cu, comp, qp, writer, ref_state, encoder,
                             rec_pic);
}

Distortion
IntraSearch::PredictAndTransform(CodingUnit *cu, YuvComponent comp,
                                 const Qp & qp, const SyntaxWriter &writer,
                                 const IntraPrediction::RefState &ref_state,
                                 TransformEncoder *encoder,
                                 YuvPicture *rec_pic) {
  SampleBuffer &pred_buffer = encoder->GetPredBuffer(comp);
  IntraMode intra_mode = cu->GetIntraMode(comp);
  Predict(intra_mode, *cu, comp, ref_state, *rec_pic, &pred_buffer);
  TxSearchFlags tx_flags = TxSearchFlags::kFullEval & ~TxSearchFlags::kCbfZero;
  TransformEncoder::RdCost tx_cost =
    encoder->CompressAndEvalTransform(cu, comp, qp, writer, orig_pic_, tx_flags,
                                      nullptr, nullptr, &cu_writer_, rec_pic);
  return tx_cost.dist_reco;
}

int
IntraSearch::DetermineSlowIntraModes(CodingUnit *cu, const Qp &qp,
                                     const SyntaxWriter &bitstream_writer,
                                     const IntraPrediction::RefState &ref_state,
                                     TransformEncoder *encoder,
                                     YuvPicture *rec_pic,
                                     IntraModeSet *modes_cost) {
  static const std::array<std::array<uint8_t, 8>, 8> kNumIntraFastModesExt = { {
      // 1, 2, 4, 8, 16, 32, 64, 128
    { 0, 0, 0, 0, 0, 0, 0, 0 },   // 1
    { 0, 0, 0, 0, 0, 0, 0, 0 },   // 2
    { 0, 0, 3, 3, 3, 3, 2, 2 },   // 4
    { 0, 0, 3, 3, 3, 3, 3, 2 },   // 8
    { 0, 0, 3, 3, 3, 3, 3, 2 },   // 16
    { 0, 0, 3, 3, 3, 3, 3, 2 },   // 32
    { 0, 0, 2, 3, 3, 3, 3, 2 },   // 64
    { 0, 0, 2, 2, 2, 2, 2, 3 },   // 128
    } };
  static const std::array<uint8_t, 7> kNumIntraFastModesNoExt = {
    /*1x1: */ 0, /*2x2: */ 3, /*4x4: */ 8, /*8x8: */ 8, /*16x16: */ 3,
    /*32x32: */ 3, /*64x64: */ 3
  };
  const YuvComponent comp = YuvComponent::kY;
  const int num_intra_modes =
    !Restrictions::Get().disable_ext2_intra_67_modes ?
    kNbrIntraModesExt : kNbrIntraModes;
  const bool two_fast_search_passes =
    !Restrictions::Get().disable_ext2_intra_67_modes;
  SampleBuffer &pred_buf = encoder->GetPredBuffer(comp);
  std::array<bool, kNbrIntraModesExt> evaluated_modes = { false };

  IntraPredictorLuma mpm = GetPredictorLuma(*cu);
  for (int i = 0; i < num_intra_modes; i++) {
    IntraMode intra_mode = static_cast<IntraMode>(i);
    if (two_fast_search_passes && intra_mode > IntraMode::kDc && (i % 2) != 0) {
      (*modes_cost)[i] = std::make_pair(intra_mode,
                                        std::numeric_limits<double>::max());
      continue;
    }
    Predict(intra_mode, *cu, comp, ref_state, *rec_pic, &pred_buf);

    // Bits
    RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
    rdo_writer.WriteIntraMode(intra_mode, mpm);
    Bits bits = rdo_writer.GetNumWrittenBits();

    uint64_t dist =
      satd_metric_.CompareSample(*cu, comp, orig_pic_,
                                 pred_buf.GetDataPtr(), pred_buf.GetStride());
    double cost = dist + bits * qp.GetLambdaSqrt();
    (*modes_cost)[i] = std::make_pair(intra_mode, cost);
    evaluated_modes[intra_mode] = true;
  }
  std::stable_sort(modes_cost->begin(), modes_cost->begin() + num_intra_modes,
                   [](std::pair<IntraMode, double> p1,
                      std::pair<IntraMode, double> p2) {
    return p1.second < p2.second;
  });

  // Number of modes to evaluate in slow pass
  int width_log2 = util::SizeToLog2(cu->GetWidth(comp));
  int height_log2 = util::SizeToLog2(cu->GetHeight(comp));
  int num_modes_for_slow_rdo = kNumIntraFastModesNoExt[width_log2];
  if (encoder_settings_.fast_intra_mode_eval_level == 2) {
    num_modes_for_slow_rdo = kNumIntraFastModesExt[width_log2][height_log2];
  } else if (encoder_settings_.fast_intra_mode_eval_level == 0) {
    num_modes_for_slow_rdo = 33;
  }

  if (two_fast_search_passes) {
    int modes_added = num_modes_for_slow_rdo;
    for (int i = 0; i < num_modes_for_slow_rdo; i++) {
      int base_mode = static_cast<int>((*modes_cost)[i].first);
      if (base_mode <= (IntraMode::kDc + 1) ||
          base_mode >= kNbrIntraModesExt - 1) {
        continue;
      }
      for (int offset = -1; offset <= 1; offset += 2) {
        IntraMode intra_mode = static_cast<IntraMode>(base_mode + offset);
        if (evaluated_modes[intra_mode]) {
          continue;
        }
        Predict(intra_mode, *cu, comp, ref_state, *rec_pic, &pred_buf);
        // Bits
        RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
        rdo_writer.WriteIntraMode(intra_mode, mpm);
        Bits bits = rdo_writer.GetNumWrittenBits();
        uint64_t dist =
          satd_metric_.CompareSample(*cu, comp, orig_pic_, pred_buf);
        double cost = dist + bits * qp.GetLambdaSqrt();
        (*modes_cost)[modes_added++] = std::make_pair(intra_mode, cost);
        evaluated_modes[intra_mode] = true;
      }
    }
    std::stable_sort(modes_cost->begin(), modes_cost->begin() + modes_added,
                     [](std::pair<IntraMode, double> p1,
                        std::pair<IntraMode, double> p2) {
      return p1.second < p2.second;
    });
  }

  // Extend shortlist with predictor modes
  for (int i = 0; i < mpm.num_neighbor_modes; i++) {
    bool found = false;
    for (int j = 0; j < num_modes_for_slow_rdo; j++) {
      if ((*modes_cost)[j].first == mpm[i]) {
        found = true;
        break;
      }
    }
    if (!found) {
      (*modes_cost)[num_modes_for_slow_rdo++].first = mpm[i];
    }
  }
  return num_modes_for_slow_rdo;
}

}   // namespace xvc
