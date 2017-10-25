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

#include "xvc_enc_lib/intra_search.h"

#include <limits>

#include "xvc_common_lib/restrictions.h"
#include "xvc_enc_lib/sample_metric.h"

namespace xvc {

IntraSearch::IntraSearch(int bitdepth, const PictureData &pic_data,
                         const YuvPicture &orig_pic,
                         const EncoderSettings &encoder_settings)
  : IntraPrediction(bitdepth),
  pic_data_(pic_data),
  orig_pic_(orig_pic),
  encoder_settings_(encoder_settings),
  cu_writer_(pic_data, this) {
}

Distortion
IntraSearch::CompressIntraLuma(CodingUnit *cu, const Qp &qp,
                               const SyntaxWriter &bitstream_writer,
                               TransformEncoder *encoder, YuvPicture *rec_pic) {
  const YuvComponent comp = YuvComponent::kY;
  IntraModeSet intra_modes;
  int num_modes_for_slow_rdo =
    DetermineSlowIntraModes(cu, qp, bitstream_writer, encoder, rec_pic,
                            &intra_modes);

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
    Distortion ssd = CompressIntra(cu, comp, qp, rdo_writer, encoder, rec_pic);
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
  Distortion best_dist = 0;
  if (Restrictions::Get().disable_intra_chroma_predictor) {
    cu->SetIntraModeChroma(IntraChromaMode::kDmChroma);
    best_dist +=
      CompressIntra(cu, YuvComponent::kU, qp, bitstream_writer, enc, rec_pic);
    best_dist +=
      CompressIntra(cu, YuvComponent::kV, qp, bitstream_writer, enc, rec_pic);
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
    dist += CompressIntra(cu, YuvComponent::kU, qp, rdo_writer, enc, rec_pic);
    cu_writer_.WriteResidualData(*cu, YuvComponent::kU, &rdo_writer);
    dist += CompressIntra(cu, YuvComponent::kV, qp, rdo_writer, enc, rec_pic);
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

Distortion IntraSearch::CompressIntra(CodingUnit *cu, YuvComponent comp,
                                      const Qp & qp, const SyntaxWriter &writer,
                                      TransformEncoder *encoder,
                                      YuvPicture *rec_pic) {
  SampleBuffer &pred_buffer = encoder->GetPredBuffer(comp);
  IntraMode intra_mode = cu->GetIntraMode(comp);
  Predict(intra_mode, *cu, comp, *rec_pic, &pred_buffer);
  TxSearchFlags tx_flags = TxSearchFlags::kFullEval & ~TxSearchFlags::kCbfZero;
  TransformEncoder::RdCost tx_cost =
    encoder->CompressAndEvalTransform(cu, comp, qp, writer, orig_pic_, tx_flags,
                                      nullptr, nullptr, &cu_writer_, rec_pic);
  return tx_cost.dist_reco;
}

int
IntraSearch::DetermineSlowIntraModes(CodingUnit *cu, const Qp &qp,
                                       const SyntaxWriter &bitstream_writer,
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
    !Restrictions::Get().disable_ext_intra_extra_modes ?
    kNbrIntraModesExt : kNbrIntraModes;
  const bool two_fast_search_passes =
    !Restrictions::Get().disable_ext_intra_extra_modes;
  SampleBuffer rec_buffer =
    rec_pic->GetSampleBuffer(comp, cu->GetPosX(comp), cu->GetPosY(comp));
  IntraPrediction::State intra_state;
  SampleBuffer &pred_buf = encoder->GetPredBuffer(comp);
  SampleMetric metric(MetricType::kSatd, qp, rec_pic->GetBitdepth());
  std::array<bool, kNbrIntraModesExt> evaluated_modes = { false };

  IntraPredictorLuma mpm = GetPredictorLuma(*cu);
  FillReferenceState(*cu, comp, rec_buffer, &intra_state);
  for (int i = 0; i < num_intra_modes; i++) {
    IntraMode intra_mode = static_cast<IntraMode>(i);
    if (two_fast_search_passes && intra_mode > IntraMode::kDc && (i % 2) != 0) {
      (*modes_cost)[i] = std::make_pair(intra_mode,
                                        std::numeric_limits<double>::max());
      continue;
    }
    Predict(intra_mode, *cu, comp, intra_state, *rec_pic, &pred_buf);

    // Bits
    RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
    rdo_writer.WriteIntraMode(intra_mode, mpm);
    Bits bits = rdo_writer.GetNumWrittenBits();

    uint64_t dist =
      metric.CompareSample(*cu, comp, orig_pic_,
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
      IntraMode base_mode = (*modes_cost)[i].first;
      if (base_mode <= (IntraMode::kDc + 1) ||
          base_mode >= kNbrIntraModesExt - 1) {
        continue;
      }
      for (int offset = -1; offset <= 1; offset += 2) {
        IntraMode intra_mode = static_cast<IntraMode>(base_mode + offset);
        if (evaluated_modes[intra_mode]) {
          continue;
        }
        Predict(intra_mode, *cu, comp, intra_state, *rec_pic, &pred_buf);
        // Bits
        RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
        rdo_writer.WriteIntraMode(intra_mode, mpm);
        Bits bits = rdo_writer.GetNumWrittenBits();
        uint64_t dist =
          metric.CompareSample(*cu, comp, orig_pic_,
                               pred_buf.GetDataPtr(), pred_buf.GetStride());
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
