/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/intra_search.h"

#include <utility>
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

IntraMode
IntraSearch::SearchIntraLuma(CodingUnit *cu, YuvComponent comp, const Qp &qp,
                             const SyntaxWriter &bitstream_writer,
                             TransformEncoder *encoder, YuvPicture *rec_pic) {
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
  Sample *reco =
    rec_pic->GetSamplePtr(comp, cu->GetPosX(comp), cu->GetPosY(comp));
  const ptrdiff_t reco_stride = rec_pic->GetStride(comp);
  IntraPredictorLuma mpm = GetPredictorLuma(*cu);
  IntraPrediction::State intra_state =
    ComputeReferenceState(*cu, comp, reco, reco_stride);

  SampleBuffer &pred_buf = encoder->GetPredBuffer();
  SampleMetric metric(MetricType::kSatd, qp, rec_pic->GetBitdepth());
  std::array<std::pair<IntraMode, double>, IntraMode::kTotalNumber> modes_cost;
  for (int i = 0; i < IntraMode::kTotalNumber; i++) {
    IntraMode intra_mode = static_cast<IntraMode>(i);
    Predict(intra_mode, *cu, comp, intra_state,
            pred_buf.GetDataPtr(), pred_buf.GetStride());

    // Bits
    RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
    rdo_writer.WriteIntraMode(intra_mode, mpm);
    size_t bits = rdo_writer.GetNumWrittenBits();

    uint64_t sad = metric.CompareSample(*cu, comp, orig_pic_,
                                        pred_buf.GetDataPtr(),
                                        pred_buf.GetStride());
    double cost = sad + bits * qp.GetLambdaSqrt();
    modes_cost[i] = std::make_pair(intra_mode, cost);
  }
  std::stable_sort(modes_cost.begin(), modes_cost.end(),
                   [](std::pair<IntraMode, double> p1,
                      std::pair<IntraMode, double> p2) {
    return p1.second < p2.second;
  });

  // Extend shortlist with mpm modes if not already included
  int width_log2 = util::SizeToLog2(cu->GetWidth(comp));
  int height_log2 = util::SizeToLog2(cu->GetHeight(comp));
  int num_modes_for_slow_rdo = kNumIntraFastModesNoExt[width_log2];
  if (encoder_settings_.fast_intra_mode_eval_level == 2) {
    num_modes_for_slow_rdo = kNumIntraFastModesExt[width_log2][height_log2];
  } else if (encoder_settings_.fast_intra_mode_eval_level == 0) {
    num_modes_for_slow_rdo = 33;
  }
  for (int i = 0; i < mpm.num_neighbor_modes; i++) {
    bool found = false;
    for (int j = 0; j < num_modes_for_slow_rdo; j++) {
      if (modes_cost[j].first == mpm[i]) {
        found = true;
        break;
      }
    }
    if (!found) {
      modes_cost[num_modes_for_slow_rdo++].first = mpm[i];
    }
  }

  IntraMode best_mode = static_cast<IntraMode>(0);
  Cost best_cost = std::numeric_limits<Cost>::max();
  for (int i = 0; i < num_modes_for_slow_rdo; i++) {
    IntraMode intra_mode = modes_cost[i].first;
    cu->SetIntraModeLuma(intra_mode);

    // Full reconstruction
    Distortion ssd =
      CompressIntra(cu, comp, qp, bitstream_writer, encoder, rec_pic);

    // Bits
    RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
    cu_writer_.WriteComponent(*cu, comp, &rdo_writer);
    size_t bits = rdo_writer.GetNumWrittenBits();

    Cost cost = ssd + static_cast<Cost>(bits * qp.GetLambda() + 0.5);
    if (cost < best_cost) {
      best_cost = cost;
      best_mode = intra_mode;
    }
  }
  return best_mode;
}

IntraChromaMode
IntraSearch::SearchIntraChroma(CodingUnit *cu, const Qp &qp,
                               const SyntaxWriter &bitstream_writer,
                               TransformEncoder *enc, YuvPicture *rec_pic) {
  const CodingUnit *luma_cu = pic_data_.GetLumaCu(cu);
  IntraMode luma_mode = luma_cu->GetIntraMode(YuvComponent::kY);
  IntraPredictorChroma chroma_modes = GetPredictorsChroma(luma_mode);
  IntraChromaMode best_mode = IntraChromaMode::kDmChroma;
  Cost best_cost = std::numeric_limits<Cost>::max();
  if (Restrictions::Get().disable_intra_chroma_predictor) {
    return best_mode;
  }
  for (int i = 0; i < static_cast<int>(chroma_modes.size()); i++) {
    IntraChromaMode chroma_mode = chroma_modes[i];
    cu->SetIntraModeChroma(chroma_mode);

    // Full reconstruction
    Distortion ssd = 0;
    ssd +=
      CompressIntra(cu, YuvComponent::kU, qp, bitstream_writer, enc, rec_pic);
    ssd +=
      CompressIntra(cu, YuvComponent::kV, qp, bitstream_writer, enc, rec_pic);

    // Bits
    RdoSyntaxWriter rdo_writer(bitstream_writer, 0);
    cu_writer_.WriteComponent(*cu, YuvComponent::kU, &rdo_writer);
    cu_writer_.WriteComponent(*cu, YuvComponent::kV, &rdo_writer);
    size_t bits = rdo_writer.GetNumWrittenBits();

    Cost cost = ssd + static_cast<Cost>(bits * qp.GetLambda() + 0.5);
    if (cost < best_cost) {
      best_cost = cost;
      best_mode = chroma_modes[i];
    }
  }
  return best_mode;
}

Distortion IntraSearch::CompressIntra(CodingUnit *cu, YuvComponent comp,
                                      const Qp &qp, const SyntaxWriter &writer,
                                      TransformEncoder *encoder,
                                      YuvPicture *rec_pic) {
  int cu_x = cu->GetPosX(comp);
  int cu_y = cu->GetPosY(comp);
  SampleBuffer reco_buffer = rec_pic->GetSampleBuffer(comp, cu_x, cu_y);
  SampleBuffer &pred_buf = encoder->GetPredBuffer();
  IntraMode intra_mode = cu->GetIntraMode(comp);
  Predict(intra_mode, *cu, comp, reco_buffer.GetDataPtr(),
          reco_buffer.GetStride(), pred_buf.GetDataPtr(), pred_buf.GetStride());
  return encoder->TransformAndReconstruct(cu, comp, qp, writer, orig_pic_,
                                          rec_pic);
}

}   // namespace xvc
