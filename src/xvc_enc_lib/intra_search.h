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

#ifndef XVC_ENC_LIB_INTRA_SEARCH_H_
#define XVC_ENC_LIB_INTRA_SEARCH_H_

#include <utility>

#include "xvc_common_lib/intra_prediction.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_enc_lib/syntax_writer.h"
#include "xvc_enc_lib/cu_writer.h"
#include "xvc_enc_lib/encoder_settings.h"
#include "xvc_enc_lib/transform_encoder.h"

namespace xvc {

class IntraSearch : public IntraPrediction {
public:
  IntraSearch(int bitdepth, const PictureData &pic_data,
              const YuvPicture &orig_pic,
              const EncoderSettings &encoder_settings);

  Distortion CompressIntraLuma(CodingUnit *cu, const Qp &qp,
                               const SyntaxWriter &bitstream_writer,
                               TransformEncoder *encoder, YuvPicture *rec_pic);
  Distortion CompressIntraChroma(CodingUnit *cu, const Qp &qp,
                                 const SyntaxWriter &bitstream_writer,
                                 TransformEncoder *encoder,
                                 YuvPicture *rec_pic);
  Distortion CompressIntraFast(CodingUnit *cu, YuvComponent comp, const Qp &qp,
                               const SyntaxWriter &writer,
                               TransformEncoder *encoder, YuvPicture *rec_pic);

private:
  using IntraModeSet =
    std::array<std::pair<IntraMode, double>, kNbrIntraModesExt>;
  Distortion PredictAndTransform(CodingUnit *cu, YuvComponent comp,
                                 const Qp &qp, const SyntaxWriter &writer,
                                 const IntraPrediction::RefState &ref_state,
                                 TransformEncoder *encoder,
                                 YuvPicture *rec_pic);
  int DetermineSlowIntraModes(CodingUnit *cu, const Qp &qp,
                              const SyntaxWriter &bitstream_writer,
                              const IntraPrediction::RefState &ref_state,
                              TransformEncoder *encoder, YuvPicture *rec_pic,
                              IntraModeSet *modes_cost);

  const PictureData &pic_data_;
  const YuvPicture &orig_pic_;
  const EncoderSettings &encoder_settings_;
  CodingUnit::ResidualState best_cu_state_;
  CuWriter cu_writer_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_INTRA_SEARCH_H_
