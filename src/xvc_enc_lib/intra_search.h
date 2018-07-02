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

#ifndef XVC_ENC_LIB_INTRA_SEARCH_H_
#define XVC_ENC_LIB_INTRA_SEARCH_H_

#include <utility>

#include "xvc_common_lib/intra_prediction.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_enc_lib/syntax_writer.h"
#include "xvc_enc_lib/cu_writer.h"
#include "xvc_enc_lib/encoder_settings.h"
#include "xvc_enc_lib/encoder_simd_functions.h"
#include "xvc_enc_lib/transform_encoder.h"

namespace xvc {

class IntraSearch : public IntraPrediction {
public:
  IntraSearch(const EncoderSimdFunctions &simd, int bitdepth,
              const PictureData &pic_data, const YuvPicture &orig_pic,
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
  const SampleMetric satd_metric_;
  CodingUnit::ResidualState best_cu_state_;
  CuWriter cu_writer_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_INTRA_SEARCH_H_
