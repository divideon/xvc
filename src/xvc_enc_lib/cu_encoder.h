/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_LIB_CU_ENCODER_H_
#define XVC_ENC_LIB_CU_ENCODER_H_

#include <memory>
#include <vector>


#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_enc_lib/cu_writer.h"
#include "xvc_enc_lib/inter_search.h"
#include "xvc_enc_lib/intra_search.h"
#include "xvc_enc_lib/speed_settings.h"
#include "xvc_enc_lib/syntax_writer.h"
#include "xvc_enc_lib/transform_encoder.h"

namespace xvc {

class CuEncoder : public TransformEncoder {
public:
  CuEncoder(const QP &qp, const YuvPicture &orig_pic, YuvPicture *rec_pic,
            PictureData *pic_data, const SpeedSettings &speed_settings);
  ~CuEncoder();
  void EncodeCtu(int rsaddr, SyntaxWriter *writer);

private:
  struct RdoCost;
  Distortion CompressCu(CodingUnit **cu, int rdo_depth,
                        SplitRestriction split_restiction,
                        RdoSyntaxWriter *rdo_writer);
  RdoCost CompressSplitCu(CodingUnit *cu, int rdo_depth, const QP &qp,
                          SplitType split_type, SplitRestriction split_restrct,
                          RdoSyntaxWriter *rdo_writer);
  Distortion CompressNoSplit(CodingUnit **cu, int rdo_depth,
                             SplitRestriction split_restrct,
                             RdoSyntaxWriter *rdo_writer);
  RdoCost CompressIntra(CodingUnit *cu, const QP &qp,
                        const SyntaxWriter &bitstream_writer);
  RdoCost CompressInter(CodingUnit *cu, const QP &qp,
                        const SyntaxWriter &bitstream_writer);
  RdoCost CompressMerge(CodingUnit *cu, const QP &qp,
                        const SyntaxWriter &bitstream_writer);
  RdoCost GetCuCostWithoutSplit(const CodingUnit &cu, const QP &qp,
                                const SyntaxWriter &bitstream_writer,
                                Distortion ssd);
  void WriteCtu(int rsaddr, SyntaxWriter *writer);

  const QP &pic_qp_;
  const YuvPicture &orig_pic_;
  const SpeedSettings &speed_settings_;
  YuvPicture &rec_pic_;
  PictureData &pic_data_;
  InterSearch inter_search_;
  IntraSearch intra_search_;
  CuWriter cu_writer_;
  // +2 for allow access to one depth lower than smallest CU in RDO
  std::array<CodingUnit::ReconstructionState,
    constants::kMaxBlockDepth + 2> temp_cu_state_;
  std::array<std::array<CodingUnit*, constants::kMaxBlockDepth + 2>,
    constants::kMaxNumCuTrees> rdo_temp_cu_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_CU_ENCODER_H_
