/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_LIB_CU_ENCODER_H_
#define XVC_ENC_LIB_CU_ENCODER_H_

#include <memory>
#include <vector>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/sample_buffer.h"
#include "xvc_common_lib/intra_prediction.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/transform.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_enc_lib/cu_writer.h"
#include "xvc_enc_lib/inter_search.h"
#include "xvc_enc_lib/syntax_writer.h"

namespace xvc {

class CuEncoder {
public:
  CuEncoder(const QP &qp, const YuvPicture &orig_pic, YuvPicture *rec_pic,
            PictureData *pic_data);
  ~CuEncoder();
  void EncodeCtu(int rsaddr, SyntaxWriter *writer);

private:
  static const ptrdiff_t kBufferStride_ = constants::kMaxBlockSize;
  struct RdoCost;
  Distortion CompressCu(CodingUnit **cu, RdoSyntaxWriter *rdo_writer);
  Distortion CompressSplitCu(CodingUnit *cu, RdoSyntaxWriter *rdo_writer,
                             Bits *frac_bits_before_split);
  Distortion CompressNoSplit(CodingUnit **cu, RdoSyntaxWriter *rdo_writer);

  RdoCost CompressIntra(CodingUnit *cu, const QP &qp,
                         const SyntaxWriter &bitstream_writer);
  RdoCost CompressInter(CodingUnit *cu, const QP &qp,
                         const SyntaxWriter &bitstream_writer);
  RdoCost CompressMerge(CodingUnit *cu, const QP &qp,
                         const SyntaxWriter &bitstream_writer);
  Distortion CompressAndEvalCbf(CodingUnit *cu, const QP &qp,
                                const SyntaxWriter &bitstream_writer,
                                Distortion *out_dist_zero = nullptr);
  Distortion CompressSkipOnly(CodingUnit *cu, const QP &qp,
                              const SyntaxWriter &bitstream_writer,
                              bool calc_distortion = true);
  bool SearchCompCbfZero(CodingUnit *cu, const QP &qp, YuvComponent comp,
                         const SyntaxWriter &bitstream_writer,
                         Distortion dist_non_zero, Distortion dist_zero);
  bool SearchCbfAllZero(CodingUnit *cu, const QP &qp,
                        const SyntaxWriter &bitstream_writer,
                        Distortion sum_dist_non_zero,
                        Distortion sum_dist_zero);
  IntraMode SearchIntraLuma(CodingUnit *cu, YuvComponent comp, const QP &qp,
                            const SyntaxWriter &bitstream_writer);
  IntraChromaMode SearchIntraChroma(CodingUnit *cu, YuvComponent comp,
                                    const QP &qp,
                                    const SyntaxWriter &bitstream_writer);
  Distortion CompressComponent(CodingUnit *cu, YuvComponent comp, const QP &qp);
  RdoCost ComputeDistCostNoSplit(const CodingUnit &cu, const QP &qp,
                                 const SyntaxWriter &bitstream_writer,
                                 Distortion ssd);
  void WriteCtu(CodingUnit *ctu, SyntaxWriter *writer);

  const Sample min_pel_;
  const Sample max_pel_;
  const QP &pic_qp_;
  const YuvPicture &orig_pic_;
  YuvPicture &rec_pic_;
  PictureData &pic_data_;
  InterSearch inter_search_;
  IntraPrediction intra_pred_;
  InverseTransform inv_transform_;
  ForwardTransform fwd_transform_;
  Quantize quantize_;
  CuWriter cu_writer_;
  SampleBufferStorage temp_pred_;
  ResidualBufferStorage temp_resi_orig_;
  ResidualBufferStorage temp_resi_;
  CoeffBufferStorage temp_coeff_;
  std::array<CodingUnit::ReconstructionState,
    constants::kMaxCuDepth + 2> temp_cu_state_;
  std::array<CodingUnit*, constants::kMaxCuDepth + 1> rdo_temp_cu_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_CU_ENCODER_H_
