/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_DEC_LIB_CU_DECODER_H_
#define XVC_DEC_LIB_CU_DECODER_H_

#include <vector>

#include "xvc_common_lib/sample_buffer.h"
#include "xvc_common_lib/inter_prediction.h"
#include "xvc_common_lib/intra_prediction.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/transform.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_dec_lib/cu_reader.h"
#include "xvc_dec_lib/syntax_reader.h"

namespace xvc {

class CuDecoder {
public:
  CuDecoder(const Qp &pic_qp, YuvPicture *decoded_pic,
            PictureData *picture_data);
  void DecodeCtu(int rsaddr, SyntaxReader *reader);

private:
  static const ptrdiff_t kBufferStride_ = constants::kMaxBlockSize;
  void ReadCtu(int rsaddr, SyntaxReader *reader);
  void DecompressCu(CodingUnit *cu);
  void DecompressComponent(CodingUnit *cu, YuvComponent comp, const Qp &qp);

  const Sample min_pel_;
  const Sample max_pel_;
  const Qp &pic_qp_;
  YuvPicture &decoded_pic_;
  PictureData &pic_data_;
  InterPrediction inter_pred_;
  IntraPrediction intra_pred_;
  InverseTransform inv_transform_;
  Quantize quantize_;
  CuReader cu_reader_;
  SampleBufferStorage temp_pred_;
  ResidualBufferStorage temp_resi_;
  CoeffBufferStorage temp_coeff_;
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_CU_DECODER_H_
