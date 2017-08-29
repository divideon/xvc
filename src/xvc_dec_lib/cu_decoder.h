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
#ifndef XVC_DEC_LIB_CU_DECODER_H_
#define XVC_DEC_LIB_CU_DECODER_H_

#include <vector>

#include "xvc_common_lib/sample_buffer.h"
#include "xvc_common_lib/inter_prediction.h"
#include "xvc_common_lib/intra_prediction.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/simd_functions.h"
#include "xvc_common_lib/transform.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_dec_lib/cu_reader.h"
#include "xvc_dec_lib/syntax_reader.h"

namespace xvc {

class CuDecoder {
public:
  CuDecoder(const SimdFunctions &simd, const Qp &pic_qp,
            YuvPicture *decoded_pic, PictureData *picture_data);
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
