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
  CuDecoder(const SimdFunctions &simd, YuvPicture *decoded_pic,
            PictureData *picture_data);
  void DecodeCtu(int rsaddr, SyntaxReader *reader);

private:
  static const ptrdiff_t kBufferStride_ = constants::kMaxBlockSize;
  void ReadCtu(int rsaddr, SyntaxReader *reader);
  void DecompressCu(CodingUnit *cu);
  void DecompressComponent(CodingUnit *cu, YuvComponent comp, const Qp &qp);
  void PredictIntra(const CodingUnit &cu, YuvComponent comp,
                    SampleBuffer *pred_buffer);

  const Sample min_pel_;
  const Sample max_pel_;
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
