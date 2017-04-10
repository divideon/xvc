/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_LIB_TRANSFORM_ENCODER_H_
#define XVC_ENC_LIB_TRANSFORM_ENCODER_H_

#include "xvc_common_lib/sample_buffer.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/transform.h"
#include "xvc_common_lib/yuv_pic.h"

namespace xvc {

class TransformEncoder {
public:
  TransformEncoder(int bitdepth, const YuvPicture &orig_pic);

  Distortion TransformAndReconstruct(CodingUnit *cu, YuvComponent comp,
                                     const QP &qp, const YuvPicture &orig_pic,
                                     YuvPicture *rec_pic);

protected:
  SampleBufferStorage temp_pred_;
  ResidualBufferStorage temp_resi_orig_;
  ResidualBufferStorage temp_resi_;

private:
  static const ptrdiff_t kBufferStride_ = constants::kMaxBlockSize;
  const Sample min_pel_;
  const Sample max_pel_;
  InverseTransform inv_transform_;
  ForwardTransform fwd_transform_;
  Quantize quantize_;
  CoeffBufferStorage temp_coeff_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_TRANSFORM_ENCODER_H_
