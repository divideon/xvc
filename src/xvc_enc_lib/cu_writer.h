/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_LIB_CU_WRITER_H_
#define XVC_ENC_LIB_CU_WRITER_H_

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/intra_prediction.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_enc_lib/syntax_writer.h"

namespace xvc {

class CuWriter {
public:
  CuWriter(const PictureData &pic_data, const IntraPrediction &intra_pred)
    : pic_data_(pic_data),
    intra_pred_(intra_pred) {
  }
  void WriteCu(const CodingUnit &cu, SyntaxWriter *writer) const;
  void WriteComponent(const CodingUnit &cu, YuvComponent comp,
                      SyntaxWriter *writer)  const;
  void WriteIntraPrediction(const CodingUnit &cu, YuvComponent comp,
                            SyntaxWriter *writer) const;
  void WriteInterPrediction(const CodingUnit &cu, YuvComponent comp,
                            SyntaxWriter *writer) const;
  void WriteCoefficients(const CodingUnit &cu, YuvComponent comp,
                         SyntaxWriter *writer) const;

private:
  const PictureData &pic_data_;
  const IntraPrediction &intra_pred_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_CU_WRITER_H_
