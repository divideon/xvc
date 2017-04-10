/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_DEC_LIB_CU_READER_H_
#define XVC_DEC_LIB_CU_READER_H_

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_dec_lib/syntax_reader.h"

namespace xvc {

class CuReader {
public:
  CuReader(PictureData *pic_data, const IntraPrediction &intra_pred)
    : pic_data_(pic_data),
    intra_pred_(intra_pred) {
  }
  void ReadCu(CodingUnit *cu, SyntaxReader *reader);

private:
  SplitType ReadSplit(CodingUnit *cu, SyntaxReader *reader);
  void ReadComponent(CodingUnit *cu, YuvComponent comp, SyntaxReader *reader);
  void ReadIntraPrediction(CodingUnit *cu, YuvComponent comp,
                           SyntaxReader *reader);
  void ReadInterPrediction(CodingUnit *cu, YuvComponent comp,
                           SyntaxReader *reader);
  void ReadCoefficients(CodingUnit *cu, YuvComponent comp,
                        SyntaxReader *reader);

  PictureData *pic_data_;
  const IntraPrediction &intra_pred_;
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_CU_READER_H_
