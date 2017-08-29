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
  bool ReadCtu(CodingUnit *cu, SyntaxReader *reader) {
    ctu_has_coeffs_ = false;
    ReadCu(cu, SplitRestriction::kNone, reader);
    return ctu_has_coeffs_;
  }

private:
  void ReadCu(CodingUnit *cu, SplitRestriction split_restrict,
              SyntaxReader *reader);
  SplitType ReadSplit(CodingUnit *cu, SplitRestriction split_restriction,
                      SyntaxReader *reader);
  void ReadComponent(CodingUnit *cu, YuvComponent comp, SyntaxReader *reader);
  void ReadIntraPrediction(CodingUnit *cu, YuvComponent comp,
                           SyntaxReader *reader);
  void ReadInterPrediction(CodingUnit *cu, YuvComponent comp,
                           SyntaxReader *reader);
  void ReadCoefficients(CodingUnit *cu, YuvComponent comp,
                        SyntaxReader *reader);

  PictureData *pic_data_;
  const IntraPrediction &intra_pred_;
  bool ctu_has_coeffs_ = false;
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_CU_READER_H_
