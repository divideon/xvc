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

#ifndef XVC_ENC_LIB_CU_WRITER_H_
#define XVC_ENC_LIB_CU_WRITER_H_

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/intra_prediction.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_enc_lib/syntax_writer.h"

namespace xvc {

class CuWriter {
public:
  CuWriter(const PictureData &pic_data, const IntraPrediction *intra_pred)
    : pic_data_(pic_data),
    intra_pred_(intra_pred) {
  }
  bool WriteCtu(CodingUnit *ctu, PictureData *cu_map, SyntaxWriter *writer);
  void WriteCu(CodingUnit *cu, SplitRestriction split_restriction,
               PictureData *cu_map, SyntaxWriter *writer);
  void WriteSplit(const CodingUnit &cu, SplitRestriction split_restriction,
                  SyntaxWriter *writer);
  void WriteComponent(const CodingUnit &cu, YuvComponent comp,
                      SyntaxWriter *writer);
  void WriteIntraPrediction(const CodingUnit &cu, YuvComponent comp,
                            SyntaxWriter *writer);
  void WriteInterPrediction(const CodingUnit &cu, YuvComponent comp,
                            SyntaxWriter *writer);
  void WriteMergePrediction(const CodingUnit &cu, YuvComponent comp,
                           SyntaxWriter *writer);
  void WriteResidualData(const CodingUnit &cu, YuvComponent comp,
                         SyntaxWriter *writer);
  // Encoder only method with simplified cbf rdo signaling
  void WriteResidualDataRdoCbf(const CodingUnit &cu, YuvComponent comp,
                               SyntaxWriter *writer) const;

private:
  void WriteResidualDataInternal(const CodingUnit &cu, YuvComponent comp,
                                 SyntaxWriter *writer) const;
  bool WriteCbfInvariant(const CodingUnit &cu, YuvComponent comp,
                         SyntaxWriter *writer) const;

  const PictureData &pic_data_;
  const IntraPrediction *intra_pred_;
  bool ctu_has_coeffs_ = false;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_CU_WRITER_H_
