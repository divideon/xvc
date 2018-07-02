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
