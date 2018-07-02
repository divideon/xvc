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
  void ReadMergePrediction(CodingUnit *cu, YuvComponent comp,
                           SyntaxReader *reader);
  void ReadResidualData(CodingUnit *cu, YuvComponent comp,
                        SyntaxReader *reader);
  void ReadResidualDataInternal(CodingUnit *cu, YuvComponent comp,
                                SyntaxReader *reader) const;
  bool ReadCbfInvariant(CodingUnit *cu, YuvComponent comp,
                        SyntaxReader *reader) const;

  PictureData *pic_data_;
  const IntraPrediction &intra_pred_;
  bool ctu_has_coeffs_ = false;
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_CU_READER_H_
