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

#ifndef XVC_DEC_LIB_SYNTAX_READER_H_
#define XVC_DEC_LIB_SYNTAX_READER_H_

#include <memory>

#include "xvc_common_lib/cabac.h"
#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/intra_prediction.h"
#include "xvc_common_lib/picture_types.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/transform.h"
#include "xvc_dec_lib/entropy_decoder.h"

namespace xvc {

class SyntaxReader {
public:
  static std::unique_ptr<SyntaxReader>
    Create(const Qp &qp, PicturePredictionType pic_type,
           BitReader *bit_reader);
  virtual ~SyntaxReader() {}
  virtual bool Finish() = 0;

  virtual bool ReadAffineFlag(const CodingUnit &cu, bool is_merge) = 0;
  virtual bool ReadCbf(const CodingUnit &cu, YuvComponent comp) = 0;
  virtual int ReadCoefficients(const CodingUnit &cu, YuvComponent comp,
                               Coeff *dst_coeff,
                               ptrdiff_t dst_coeff_stride) = 0;
  virtual bool ReadEndOfSlice() = 0;
  virtual InterDir ReadInterDir(const CodingUnit &cu) = 0;
  virtual bool ReadInterFullpelMvFlag(const CodingUnit &cu) = 0;
  virtual MvDelta ReadInterMvd() = 0;
  virtual int ReadInterMvpIdx(const CodingUnit &cu) = 0;
  virtual int ReadInterRefIdx(int num_refs_available) = 0;
  virtual IntraMode ReadIntraMode(const IntraPredictorLuma &mpm) = 0;
  virtual IntraChromaMode ReadIntraChromaMode(
    IntraPredictorChroma chroma_preds) = 0;
  virtual bool ReadLicFlag() = 0;
  virtual bool ReadMergeFlag() = 0;
  virtual int ReadMergeIdx() = 0;
  virtual PartitionType ReadPartitionType(const CodingUnit &cu) = 0;
  virtual PredictionMode ReadPredMode() = 0;
  virtual int ReadQp(int predicted_qp, int base_qp, int aqp_mode) = 0;
  virtual bool ReadRootCbf() = 0;
  virtual bool ReadSkipFlag(const CodingUnit &cu) = 0;
  virtual SplitType ReadSplitBinary(const CodingUnit &cu,
                                    SplitRestriction split_restriction) = 0;
  virtual SplitType ReadSplitQuad(const CodingUnit &cu, int max_depth) = 0;
  virtual bool ReadTransformSkip(const CodingUnit &cu, YuvComponent comp) = 0;
  virtual bool ReadTransformSelectEnable(const CodingUnit &cu) = 0;
  virtual int ReadTransformSelectIdx(const CodingUnit &cu) = 0;
};

template<typename ContextModel>
class SyntaxReaderCabac final : public SyntaxReader {
public:
  SyntaxReaderCabac(const Qp &qp, PicturePredictionType pic_type,
                    BitReader *bit_reader);
  bool Finish();
  bool ReadAffineFlag(const CodingUnit &cu, bool is_merge);
  bool ReadCbf(const CodingUnit &cu, YuvComponent comp);
  int ReadCoefficients(const CodingUnit &cu, YuvComponent comp,
                       Coeff *dst_coeff, ptrdiff_t dst_coeff_stride);
  bool ReadEndOfSlice();
  InterDir ReadInterDir(const CodingUnit &cu);
  bool ReadInterFullpelMvFlag(const CodingUnit &cu);
  MvDelta ReadInterMvd();
  int ReadInterMvpIdx(const CodingUnit &cu);
  int ReadInterRefIdx(int num_refs_available);
  IntraMode ReadIntraMode(const IntraPredictorLuma &mpm);
  IntraChromaMode ReadIntraChromaMode(IntraPredictorChroma chroma_preds);
  bool ReadLicFlag();
  bool ReadMergeFlag();
  int ReadMergeIdx();
  PartitionType ReadPartitionType(const CodingUnit &cu);
  PredictionMode ReadPredMode();
  int ReadQp(int predicted_qp, int base_qp, int aqp_mode);
  bool ReadRootCbf();
  bool ReadSkipFlag(const CodingUnit &cu);
  SplitType ReadSplitBinary(const CodingUnit &cu,
                            SplitRestriction split_restriction);
  SplitType ReadSplitQuad(const CodingUnit &cu, int max_depth);
  bool ReadTransformSkip(const CodingUnit &cu, YuvComponent comp);
  bool ReadTransformSelectEnable(const CodingUnit &cu);
  int ReadTransformSelectIdx(const CodingUnit &cu);

private:
  template<int SubBlockShift>
  int ReadCoeffSubblock(const CodingUnit &cu, YuvComponent comp,
                        Coeff *dst_coeff, ptrdiff_t dst_coeff_stride);
  void ReadCoeffLastPos(int width, int height, YuvComponent comp,
                        ScanOrder scan_order, uint32_t *pos_last_x,
                        uint32_t *pos_last_y);
  template<int SubBlockShift>
  int DetermineLastIndex(int subblock_width, int subblock_height,
                         int pos_last_x, int pos_last_y,
                         const uint16_t *subblock_scan_table,
                         const uint8_t *coeff_scan_table);
  uint32_t ReadCoeffRemainExpGolomb(uint32_t golomb_rice_k);
  uint32_t ReadExpGolomb(uint32_t golomb_rice_k);
  uint32_t ReadUnaryMaxSymbol(uint32_t max_val, ContextModel *ctx_start,
                              ContextModel *ctx_rest);

  CabacContexts<ContextModel> ctx_;
  EntropyDecoder<ContextModel> decoder_;
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_SYNTAX_READER_H_
