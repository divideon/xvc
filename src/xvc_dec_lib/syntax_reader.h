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
  virtual int ReadQp() = 0;
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
  int ReadQp();
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
