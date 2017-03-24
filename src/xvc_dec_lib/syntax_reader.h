/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_DEC_LIB_SYNTAX_READER_H_
#define XVC_DEC_LIB_SYNTAX_READER_H_

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
  SyntaxReader(const QP &qp, PicturePredictionType pic_type,
               EntropyDecoder *entropydec);
  bool ReadCbf(const CodingUnit &cu, YuvComponent comp);
  void ReadCoefficients(const CodingUnit &cu, YuvComponent comp,
                        Coeff *dst_coeff, ptrdiff_t dst_coeff_stride);
  bool ReadEndOfSlice();
  IntraMode ReadIntraMode(const IntraPredictorLuma &mpm);
  InterDir ReadInterDir(const CodingUnit &cu);
  MotionVector ReadInterMvd();
  int ReadInterMvpIdx();
  int ReadInterRefIdx(int num_refs_available);
  IntraChromaMode ReadIntraChromaMode(IntraPredictorChroma chroma_preds);
  bool ReadMergeFlag();
  int ReadMergeIdx();
  PartitionType ReadPartitionType(const CodingUnit &cu);
  PredictionMode ReadPredMode();
  bool ReadRootCbf();
  bool ReadSkipFlag(const CodingUnit &cu);
  bool ReadSplitFlag(const CodingUnit &cu);

private:
  void ReadCoeffLastPos(int width, int height, YuvComponent comp,
                        ScanOrder scan_order, uint32_t *pos_last_x,
                        uint32_t *pos_last_y);
  uint32_t ReadCoeffRemainExpGolomb(uint32_t golomb_rice_k);
  uint32_t ReadExpGolomb(uint32_t golomb_rice_k);
  uint32_t ReadUnaryMaxSymbol(uint32_t max_val, ContextModel *ctx_start,
                              ContextModel *ctx_rest);

  CabacContexts ctx_;
  EntropyDecoder *entropydec_;
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_SYNTAX_READER_H_
