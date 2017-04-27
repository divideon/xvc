/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_LIB_SYNTAX_WRITER_H_
#define XVC_ENC_LIB_SYNTAX_WRITER_H_

#include "xvc_common_lib/cabac.h"
#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/intra_prediction.h"
#include "xvc_common_lib/picture_types.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/transform.h"
#include "xvc_enc_lib/entropy_encoder.h"

namespace xvc {

class SyntaxWriter {
public:
  SyntaxWriter(const QP &qp, PicturePredictionType pic_type,
               EntropyEncoder *entropyenc);
  SyntaxWriter(const CabacContexts &contexts, EntropyEncoder *entropyenc);
  const CabacContexts &GetContexts() const { return ctx_; }
  Bits GetNumWrittenBits() const {
    return entropyenc_->GetNumWrittenBits();
  }
  Bits GetFractionalBits() const {
    return entropyenc_->GetFractionalBits();
  }
  void ResetBitCounting() { entropyenc_->ResetBitCounting(); }

  void WriteCbf(const CodingUnit &cu, YuvComponent comp, bool cbf);
  void WriteQp(int qp_value);
  void WriteCoefficients(const CodingUnit &cu, YuvComponent comp,
                         const Coeff *coeff, ptrdiff_t coeff_stride);
  void WriteEndOfSlice(bool end_of_slice);
  void WriteInterDir(const CodingUnit &cu, InterDir inter_dir);
  void WriteInterMvd(const MotionVector &mvd);
  void WriteInterMvpIdx(int mvp_idx);
  void WriteInterRefIdx(int ref_idx, int num_refs_available);
  void WriteIntraMode(IntraMode intra_mode, const IntraPredictorLuma &mpm);
  void WriteIntraChromaMode(IntraChromaMode chroma_mode,
                            IntraPredictorChroma chroma_preds);
  void WriteMergeFlag(bool merge);
  void WriteMergeIdx(int merge_idx);
  void WritePartitionType(const CodingUnit &cu, PartitionType type);
  void WritePredMode(PredictionMode pred_mode);
  void WriteRootCbf(bool root_cbf);
  void WriteSkipFlag(const CodingUnit &cu, bool flag);
  void WriteSplitBinary(const CodingUnit &cu,
                        SplitRestriction split_restriction, SplitType split);
  void WriteSplitQuad(const CodingUnit &cu, int max_depth, SplitType split);

protected:
  template<int SubBlockShift>
  void WriteCoeffSubblock(const CodingUnit &cu, YuvComponent comp,
                          const Coeff *coeff, ptrdiff_t coeff_stride);
  void WriteCoeffLastPos(int width, int height, YuvComponent comp,
                         ScanOrder scan_order, int last_pos_x,
                         int last_pos_y);
  void WriteCoeffRemainExpGolomb(uint32_t abs_level, uint32_t golomb_rice_k);
  void WriteExpGolomb(uint32_t abs_level, uint32_t golomb_rice_k);
  void WriteUnaryMaxSymbol(uint32_t symbol, uint32_t max_val,
                           ContextModel *ctx_start, ContextModel *ctx_rest);

  CabacContexts ctx_;
  EntropyEncoder *entropyenc_;
};

class RdoSyntaxWriter : public SyntaxWriter {
public:
  explicit RdoSyntaxWriter(const SyntaxWriter &writer);
  explicit RdoSyntaxWriter(const RdoSyntaxWriter &writer);
  RdoSyntaxWriter(const SyntaxWriter &writer, uint32_t bits_written);
  RdoSyntaxWriter& operator=(const RdoSyntaxWriter &writer);
private:
  EntropyEncoder entropy_instance_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_SYNTAX_WRITER_H_
