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
  SyntaxWriter(const Qp &qp, PicturePredictionType pic_type,
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
  int WriteCoefficients(const CodingUnit &cu, YuvComponent comp,
                         const Coeff *coeff, ptrdiff_t coeff_stride);
  void WriteEndOfSlice(bool end_of_slice);
  void WriteInterDir(const CodingUnit &cu, InterDir inter_dir);
  void WriteInterFullpelMvFlag(const CodingUnit &cu, bool fullpel_mv_only);
  void WriteInterMvd(const MotionVector &mvd);
  void WriteInterMvpIdx(int mvp_idx);
  void WriteInterRefIdx(int ref_idx, int num_refs_available);
  void WriteIntraMode(IntraMode intra_mode, const IntraPredictorLuma &mpm);
  void WriteIntraChromaMode(IntraChromaMode chroma_mode,
                            IntraPredictorChroma chroma_preds);
  void WriteLicFlag(bool use_lic);
  void WriteMergeFlag(bool merge);
  void WriteMergeIdx(int merge_idx);
  void WritePartitionType(const CodingUnit &cu, PartitionType type);
  void WritePredMode(PredictionMode pred_mode);
  void WriteRootCbf(bool root_cbf);
  void WriteSkipFlag(const CodingUnit &cu, bool flag);
  void WriteSplitBinary(const CodingUnit &cu,
                        SplitRestriction split_restriction, SplitType split);
  void WriteSplitQuad(const CodingUnit &cu, int max_depth, SplitType split);
  void WriteTransformSkip(const CodingUnit &cu, YuvComponent comp,
                          bool tx_skip);
  void WriteTransformSelectEnable(const CodingUnit &cu, bool enable);
  void WriteTransformSelectIdx(const CodingUnit &cu, int type_idx);

protected:
  template<int SubBlockShift>
  int WriteCoeffSubblock(const CodingUnit &cu, YuvComponent comp,
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
  RdoSyntaxWriter(const SyntaxWriter &writer, uint32_t bits_written,
                  uint32_t frac_bits);
  RdoSyntaxWriter& operator=(const RdoSyntaxWriter &writer);
private:
  EntropyEncoder entropy_instance_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_SYNTAX_WRITER_H_
