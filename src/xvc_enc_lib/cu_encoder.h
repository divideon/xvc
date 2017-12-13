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

#ifndef XVC_ENC_LIB_CU_ENCODER_H_
#define XVC_ENC_LIB_CU_ENCODER_H_

#include <memory>
#include <vector>

#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/simd_functions.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_enc_lib/cu_cache.h"
#include "xvc_enc_lib/cu_writer.h"
#include "xvc_enc_lib/inter_search.h"
#include "xvc_enc_lib/intra_search.h"
#include "xvc_enc_lib/encoder_settings.h"
#include "xvc_enc_lib/syntax_writer.h"
#include "xvc_enc_lib/transform_encoder.h"

namespace xvc {

class CuEncoder : public TransformEncoder {
public:
  CuEncoder(const SimdFunctions &simd, const YuvPicture &orig_pic,
            YuvPicture *rec_pic, PictureData *pic_data,
            const EncoderSettings &encoder_settings);
  ~CuEncoder();
  void EncodeCtu(int rsaddr, SyntaxWriter *writer);

private:
  enum class RdMode {
    INTER_ME,
    INTER_FULLPEL,
    INTER_LIC,
    INTER_LIC_FULLPEL,
  };
  struct RdoCost;

  Distortion CompressCu(CodingUnit **cu, int rdo_depth,
                        SplitRestriction split_restiction,
                        RdoSyntaxWriter *rdo_writer,
                        const Qp &qp);
  RdoCost CompressSplitCu(CodingUnit *cu, int rdo_depth, const Qp &qp,
                          SplitType split_type, SplitRestriction split_restrct,
                          RdoSyntaxWriter *rdo_writer);
  Distortion CompressNoSplit(CodingUnit **cu, int rdo_depth,
                             SplitRestriction split_restrct,
                             RdoSyntaxWriter *rdo_writer);
  Distortion CompressFast(CodingUnit *cu, const Qp &qp,
                          const SyntaxWriter &writer);
  RdoCost CompressInterPic(CodingUnit **best_cu_ref, CodingUnit **temp_cu_ref,
                           const Qp &qp, int rdo_depth,
                           const CuCache::Result &cache_result,
                           const SyntaxWriter &bitstream_writer);
  RdoCost CompressIntra(CodingUnit *cu, const Qp &qp,
                        const SyntaxWriter &bitstream_writer);
  RdoCost CompressInter(CodingUnit *cu, const Qp &qp,
                        const SyntaxWriter &bitstream_writer, RdMode rd_mode,
                        Cost best_cu_cost);
  RdoCost CompressMerge(CodingUnit *cu, const Qp &qp,
                        const SyntaxWriter &bitstream_writer,
                        Cost best_cu_cost, bool fast_merge_skip);
  RdoCost CompressAffineMerge(CodingUnit *cu, const Qp &qp,
                              const SyntaxWriter &bitstream_writer,
                              Cost best_cu_cost);
  RdoCost GetCuCostWithoutSplit(const CodingUnit &cu, const Qp &qp,
                                const SyntaxWriter &bitstream_writer,
                                Distortion ssd);
  int CalcDeltaQpFromVariance(const CodingUnit *cu);
  void WriteCtu(int rsaddr, SyntaxWriter *writer);
  void SetQpForAllCusInCtu(CodingUnit *ctu, int qp);

  static bool CanSkipAnySplitForCu(const PictureData &pic_data,
                                   const CodingUnit &cu);
  static bool CanSkipQuadSplitForCu(const PictureData &pic_data,
                                    const CodingUnit &cu);

  const YuvPicture &orig_pic_;
  const EncoderSettings &encoder_settings_;
  YuvPicture &rec_pic_;
  PictureData &pic_data_;
  InterSearch inter_search_;
  IntraSearch intra_search_;
  CuWriter cu_writer_;
  CuCache cu_cache_;
  uint32_t last_ctu_frac_bits_ = 0;
  // +2 for allow access to one depth lower than smallest CU in RDO
  std::array<CodingUnit::ReconstructionState,
    constants::kMaxBlockDepth + 2> temp_cu_state_;
  CodingUnit::ResidualState rd_transform_state_;
  std::array<std::array<CodingUnit*, constants::kMaxBlockDepth + 2>,
    constants::kMaxNumCuTrees> rdo_temp_cu_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_CU_ENCODER_H_
