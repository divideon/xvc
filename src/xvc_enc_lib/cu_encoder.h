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

#ifndef XVC_ENC_LIB_CU_ENCODER_H_
#define XVC_ENC_LIB_CU_ENCODER_H_

#include <memory>
#include <vector>

#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_enc_lib/cu_cache.h"
#include "xvc_enc_lib/cu_writer.h"
#include "xvc_enc_lib/inter_search.h"
#include "xvc_enc_lib/intra_search.h"
#include "xvc_enc_lib/encoder_settings.h"
#include "xvc_enc_lib/encoder_simd_functions.h"
#include "xvc_enc_lib/syntax_writer.h"
#include "xvc_enc_lib/transform_encoder.h"

namespace xvc {

class CuEncoder : public TransformEncoder {
public:
  CuEncoder(const EncoderSimdFunctions &simd, const YuvPicture &orig_pic,
            YuvPicture *rec_pic, PictureData *pic_data,
            const EncoderSettings &encoder_settings);
  ~CuEncoder();
  void EncodeCtu(int rsaddr, SyntaxWriter *writer);

private:
  enum class RdMode {
    kInterMe,
    kInterFullpel,
    kInterLic,
    kInterLicFullpel,
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
  bool CanSkipAnySplitForCu(const CodingUnit &cu) const;
  bool CanSkipQuadSplitForCu(const CodingUnit &cu,
                             bool binary_depth_greater_than_one) const;

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
