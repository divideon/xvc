/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_LIB_INTER_SEARCH_H_
#define XVC_ENC_LIB_INTER_SEARCH_H_

#include <array>

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/inter_prediction.h"
#include "xvc_common_lib/reference_picture_lists.h"
#include "xvc_common_lib/sample_buffer.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_enc_lib/sample_metric.h"
#include "xvc_enc_lib/speed_settings.h"
#include "xvc_enc_lib/syntax_writer.h"
#include "xvc_enc_lib/transform_encoder.h"

namespace xvc {

class InterSearch : public InterPrediction {
public:
  InterSearch(int bitdepth, int max_components, const YuvPicture &orig_pic,
              const ReferencePictureLists &ref_pic_list,
              const SpeedSettings &speed_settings);

  void SearchMotion(CodingUnit *cu, const QP &qp, bool uni_prediction_only,
                    const SyntaxWriter &bitstream_writer,
                    SampleBuffer *pred_buffer);
  Distortion SearchMergeCbf(CodingUnit *cu, const QP &qp,
                            const SyntaxWriter &bitstream_writer,
                            const InterMergeCandidateList &merge_list,
                            int merge_idx, bool force_skip,
                            TransformEncoder *encoder, YuvPicture *rec_pic);
  Distortion CompressAndEvalCbf(CodingUnit *cu, const QP &qp,
                                const SyntaxWriter &bitstream_writer,
                                TransformEncoder *encoder, YuvPicture *rec_pic);

private:
  enum class SearchMethod { TZSearch, FullSearch };
  static const int kSearchRangeUni = 64;
  static const int kSearchRangeBi = 4;

  Distortion CompressSkipOnly(CodingUnit *cu, const QP &qp,
                              const SyntaxWriter &bitstream_writer,
                              TransformEncoder *encoder, YuvPicture *rec_pic);
  Distortion SearchBiIterative(CodingUnit *cu, const QP &qp,
                               const SyntaxWriter &bitstream_writer,
                               InterDir best_uni_dir,
                               Sample *pred_buf, ptrdiff_t pred_stride,
                               CodingUnit::InterState *best_state);
  template<typename TOrig>
  Distortion SearchRefIdx(CodingUnit *cu, const QP &qp, RefPicList ref_list,
                          const SyntaxWriter &bitstream_writer,
                          const DataBuffer<TOrig> &orig_buffer,
                          Sample *pred, ptrdiff_t pred_stride,
                          Distortion initial_best_cost,
                          CodingUnit::InterState *best_state,
                          CodingUnit::InterState *best_unique = nullptr,
                          Distortion *best_unique_cost = nullptr);
  template<typename TOrig>
  MotionVector MotionEstimation(const CodingUnit &cu, const QP &qp,
                                SearchMethod search_method,
                                RefPicList ref_list, int ref_idx,
                                const DataBuffer<TOrig> &orig_buffer,
                                const MotionVector &mvp,
                                const MotionVector *bipred_mv_start,
                                Sample *pred, ptrdiff_t pred_stride,
                                Distortion *out_dist);
  MotionVector FullSearch(const CodingUnit &cu, const QP &qp,
                          const MotionVector &mvp, const YuvPicture &ref_pic,
                          const MotionVector &mv_min,
                          const MotionVector &mv_max);
  template<typename TOrig>
  MotionVector SubpelSearch(const CodingUnit &cu, const QP &qp,
                            const YuvPicture &ref_pic, const MotionVector &mvp,
                            const MotionVector &mv_fullpel,
                            const DataBuffer<TOrig> &orig_buffer,
                            Sample *pred_buffer, ptrdiff_t pred_buffer_stride,
                            Distortion *out_dist);
  template<typename TOrig>
  Distortion GetSubpelDistortion(const CodingUnit &cu,
                                 const YuvPicture &ref_pic,
                                 SampleMetric *metric, int mv_x, int mv_y,
                                 const DataBuffer<TOrig> &orig_buffer,
                                 Sample *pred_buf, ptrdiff_t pred_buf_stride);
  int EvalStartMvp(const CodingUnit &cu, const QP &qp,
                   const InterPredictorList &mvp_list,
                   const YuvPicture &ref_pic, Sample *pred_buf,
                   ptrdiff_t pred_stride);
  int EvalFinalMvpIdx(const CodingUnit &cu,
                      const InterPredictorList &mvp_list,
                      const MotionVector &mv_final, int mvp_idx_start);
  MetricType GetFullpelMetric(const CodingUnit &cu);
  Bits GetInterPredBits(const CodingUnit &cu,
                        const SyntaxWriter &bitstream_writer);
  static Bits GetMvpBits(int mvp_idx, int num_mvp);
  static Bits GetMvdBits(const MotionVector &mvp, int mv_x, int mv_y,
                         int mv_scale);
  static Bits GetNumExpGolombBits(int mvd);

  const int bitdepth_;
  const int max_components_;
  const YuvPicture &orig_pic_;
  const SpeedSettings &speed_settings_;
  ResidualBufferStorage bipred_orig_buffer_;
  SampleBufferStorage bipred_pred_buffer_;
  // Mapping of ref_idx from L1 to L0 when POC is same
  std::array<int, constants::kMaxNumRefPics> same_poc_in_l0_mapping_;
  // Best uni-prediction motion estimation result for a single CU
  std::array<std::array<MotionVector, constants::kMaxNumRefPics>,
    static_cast<int>(RefPicList::kTotalNumber)> unipred_best_mv_;
  std::array<std::array<int, constants::kMaxNumRefPics>,
    static_cast<int>(RefPicList::kTotalNumber)> unipred_best_mvp_idx_;
  std::array<std::array<Distortion, constants::kMaxNumRefPics>,
    static_cast<int>(RefPicList::kTotalNumber)> unipred_best_dist_;
  // Best fullpel search mv per ref list, ref idx and picture
  std::array<std::array<MotionVector, constants::kMaxNumRefPics>,
    static_cast<int>(RefPicList::kTotalNumber)> previous_fullpel_;
  friend class TZSearch;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_INTER_SEARCH_H_
