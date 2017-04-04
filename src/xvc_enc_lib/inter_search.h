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
#include "xvc_enc_lib/syntax_writer.h"

namespace xvc {

class InterSearch : public InterPrediction {
public:
  enum class SearchMethod { TZSearch, FullSearch };
  struct Left { static const int index = -1; };
  struct Right { static const int index = 1; };
  struct Up { static const int index = -3; };
  struct Down { static const int index = 3; };
  struct SearchState;

  InterSearch(int bitdepth, const YuvPicture &orig_pic,
              const ReferencePictureLists &ref_pic_list);
  void Search(CodingUnit *cu, const QP &qp,
              PicturePredictionType pic_type,
              const SyntaxWriter &bitstream_writer,
              SampleBuffer *pred_buffer);

private:
  static const int kSearchRangeUni = 64;
  static const int kSearchRangeBi = 4;
  static const int kMaxIterationsBi = 1;

  template<typename TOrig> class DistortionWrapper;
  Distortion SearchBiIterative(CodingUnit *cu, const QP &qp,
                               PicturePredictionType pic_type,
                               const SyntaxWriter &bitstream_writer,
                               InterDir best_uni_dir,
                               Sample *pred_buf, ptrdiff_t pred_stride,
                               CodingUnit::InterState *best_state);
  template<typename TOrig>
  Distortion SearchRefIdx(CodingUnit *cu, const QP &qp,
                          PicturePredictionType pic_type,
                          RefPicList ref_list,
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
  MotionVector TZSearch(const CodingUnit &cu, const QP &qp,
                        const MotionVector &mvp, const YuvPicture &ref_pic,
                        const MotionVector &mv_min, const MotionVector &mv_max,
                        const MotionVector &prev_search);
  bool FullpelDiamondSearch(SearchState *state, const MotionVector &mv_base,
                            int range);
  void FullpelNeighborPointSearch(SearchState *state);
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
  void DetermineMinMaxMv(const CodingUnit &cu, const YuvPicture &ref_pic,
                         int center_x, int center_y, int search_range,
                         MotionVector *mv_min, MotionVector *mv_max);
  int EvalStartMvp(const CodingUnit &cu, const QP &qp,
                   const InterPredictorList &mvp_list,
                   const YuvPicture &ref_pic, Sample *pred_buf,
                   ptrdiff_t pred_stride);
  int EvalFinalMvpIdx(const CodingUnit &cu,
                      const InterPredictorList &mvp_list,
                      const MotionVector &mv_final, int mvp_idx_start);
  Distortion GetCost(SearchState *state, int mv_x, int mv_y);
  bool CheckCostBest(SearchState *state, int mv_x, int mv_y);
  Bits GetInterPredBits(const CodingUnit &cu,
                        const SyntaxWriter &bitstream_writer,
                        PicturePredictionType pic_pred_type);
  Bits GetMvpBits(int mvp_idx, int num_mvp);
  Bits GetMvdBits(const MotionVector &mvp, int mv_x, int mv_y, int mv_scale);
  Bits GetNumExpGolombBits(int mvd);
  template<class Dir>
  bool CheckCost1(SearchState *state, int mv_x, int mv_y, int range);
  template<class Dir1, class Dir2>
  bool CheckCost2(SearchState *state, int mv_x, int mv_y, int range);
  MetricType GetFullpelMetric(const CodingUnit &cu);

  int bitdepth_;
  const YuvPicture &orig_pic_;
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
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_INTER_SEARCH_H_
