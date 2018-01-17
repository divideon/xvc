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

#ifndef XVC_ENC_LIB_INTER_SEARCH_H_
#define XVC_ENC_LIB_INTER_SEARCH_H_

#include <array>

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/inter_prediction.h"
#include "xvc_common_lib/reference_picture_lists.h"
#include "xvc_common_lib/sample_buffer.h"
#include "xvc_common_lib/simd_functions.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_enc_lib/encoder_settings.h"
#include "xvc_enc_lib/sample_metric.h"
#include "xvc_enc_lib/syntax_writer.h"
#include "xvc_enc_lib/transform_encoder.h"

namespace xvc {

enum class InterSearchFlags {
  kDefault = 0,
  kUniPredOnly = 1 << 0,
  kFullPelMv = 1 << 1,
  kLic = 1 << 2,
  kAffine = 1 << 3,
};

class InterSearch : public InterPrediction {
public:
  using MergeCandLookup = std::array<int, constants::kNumInterMergeCandidates>;

  InterSearch(const SimdFunctions &simd, const PictureData &pic_data,
              const YuvPicture &orig_pic,
              const YuvPicture &rec_pic,
              const ReferencePictureLists &ref_pic_list,
              const EncoderSettings &encoder_settings);
  Distortion CompressInter(CodingUnit *cu, const Qp &qp,
                           const SyntaxWriter &bitstream_writer,
                           InterSearchFlags search_flags, Cost best_cu_cost,
                           TransformEncoder *encoder, YuvPicture *rec_pic);
  Distortion CompressInterFast(CodingUnit *cu, YuvComponent comp, const Qp &qp,
                               const SyntaxWriter &bitstream_writer,
                               TransformEncoder *encoder, YuvPicture *rec_pic);
  Distortion CompressMergeCand(CodingUnit *cu, const Qp &qp,
                               const SyntaxWriter &bitstream_writer,
                               const InterMergeCandidateList &merge_list,
                               int merge_idx, bool force_skip,
                               Cost best_cu_cost, TransformEncoder *encoder,
                               YuvPicture *rec_pic);
  Distortion CompressAffineMerge(CodingUnit *cu, const Qp &qp,
                                 const SyntaxWriter &bitstream_writer,
                                 const AffineMergeCandidate &merge_cand,
                                 bool force_skip,
                                 Cost best_cu_cost, TransformEncoder *encoder,
                                 YuvPicture *rec_pic);
  int SearchMergeCandidates(CodingUnit *cu, const Qp &qp,
                            const SyntaxWriter &bitstream_writer,
                            const InterMergeCandidateList &merge_list,
                            TransformEncoder *encoder,
                            MergeCandLookup *out_cand_list);

private:
  enum class SearchMethod { TzSearch, FullSearch };
  static const int kSearchRangeBi = 4;
  static constexpr int kFastMergeNumCand = 4;
  static constexpr double kFastMergeCostFactor = 1.25;
  static constexpr double kFastTransformSelectCostFactor = 1.1;
  static constexpr int kNumMvp = constants::kNumInterMvPredictors;

  Distortion SearchMotion(CodingUnit *cu, const Qp &qp,
                          const SyntaxWriter &bitstream_writer,
                          InterSearchFlags search_flags,
                          SampleBuffer *pred_buffer);
  Distortion CompressAndEvalCbf(CodingUnit *cu, const Qp &qp,
                                const SyntaxWriter &bitstream_writer,
                                Cost best_cu_cost, TransformEncoder *encoder,
                                YuvPicture *rec_pic);
  Distortion CompressSkipOnly(CodingUnit *cu, const Qp &qp,
                              const SyntaxWriter &bitstream_writer,
                              TransformEncoder *encoder, YuvPicture *rec_pic);
  Distortion SearchBiIterative(CodingUnit *cu, const Qp &qp,
                               const SyntaxWriter &bitstream_writer,
                               InterDir best_uni_dir, SampleBuffer *pred_buffer,
                               CodingUnit::InterState *best_state);
  template<typename TOrig>
  Distortion SearchRefIdx(CodingUnit *cu, const Qp &qp, RefPicList ref_list,
                          const SyntaxWriter &bitstream_writer,
                          const DataBuffer<TOrig> &orig_buffer,
                          Distortion initial_best_cost,
                          SampleBuffer *pred_buffer,
                          CodingUnit::InterState *best_state,
                          CodingUnit::InterState *best_unique = nullptr,
                          Distortion *best_unique_cost = nullptr);
  template<bool IsAffine, typename MotionVec, typename TOrig>
  Distortion SearchRefIdx(CodingUnit *cu, const Qp &qp, RefPicList ref_list,
                          const SyntaxWriter &bitstream_writer,
                          const DataBuffer<TOrig> &orig_buffer,
                          Distortion initial_best_cost,
                          SampleBuffer *pred_buffer,
                          CodingUnit::InterState *best_state,
                          CodingUnit::InterState *best_unique = nullptr,
                          Distortion *best_unique_cost = nullptr);
  template<typename TOrig>
  MotionVector MotionEstimation(const CodingUnit &cu, const Qp &qp,
                                SearchMethod search_method,
                                RefPicList ref_list, int ref_idx,
                                bool bipred,
                                const DataBuffer<TOrig> &orig_buffer,
                                const MotionVector &mvp,
                                const MotionVector *mv_bootstrap,
                                SampleBuffer *pred_buffer,
                                Distortion *out_dist);
  template<typename TOrig>
  MotionVector3 MotionEstimation(const CodingUnit &cu, const Qp &qp,
                                 SearchMethod search_method,
                                 RefPicList ref_list, int ref_idx,
                                 bool bipred,
                                 const DataBuffer<TOrig> &orig_buffer,
                                 const MotionVector3 &mvp,
                                 const MotionVector3 *mv_bootstrap,
                                 SampleBuffer *pred_buffer,
                                 Distortion *out_dist);
  template<typename TOrig>
  MotionVector MotionEstNormal(const CodingUnit &cu, const Qp &qp,
                               SearchMethod search_method,
                               RefPicList ref_list, int ref_idx, bool bipred,
                               const DataBuffer<TOrig> &orig_buffer,
                               const MotionVector &mvp,
                               const MotionVector *mv_bootstrap,
                               SampleBuffer *pred_buffer, Distortion *out_dist);
  template<typename TOrig>
  MotionVector3 MotionEstAffine(const CodingUnit &cu, const Qp &qp,
                                SearchMethod search_method,
                                RefPicList ref_list, int ref_idx, bool bipred,
                                const DataBuffer<TOrig> &orig_buffer,
                                const MotionVector3 &mvp,
                                const MotionVector3 *mv_bootstrap,
                                SampleBuffer *pred_buffer,
                                Distortion *out_dist);
  MvDelta2 AffineGradientSearch(int width, int height,
                                const SampleBuffer &pred_buffer,
                                const ResidualBuffer &err_buffer);
  MvFullpel FullSearch(const CodingUnit &cu, const Qp &qp,
                       const SampleMetric &metric,
                       const MotionVector &mvp, const YuvPicture &ref_pic,
                       const MvFullpel &mv_min,
                       const MvFullpel &mv_max);
  template<typename TOrig>
  MotionVector SubpelSearch(const CodingUnit &cu, const Qp &qp,
                            const SampleMetric &metric,
                            const YuvPicture &ref_pic, const MotionVector &mvp,
                            const MvFullpel &mv_fullpel,
                            const DataBuffer<TOrig> &orig_buffer,
                            SampleBuffer *pred_buffer, Distortion *out_dist);
  template<typename TOrig>
  Distortion GetSubpelDist(const CodingUnit &cu, const Qp &qp,
                           const YuvPicture &ref_pic,
                           const SampleMetric &metric, const MotionVector &mv,
                           const DataBuffer<TOrig> &orig_buffer,
                           SampleBuffer *pred_buffer);
  template<bool IsAffine, typename MotionVec>
  int EvalStartMvp(const CodingUnit &cu, const Qp &qp,
                   const std::array<MotionVec, kNumMvp> &mvp_list,
                   const YuvPicture &ref_pic, SampleBuffer *pred_buffer,
                   Distortion *best_cost);
  template<typename MotionVec>
  int EvalFinalMvpIdx(const CodingUnit &cu,
                      const std::array<MotionVec, kNumMvp> &mvp_list,
                      const MotionVec &mv_final, int mvp_idx_start);
  template<typename MotionVec>
  void SetMvd(CodingUnit *cu, RefPicList ref_list, const MotionVec &mvp,
              const MotionVec &mv);
  int GetSearchRangeUniPred(PicNum ref_poc) const;
  MetricType GetFullpelMetric(const CodingUnit &cu) const;
  MetricType GetSubpelMetric(const CodingUnit &cu) const;
  MetricType GetMvpMetricType(const CodingUnit &cu) const;
  Bits GetInterPredBits(const CodingUnit &cu,
                        const SyntaxWriter &bitstream_writer);
  static Bits GetMvpBits(int mvp_idx, int num_mvp);
  static Bits GetMvdBits(const MotionVector &mvp, const MotionVector &mv,
                         int mvd_down_shift);
  static Bits GetMvdBits(const MotionVector3 &mvp, const MotionVector3 &mv,
                         int mvd_down_shift);
  static Bits GetMvdBitsFullpel(const MotionVector &mvp, int mv_x, int mv_y,
                                int mvd_down_shift);
  static Bits GetNumExpGolombBits(int mvd);
  // Affine template helper functions
  template<bool IsAffine, typename MotionVec>
  std::array<MotionVec, kNumMvp>
    GetMvpListHelper(const CodingUnit &cu, RefPicList ref_list, int ref_idx,
                     int max_num_mvp);
  template<bool IsAffine, typename MotionVec>
  const MotionVec& GetBestUniPredMv(RefPicList ref_list, int ref_idx) const;
  template<bool IsAffine, typename MotionVec>
  void SetBestUniPredMv(RefPicList ref_list, int ref_idx, const MotionVec &mv);

  const int bitdepth_;
  const int max_components_;
  const PicNum poc_;
  const int sub_gop_length_;
  const YuvPicture &orig_pic_;
  const EncoderSettings &encoder_settings_;
  const SampleMetric cu_metric_;    // TODO(PH) Get this from TransformEncocder
  const SampleMetric satd_metric_;
  CuWriter cu_writer_;
  ResidualBufferStorage bipred_orig_buffer_;
  SampleBufferStorage bipred_pred_buffer_;
  // Mapping of ref_idx from L1 to L0 when POC is same
  std::array<int, constants::kMaxNumRefPics> same_poc_in_l0_mapping_;
  // Best uni-prediction motion estimation result for a single CU
  std::array<std::array<MotionVector, constants::kMaxNumRefPics>,
    static_cast<int>(RefPicList::kTotalNumber)> unipred_best_mv_;
  std::array<std::array<MotionVector3, constants::kMaxNumRefPics>,
    static_cast<int>(RefPicList::kTotalNumber)> affine_best_mv_;
  std::array<std::array<int, constants::kMaxNumRefPics>,
    static_cast<int>(RefPicList::kTotalNumber)> unipred_best_mvp_idx_;
  std::array<std::array<Distortion, constants::kMaxNumRefPics>,
    static_cast<int>(RefPicList::kTotalNumber)> unipred_best_dist_;
  // Best fullpel search mv per ref list, ref idx and picture
  std::array<std::array<MvFullpel, constants::kMaxNumRefPics>,
    static_cast<int>(RefPicList::kTotalNumber)> previous_fullpel_;
  // Affine search buffers
  std::array<std::array<float, constants::kMaxBlockSize>,
    constants::kMaxBlockSize> affine_delta_hor_;
  std::array<std::array<float, constants::kMaxBlockSize>,
    constants::kMaxBlockSize> affine_delta_ver_;
  friend class TzSearch;
};

inline InterSearchFlags operator~ (InterSearchFlags a) {
  return static_cast<InterSearchFlags>(~static_cast<int>(a));
}
inline InterSearchFlags operator| (InterSearchFlags a, InterSearchFlags b) {
  return
    static_cast<InterSearchFlags>(static_cast<int>(a) | static_cast<int>(b));
}
inline InterSearchFlags operator& (InterSearchFlags a, InterSearchFlags b) {
  return
    static_cast<InterSearchFlags>(static_cast<int>(a) & static_cast<int>(b));
}
inline InterSearchFlags& operator|= (InterSearchFlags &a, InterSearchFlags b) {  // NOLINT
  a = static_cast<InterSearchFlags>(static_cast<int>(a) | static_cast<int>(b));
  return a;
}
inline InterSearchFlags& operator&= (InterSearchFlags &a, InterSearchFlags b) {  // NOLINT
  a = static_cast<InterSearchFlags>(static_cast<int>(a) & static_cast<int>(b));
  return a;
}

}   // namespace xvc

#endif  // XVC_ENC_LIB_INTER_SEARCH_H_
