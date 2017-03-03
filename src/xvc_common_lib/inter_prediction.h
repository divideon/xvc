/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_INTER_PREDICTION_H_
#define XVC_COMMON_LIB_INTER_PREDICTION_H_

#include <array>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/sample_buffer.h"

namespace xvc {

typedef std::array<MotionVector,
  constants::kNumInterMvPredictors> InterPredictorList;

struct MergeCandidate {
  InterDir inter_dir = InterDir::kL0;
  std::array<MotionVector, static_cast<int>(RefPicList::kTotalNumber)> mv;
  std::array<int, static_cast<int>(RefPicList::kTotalNumber)> ref_idx = { 0 };
};

struct InterMergeCandidateList
  : public std::array<MergeCandidate, constants::kNumInterMergeCandidates> {
  int num = 0;
};

class InterPrediction {
public:
  static const int kNumTapsLuma = 8;
  static const int kNumTapsChroma = 4;
  static const int kInternalPrecision = 14;
  static const int kFilterPrecision = 6;
  static const int kInternalOffset = 1 << (kInternalPrecision - 1);
  static const int kMergeLevelShift = 2;

  explicit InterPrediction(int bitdepth) : bitdepth_(bitdepth) {}

  InterPredictorList GetMvPredictors(const CodingUnit &cu, RefPicList ref_list,
                                     int ref_idx);
  InterMergeCandidateList GetMergeCandidates(const CodingUnit &cu,
                                             int merge_cand_idx = -1);
  void CalculateMV(CodingUnit *cu);
  void ApplyMerge(CodingUnit *cu, const InterMergeCandidateList &merge_list,
                  int merge_idx);
  void MotionCompensation(const CodingUnit &cu, YuvComponent comp,
                          Sample *pred_ptr, ptrdiff_t pred_stride);

protected:
  void MotionCompensationMv(const CodingUnit &cu, YuvComponent comp,
                            const YuvPicture &ref_pic, int mv_x, int mv_y,
                            Sample *pred, ptrdiff_t pred_stride);
  void ClipMV(const CodingUnit &cu, const YuvPicture &ref_pic,
              int *mv_x, int *mv_y);

private:
  static const int kBufSize = constants::kMaxBlockSize *
    (constants::kMaxBlockSize + kNumTapsLuma - 1);
  static const std::array<std::array<int16_t, kNumTapsLuma>, 4> kLumaFilter;
  static const std::array<std::array<int16_t, kNumTapsChroma>, 8> kChromaFilter;
  static const std::array<std::array<uint8_t, 2>, 12> kMergeCandL0L1Idx;

  bool GetMVPCand(const CodingUnit *cu, RefPicList ref_list, int ref_idx,
                  PicNum ref_poc, MotionVector *mv_out);
  bool GetScaledMVPCand(const CodingUnit *cu, RefPicList cu_ref_list,
                        int ref_idx, MotionVector *mv_out);
  void MotionCompensationBi(const CodingUnit &cu, YuvComponent comp,
                            const YuvPicture &ref_pic, const MotionVector &mv,
                            int16_t *pred, ptrdiff_t pred_stride);
  DataBuffer<const Sample>
    GetFullpelRef(const CodingUnit &cu, YuvComponent comp,
                  const YuvPicture &ref_pic, int mv_x, int mv_y,
                  int *frac_x, int *frac_y);
  void FilterLuma(int width, int height, int frac_x, int frac_y,
                  const Sample *ref, ptrdiff_t ref_stride,
                  Sample *pred, ptrdiff_t pred_stride);
  void FilterChroma(int width, int height, int frac_x, int frac_y,
                    const Sample *ref, ptrdiff_t ref_stride,
                    Sample *pred, ptrdiff_t pred_stride);
  void FilterCopyBipred(int width, int height,
                        const Sample *ref, ptrdiff_t ref_stride,
                        int16_t *pred, ptrdiff_t pred_stride);
  void FilterLumaBipred(int width, int height, int frac_x, int frac_y,
                        const Sample *ref, ptrdiff_t ref_stride,
                        int16_t *pred, ptrdiff_t pred_stride);
  void FilterChromaBipred(int width, int height, int frac_x, int frac_y,
                          const Sample *ref, ptrdiff_t ref_stride,
                          int16_t *pred, ptrdiff_t pred_stride);
  void AddAvgBi(const CodingUnit &cu, YuvComponent comp,
                const int16_t *src_l0, ptrdiff_t src_l0_stride,
                const int16_t *src_l1, ptrdiff_t src_l1_stride,
                Sample *pred, ptrdiff_t pred_stride);
  MergeCandidate GetMergeCandidateFromCu(const CodingUnit &cu);

  std::array<int16_t, kBufSize> filter_buffer_;
  std::array<std::array<int16_t,
    constants::kMaxBlockSize * constants::kMaxBlockSize>, 2> bipred_temp_;
  int bitdepth_;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_INTER_PREDICTION_H_
