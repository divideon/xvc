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

#ifndef XVC_ENC_LIB_INTER_TZ_SEARCH_H_
#define XVC_ENC_LIB_INTER_TZ_SEARCH_H_

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/inter_prediction.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_enc_lib/sample_metric.h"
#include "xvc_enc_lib/encoder_settings.h"

namespace xvc {

class TzSearch {
public:
  TzSearch(int bitdepth, const YuvPicture &orig_pic,
           const InterPrediction &inter_pred,
           const EncoderSettings &encoder_settings, int search_range)
    : orig_pic_(orig_pic),
    inter_pred_(inter_pred),
    encoder_settings_(encoder_settings),
    bitdepth_(bitdepth),
    search_range_(search_range) {
  }
  MvFullpel Search(const CodingUnit &cu, const Qp &qp, MetricType metric,
                   const MotionVector &mvp, const YuvPicture &ref_pic,
                   const MvFullpel &mv_min, const MvFullpel &mv_max,
                   const MvFullpel &prev_search);

private:
  using const_mv = const MvFullpel;
  struct Left { static const int index = -1; };
  struct Right { static const int index = 1; };
  struct Up { static const int index = -3; };
  struct Down { static const int index = 3; };
  struct SearchState;
  template<typename TOrig> class DistortionWrapper;

  bool FullpelDiamondSearch(SearchState *state, const MvFullpel &mv_base,
                            int range);
  void FullpelNeighborPointSearch(SearchState *state);
  Distortion GetCost(SearchState *state, int mv_x, int mv_y);
  bool CheckCostBest(SearchState *state, int mv_x, int mv_y);
  template<class Dir>
  bool CheckCost1(SearchState *state, int mv_x, int mv_y, int range);
  template<class Dir1, class Dir2>
  bool CheckCost2(SearchState *state, int mv_x, int mv_y, int range);
  template<class Dir>
  bool IsInside(int mv_x, int mv_y, const_mv *mv_min, const_mv *mv_max);

  const YuvPicture &orig_pic_;
  const InterPrediction &inter_pred_;
  const EncoderSettings &encoder_settings_;
  int bitdepth_;
  int search_range_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_INTER_TZ_SEARCH_H_
