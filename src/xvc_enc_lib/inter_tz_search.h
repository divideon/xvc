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
  TzSearch(const YuvPicture &orig_pic, const InterPrediction &inter_pred,
           const EncoderSettings &encoder_settings, int search_range)
    : orig_pic_(orig_pic),
    inter_pred_(inter_pred),
    encoder_settings_(encoder_settings),
    search_range_(search_range) {
  }
  MvFullpel Search(const CodingUnit &cu, const Qp &qp,
                   const SampleMetric &metric, const MotionVector &mvp,
                   const YuvPicture &ref_pic, const MvFullpel &mv_min,
                   const MvFullpel &mv_max, const MvFullpel &prev_search);

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
  int search_range_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_INTER_TZ_SEARCH_H_
