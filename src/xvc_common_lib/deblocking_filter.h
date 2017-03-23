/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_DEBLOCKING_FILTER_H_
#define XVC_COMMON_LIB_DEBLOCKING_FILTER_H_

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/yuv_pic.h"

namespace xvc {

enum class Direction {
  kVertical,
  kHorizontal,
};

class DeblockingFilter {
public:
  explicit DeblockingFilter(PictureData *pic_data, YuvPicture *rec_pic,
                            int beta_offset, int tc_offset) :
    pic_data_(pic_data),
    rec_pic_(rec_pic),
    beta_offset_(beta_offset),
    tc_offset_(tc_offset) {
  }
  void DeblockPicture();

private:
  void DeblockCtu(int rsaddr, CuTree cu_tree, Direction dir);

  int GetBoundaryStrength(const CodingUnit &cu_p, const CodingUnit &cu_q);

  void FilterEdgeLuma(int x, int y, Direction dir,
                      int boundary_strength, int qp);
  bool CheckStrongFilter(Sample *src, int beta, int tc, ptrdiff_t offset);
  void FilterLumaWeak(Sample *src_ptr, ptrdiff_t step_size,
                      ptrdiff_t offset, int tc, bool filter_p1,
                      bool filter_q1);
  void FilterLumaStrong(Sample *src_ptr, ptrdiff_t step_size,
                        ptrdiff_t offset, int tc);
  void FilterEdgeChroma(int x, int y, int scale_x, int scale_y, Direction dir,
                        int boundary_strength, int qp);
  void FilterChroma(Sample *src_ptr, ptrdiff_t step_size,
                    ptrdiff_t offset, int tc2);

  PictureData *pic_data_;
  YuvPicture *rec_pic_;
  int beta_offset_ = 0;
  int tc_offset_ = 0;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_DEBLOCKING_FILTER_H_
