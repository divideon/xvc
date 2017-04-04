/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_RESAMPLE_H_
#define XVC_COMMON_LIB_RESAMPLE_H_

#include "xvc_common_lib/common.h"

namespace xvc {

namespace resample {

template <typename T, typename U>
void Resample(uint8_t *dst_start, int dst_width, int dst_height,
              ptrdiff_t dst_stride, int dst_bitdepth,
              const uint8_t *src_start, int src_width, int src_height,
              ptrdiff_t src_stride, int src_bitdepth);

}   // namespace resample

}   // namespace xvc

#endif  // XVC_COMMON_LIB_RESAMPLE_H_
