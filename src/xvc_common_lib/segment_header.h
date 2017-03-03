/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_SEGMENT_HEADER_H_
#define XVC_COMMON_LIB_SEGMENT_HEADER_H_

#include "xvc_common_lib/common.h"

namespace xvc {

struct SegmentHeader {
  int major_version = -1;
  int minor_version = -1;
  int pic_width = 0;
  int pic_height = 0;
  ChromaFormat chroma_format = ChromaFormat::kUndefinedChromaFormat;
  int internal_bitdepth = -1;
  int bitstream_ticks = 0;
  int base_qp = -1;
  PicNum max_sub_gop_length = 0;
  bool open_gop = false;
  int deblock = -1;
  int beta_offset = 0;
  int tc_offset = 0;
  int num_ref_pic_r0 = 0;
  int num_ref_pic_r1 = 0;
  SegmentNum soc = static_cast<SegmentNum>(-1);
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_SEGMENT_HEADER_H_
