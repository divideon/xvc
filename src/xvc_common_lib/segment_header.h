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
  static PicNum CalcDocFromPoc(PicNum poc, PicNum sub_gop_length,
                               PicNum sub_gop_start_poc_);
  static PicNum CalcPocFromDoc(PicNum doc, PicNum sub_gop_length,
                               PicNum sub_gop_start_poc_);
  static int CalcTidFromDoc(PicNum doc, PicNum sub_gop_length,
                            PicNum sub_gop_start_poc_);
  static int GetMaxTid(PicNum sub_gop_length);
  static int GetFramerateMaxTid(int decoder_ticks, int bitstream_ticks,
                                PicNum sub_gop_length);
  static double GetFramerate(int max_tid, int bitstream_ticks,
                             PicNum sub_gop_length);

  uint32_t codec_identifier = static_cast<uint32_t>(-1);
  uint32_t major_version = static_cast<uint32_t>(-1);
  uint32_t minor_version = static_cast<uint32_t>(-1);
  SegmentNum soc = static_cast<SegmentNum>(-1);
  int pic_width = 0;
  int pic_height = 0;
  ChromaFormat chroma_format = ChromaFormat::kUndefinedChromaFormat;
  int internal_bitdepth = -1;
  int bitstream_ticks = 0;
  PicNum max_sub_gop_length = 0;
  bool open_gop = false;
  int num_ref_pics = 0;
  int deblock = -1;
  int beta_offset = 0;
  int tc_offset = 0;

private:
  static PicNum DocToPoc(PicNum sub_gop_length, PicNum doc);
  static PicNum PocToDoc(PicNum sub_gop_length, PicNum poc);
  static int DocToTid(PicNum sub_gop_length, PicNum doc);
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_SEGMENT_HEADER_H_
