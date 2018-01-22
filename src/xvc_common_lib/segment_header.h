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

#ifndef XVC_COMMON_LIB_SEGMENT_HEADER_H_
#define XVC_COMMON_LIB_SEGMENT_HEADER_H_

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/checksum.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/yuv_pic.h"

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
  void SetWidth(int width) { pic_width = width; }
  int GetOutputWidth() const { return pic_width; }
  int GetInternalWidth() const {
    return pic_width - (pic_width % constants::kMinCuSize);
  }
  void SetHeight(int height) { pic_height = height; }
  int GetOutputHeight() const { return pic_height; }
  int GetInternalHeight() const {
    return pic_height - (pic_height % constants::kMinCuSize);
  }
  PictureFormat GetInternalPicFormat() const {
    return PictureFormat(GetInternalWidth(), GetInternalHeight(),
                         internal_bitdepth, chroma_format, color_matrix, false);
  }

  uint32_t codec_identifier = static_cast<uint32_t>(-1);
  uint32_t major_version = static_cast<uint32_t>(-1);
  uint32_t minor_version = static_cast<uint32_t>(-1);
  SegmentNum soc = static_cast<SegmentNum>(-1);
  ChromaFormat chroma_format = ChromaFormat::kUndefined;
  ColorMatrix color_matrix = ColorMatrix::kUndefined;
  int internal_bitdepth = -1;
  int bitstream_ticks = 0;
  PicNum max_sub_gop_length = 0;
  bool open_gop = false;
  int num_ref_pics = 0;
  int max_binary_split_depth = -1;
  Checksum::Mode checksum_mode = Checksum::Mode::kInvalid;
  int adaptive_qp = -1;
  int chroma_qp_offset_table = -1;
  int chroma_qp_offset_u = 0;
  int chroma_qp_offset_v = 0;
  int deblock = -1;
  int beta_offset = 0;
  int tc_offset = 0;
  Restrictions restrictions;

private:
  int pic_width = 0;
  int pic_height = 0;
  static PicNum DocToPoc(PicNum sub_gop_length, PicNum doc);
  static PicNum PocToDoc(PicNum sub_gop_length, PicNum poc);
  static int DocToTid(PicNum sub_gop_length, PicNum doc);
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_SEGMENT_HEADER_H_
