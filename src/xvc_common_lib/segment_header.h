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
  void SetWidth(int output_width) {
    output_pic_width_ = output_width;
    internal_pic_width_ = constants::kMinCuSize *
      ((output_width + constants::kMinCuSize - 1) / constants::kMinCuSize);
  }
  int GetOutputWidth() const { return output_pic_width_; }
  int GetInternalWidth() const { return internal_pic_width_; }
  void SetHeight(int output_height) {
    output_pic_height_ = output_height;
    internal_pic_height_ = constants::kMinCuSize *
      ((output_height + constants::kMinCuSize - 1) / constants::kMinCuSize);
  }
  int GetOutputHeight() const { return output_pic_height_; }
  int GetInternalHeight() const { return internal_pic_height_; }
  PictureFormat GetInternalPicFormat() const {
    return PictureFormat(internal_pic_width_, internal_pic_height_,
                         internal_bitdepth, chroma_format, color_matrix, false);
  }
  int GetCropWidth() const {
    return !source_padding ? 0 : internal_pic_width_ - output_pic_width_;
  }
  int GetCropHeight() const {
    return !source_padding ? 0 : internal_pic_height_ - output_pic_height_;
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
  bool low_delay = false;
  int leading_pictures = 0;
  int num_ref_pics = 0;
  int max_binary_split_depth = -1;
  Checksum::Mode checksum_mode = Checksum::Mode::kInvalid;
  bool source_padding = false;
  int adaptive_qp = -1;
  int chroma_qp_offset_table = -1;
  int chroma_qp_offset_u = 0;
  int chroma_qp_offset_v = 0;
  int deblock = -1;
  int beta_offset = 0;
  int tc_offset = 0;
  Restrictions restrictions;

private:
  int output_pic_width_ = 0;
  int output_pic_height_ = 0;
  int internal_pic_width_ = 0;
  int internal_pic_height_ = 0;
  static PicNum DocToPoc(PicNum sub_gop_length, PicNum doc);
  static PicNum PocToDoc(PicNum sub_gop_length, PicNum poc);
  static int DocToTid(PicNum sub_gop_length, PicNum doc);
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_SEGMENT_HEADER_H_
