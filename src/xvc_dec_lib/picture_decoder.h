/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_DEC_LIB_PICTURE_DECODER_H_
#define XVC_DEC_LIB_PICTURE_DECODER_H_

#include <memory>
#include <string>
#include <vector>

#include "xvc_common_lib/checksum.h"
#include "xvc_common_lib/common.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/segment_header.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_dec_lib/bit_reader.h"
#include "xvc_dec_lib/syntax_reader.h"
#include "xvc_dec_lib/xvcdec.h"

namespace xvc {

class PictureDecoder {
public:
  explicit PictureDecoder(ChromaFormat chroma_format, int width,
                          int height, int bitdepth);
  std::shared_ptr<const PictureData> GetPicData() const { return pic_data_; }
  std::shared_ptr<PictureData> GetPicData() { return pic_data_; }
  std::shared_ptr<const YuvPicture> GetRecPic() const { return rec_pic_; }
  std::shared_ptr<YuvPicture> GetRecPic() { return rec_pic_; }

  void DecodeHeader(BitReader *bit_reader, PicNum *sub_gop_end_poc,
                    PicNum *sub_gop_start_poc, PicNum *sub_gop_length,
                    PicNum max_sub_gop_length, PicNum prev_sub_gop_length,
                    PicNum doc, SegmentNum soc, int num_buffered_nals);
  bool Decode(BitReader *bit_reader, Checksum::Mode checksum_mode);
  std::vector<uint8_t> GetLastChecksum() const { return checksum_.GetHash(); }
  std::shared_ptr<YuvPicture> GetAlternativeRecPic(
    ChromaFormat chroma_format, int width, int height, int bitdepth) const;

private:
  bool ValidateChecksum(BitReader *bit_reader, Checksum::Mode checksum_mode);

  std::shared_ptr<PictureData> pic_data_;
  std::shared_ptr<YuvPicture> rec_pic_;
  std::shared_ptr<YuvPicture> alt_rec_pic_;
  Checksum checksum_;
  int first_peek_;
  int pic_qp_ = -1;
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_PICTURE_DECODER_H_
