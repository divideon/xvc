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
#include "xvc_common_lib/simd_functions.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_dec_lib/bit_reader.h"
#include "xvc_dec_lib/syntax_reader.h"
#include "xvc_dec_lib/xvcdec.h"

namespace xvc {

class PictureDecoder {
public:
  struct PicNalHeader {
    NalUnitType nal_unit_type;
    SegmentNum soc;
    PicNum poc;
    PicNum doc;
    int tid;
    int pic_qp;
    bool highest_layer;
  };

  PictureDecoder(const SimdFunctions &simd, ChromaFormat chroma_format,
                 int width, int height, int bitdepth);
  void Init(const SegmentHeader &segment, const PicNalHeader &header,
            ReferencePictureLists &&ref_pic_list, int64_t user_data);
  bool Decode(const SegmentHeader &segment, BitReader *bit_reader);
  std::shared_ptr<const PictureData> GetPicData() const { return pic_data_; }
  std::shared_ptr<PictureData> GetPicData() { return pic_data_; }
  std::shared_ptr<const YuvPicture> GetRecPic() const { return rec_pic_; }
  std::shared_ptr<YuvPicture> GetRecPic() { return rec_pic_; }
  int64_t GetNalUserData() const { return user_data_; }
  void SetOutputStatus(OutputStatus status) { output_status_ = status; }
  OutputStatus GetOutputStatus() const { return output_status_; }
  bool IsReferenced() const { return ref_count > 0; }
  void AddReferenceCount(int val) const { ref_count += val; }
  void RemoveReferenceCount(int val) const { ref_count -= val; }
  std::vector<uint8_t> GetLastChecksum() const { return checksum_.GetHash(); }
  std::shared_ptr<YuvPicture> GetAlternativeRecPic(
    ChromaFormat chroma_format, int width, int height, int bitdepth) const;
  static PicNalHeader
    DecodeHeader(BitReader *bit_reader, PicNum *sub_gop_end_poc,
                 PicNum *sub_gop_start_poc, PicNum *sub_gop_length,
                 PicNum max_sub_gop_length, PicNum prev_sub_gop_length,
                 PicNum doc, SegmentNum soc, int num_buffered_nals);

private:
  bool ValidateChecksum(BitReader *bit_reader, Checksum::Mode checksum_mode);

  const SimdFunctions &simd_;
  std::shared_ptr<PictureData> pic_data_;
  std::shared_ptr<YuvPicture> rec_pic_;
  std::shared_ptr<YuvPicture> alt_rec_pic_;
  Checksum checksum_;
  int pic_qp_ = -1;
  int64_t user_data_ = 0;
  // TODO(PH) Consider using memory barrier and relax global mutex requirement
  OutputStatus output_status_ = OutputStatus::kHasBeenOutput;
  // TODO(PH) Mutable isn't really needed if const handling is relaxed...
  // Note that ref_count should only be modified on "main thread"
  mutable int ref_count = 0;
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_PICTURE_DECODER_H_
