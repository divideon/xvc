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

#ifndef XVC_DEC_LIB_PICTURE_DECODER_H_
#define XVC_DEC_LIB_PICTURE_DECODER_H_

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include "xvc_common_lib/checksum.h"
#include "xvc_common_lib/common.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/resample.h"
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
    bool allow_lic;
  };

  PictureDecoder(const SimdFunctions &simd, const PictureFormat &pic_format,
                 const PictureFormat &output_format);
  void Init(const SegmentHeader &segment, const PicNalHeader &header,
            ReferencePictureLists &&ref_pic_list, int64_t user_data);
  bool Decode(const SegmentHeader &segment, BitReader *bit_reader,
              bool post_process);
  bool Postprocess(const SegmentHeader &segment, BitReader *bit_reader);
  std::shared_ptr<const YuvPicture> GetOrigPic() const { return nullptr; }
  std::shared_ptr<const PictureData> GetPicData() const { return pic_data_; }
  std::shared_ptr<PictureData> GetPicData() { return pic_data_; }
  std::shared_ptr<const YuvPicture> GetRecPic() const { return rec_pic_; }
  std::shared_ptr<YuvPicture> GetRecPic() { return rec_pic_; }
  int64_t GetNalUserData() const { return user_data_; }
  void SetOutputStatus(OutputStatus status) {
    output_status_.store(status, std::memory_order_release);
  }
  OutputStatus GetOutputStatus() const {
    return output_status_.load(std::memory_order_acquire);
  }
  const std::vector<uint8_t>& GetOutputPictureBytes() const {
    return output_pic_bytes_;
  }
  void SetIsConforming(bool conforming) { conforming_ = conforming; }
  bool GetIsConforming() const { return conforming_; }
  bool IsReferenced() const { return ref_count > 0; }
  void AddReferenceCount(int val) const { ref_count += val; }
  void RemoveReferenceCount(int val) const { ref_count -= val; }
  const std::vector<uint8_t>& GetLastChecksum() const { return pic_hash_; }
  std::shared_ptr<YuvPicture> GetAlternativeRecPic(
    ChromaFormat chroma_format, int width, int height, int bitdepth) const;
  static PicNalHeader
    DecodeHeader(BitReader *bit_reader, PicNum *sub_gop_end_poc,
                 PicNum *sub_gop_start_poc, PicNum *sub_gop_length,
                 PicNum max_sub_gop_length, PicNum prev_sub_gop_length,
                 PicNum doc, SegmentNum soc, int num_buffered_nals);

private:
  bool ValidateChecksum(const SegmentHeader &segment,
                        BitReader *bit_reader, Checksum::Mode checksum_mode);

  const SimdFunctions &simd_;
  Resampler output_resampler_;
  std::shared_ptr<PictureData> pic_data_;
  std::shared_ptr<YuvPicture> rec_pic_;
  std::shared_ptr<YuvPicture> alt_rec_pic_;
  std::vector<uint8_t> pic_hash_;
  std::vector<uint8_t> output_pic_bytes_;
  bool conforming_ = false;
  int pic_qp_ = -1;
  int64_t user_data_ = 0;
  std::atomic<OutputStatus> output_status_ = { OutputStatus::kHasBeenOutput };
  // TODO(PH) Mutable isn't really needed if const handling is relaxed...
  // Note that ref_count should only be modified on "main thread"
  mutable int ref_count = 0;
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_PICTURE_DECODER_H_
