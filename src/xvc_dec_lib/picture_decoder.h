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
                 int crop_width, int crop_height);
  void Init(const SegmentHeader &segment, const PicNalHeader &header,
            ReferencePictureLists &&ref_pic_list,
            const PictureFormat &output_pic_format, int64_t user_data);
  bool Decode(const SegmentHeader &segment,
              const SegmentHeader &prev_segment_header, BitReader *bit_reader,
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
    const PictureFormat &pic_fmt, int crop_width, int crop_height) const;
  static PicNalHeader
    DecodeHeader(const SegmentHeader &segment_header, BitReader *bit_reader,
                 PicNum *sub_gop_end_poc, PicNum *sub_gop_start_poc,
                 PicNum *sub_gop_length, PicNum prev_sub_gop_length,
                 PicNum doc, SegmentNum soc, int num_buffered_nals);

private:
  void GenerateAlternativeRecPic(const SegmentHeader &segment,
                               const SegmentHeader &prev_segment_header) const;
  bool ValidateChecksum(const SegmentHeader &segment,
                        BitReader *bit_reader, Checksum::Mode checksum_mode);

  const SimdFunctions &simd_;
  Resampler output_resampler_;
  PictureFormat output_format_;
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
