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

#ifndef XVC_ENC_LIB_PICTURE_ENCODER_H_
#define XVC_ENC_LIB_PICTURE_ENCODER_H_

#include <memory>
#include <string>
#include <vector>

#include "xvc_common_lib/checksum.h"
#include "xvc_common_lib/common.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/segment_header.h"
#include "xvc_common_lib/yuv_pic.h"
#include "xvc_enc_lib/bit_writer.h"
#include "xvc_enc_lib/encoder_settings.h"
#include "xvc_enc_lib/encoder_simd_functions.h"
#include "xvc_enc_lib/syntax_writer.h"
#include "xvc_enc_lib/xvcenc.h"

namespace xvc {

class PictureEncoder {
public:
  PictureEncoder(const EncoderSimdFunctions &simd,
                 const PictureFormat &pic_fmt,
                 int crop_width, int crop_height);
  std::shared_ptr<const YuvPicture> GetOrigPic() const { return orig_pic_; }
  std::shared_ptr<YuvPicture> GetOrigPic() { return orig_pic_; }
  std::shared_ptr<const PictureData> GetPicData() const { return pic_data_; }
  std::shared_ptr<PictureData> GetPicData() { return pic_data_; }
  std::shared_ptr<const YuvPicture> GetRecPic() const { return rec_pic_; }
  std::shared_ptr<YuvPicture> GetRecPic() { return rec_pic_; }
  PicNum GetPoc() const { return pic_data_->GetPoc(); }
  PicNum GetDoc() const { return pic_data_->GetDoc(); }
  int GetTid() const { return pic_data_->GetTid(); }
  void SetOutputStatus(OutputStatus status) { output_status_ = status; }
  OutputStatus GetOutputStatus() const { return output_status_; }
  bool GetBufferFlag() const { return buffer_flag_; }
  void SetBufferFlag(bool buffer_flag) { buffer_flag_ = buffer_flag; }
  bool IsReferenced() const { return ref_count_ > 0; }
  int GetReferenceCount() const { return ref_count_; }
  void SetReferenceCount(int ref_count) { ref_count_ = ref_count; }
  void RemoveReferenceCount(int val) const { ref_count_ -= val; }
  uint64_t GetRecPicErrSum() const { return rec_sse_; }
  double GetRecPicPsnr(YuvComponent c) const {
    switch (c) {
      case YuvComponent::kY:
        return rec_psnr_y_;
        break;
      case YuvComponent::kU:
        return rec_psnr_u_;
        break;
      case YuvComponent::kV:
        return rec_psnr_v_;
        break;
      default:
        assert(0);
        return 0;
        break;
    }
  }
  void SetUserData(int64_t user_data) { user_data_ = user_data; }
  int64_t GetUserData() const { return user_data_; }

  void Init(const SegmentHeader &segment, PicNum doc, PicNum poc, int tid,
            bool is_access_picture);
  const std::vector<uint8_t>*
    Encode(const SegmentHeader &segment, int segment_qp, int buffer_flag,
           const EncoderSettings &encoder_settings);
  const std::vector<uint8_t>& GetLastChecksum() const { return pic_hash_; }
  std::shared_ptr<YuvPicture> GetAlternativeRecPic(
    const PictureFormat &pic_fmt, int crop_width, int crop_height) const;

private:
  void WriteHeader(const SegmentHeader &segment, const PictureData &pic_data,
                   PicNum sub_gop_length, int buffer_flag,
                   BitWriter *bit_writer);
  void WriteChecksum(const SegmentHeader &segment, BitWriter *bit_writer,
                     Checksum::Mode checksum_mode);
  int DerivePictureQp(const EncoderSettings &encoder_settings, int segment_qp,
                      PicturePredictionType pic_type, int tid) const;
  bool DetermineAllowLic(PicturePredictionType pic_type,
                         const ReferencePictureLists &ref_list) const;
  uint64_t CalculatePicMetric(const Qp &qp) const;
  double CalculatePsnr(const Qp & qp, YuvComponent c) const;
  static double CalculateLambda(const EncoderSettings &encoder_settings,
                                const SegmentHeader &segment_header,
                                int pic_qp, PicturePredictionType pic_type,
                                int sub_gop_length, int temporal_id,
                                int max_temporal_id);
  static int GetQpFromLambda(int bitdepth, double lambda);

  const EncoderSimdFunctions &simd_;
  BitWriter bit_writer_;
  std::shared_ptr<YuvPicture> orig_pic_;
  std::shared_ptr<PictureData> pic_data_;
  std::shared_ptr<YuvPicture> rec_pic_;
  std::vector<uint8_t> pic_hash_;
  uint64_t rec_sse_ = 0;
  double rec_psnr_y_ = 0;
  double rec_psnr_u_ = 0;
  double rec_psnr_v_ = 0;
  int64_t user_data_ = 0;
  OutputStatus output_status_ = OutputStatus::kHasBeenOutput;
  bool buffer_flag_ = false;
  mutable int ref_count_ = 0;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_PICTURE_ENCODER_H_
