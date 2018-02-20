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
  void SetOutputStatus(OutputStatus status) { output_status_ = status; }
  OutputStatus GetOutputStatus() const { return output_status_; }
  uint64_t GetRecPicErrSum() const { return rec_sse_; }

  std::vector<uint8_t>* Encode(const SegmentHeader &segment, int segment_qp,
                               int buffer_flag,
                               const EncoderSettings &encoder_settings);
  const std::vector<uint8_t>& GetLastChecksum() const { return pic_hash_; }
  std::shared_ptr<YuvPicture> GetAlternativeRecPic(
    const PictureFormat &pic_fmt, int crop_width, int crop_height) const;

private:
  void WriteHeader(const PictureData &pic_data, PicNum sub_gop_length,
                   int buffer_flag, BitWriter *bit_writer);
  void WriteChecksum(const SegmentHeader &segment, BitWriter *bit_writer,
                     Checksum::Mode checksum_mode);
  int DerivePictureQp(const EncoderSettings &encoder_settings, int segment_qp,
                      PicturePredictionType pic_type, int tid) const;
  bool DetermineAllowLic(PicturePredictionType pic_type,
                         const ReferencePictureLists &ref_list) const;
  uint64_t CalculatePicMetric(const Qp &qp) const;
  static double CalculateLambda(int qp, PicturePredictionType pic_type,
                                int sub_gop_length, int temporal_id,
                                int max_temporal_id,
                                const EncoderSettings &encoder_settings);
  static int GetQpFromLambda(int bitdepth, double lambda);

  const EncoderSimdFunctions &simd_;
  BitWriter bit_writer_;
  std::shared_ptr<YuvPicture> orig_pic_;
  std::shared_ptr<PictureData> pic_data_;
  std::shared_ptr<YuvPicture> rec_pic_;
  std::vector<uint8_t> pic_hash_;
  uint64_t rec_sse_ = 0;
  OutputStatus output_status_ = OutputStatus::kHasNotBeenOutput;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_PICTURE_ENCODER_H_
