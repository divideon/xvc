/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_LIB_ENCODER_H_
#define XVC_ENC_LIB_ENCODER_H_

#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/segment_header.h"
#include "xvc_enc_lib/bit_writer.h"
#include "xvc_enc_lib/picture_encoder.h"
#include "xvc_enc_lib/encoder_settings.h"
#include "xvc_enc_lib/xvcenc.h"

struct xvc_encoder {};

namespace xvc {

class Encoder : public xvc_encoder {
public:
  Encoder();
  int Encode(const uint8_t *pic_bytes, xvc_enc_nal_unit **nal_units,
             bool output_rec, xvc_enc_pic_buffer *rec_pic);
  int Flush(xvc_enc_nal_unit **nal_units, bool output_rec,
            xvc_enc_pic_buffer *rec_pic);
  const SegmentHeader* GetCurrentSegment() const { return &segment_header_; }

  void SetResolution(int width, int height) {
    segment_header_.pic_width = width;
    segment_header_.pic_height = height;
  }
  void SetChromaFormat(xvc_enc_chroma_format format) {
    segment_header_.chroma_format = ChromaFormat(format);
  }
  void SetNumRefPics(int num) {
    segment_header_.num_ref_pics = num;
  }
  void SetInputBitdepth(int bitdepth) { input_bitdepth_ = bitdepth; }
  void SetInternalBitdepth(int bitdepth) {
    segment_header_.internal_bitdepth = bitdepth;
  }
  void SetFramerate(double rate) { framerate_ = rate; }
  void SetSubGopLength(PicNum sub_gop_length) {
    segment_header_.max_sub_gop_length = sub_gop_length;
  }
  void SetPicBufferingNum(int num) { pic_buffering_num_ = num; }
  void SetSegmentLength(PicNum length) { segment_length_ = length; }
  void SetClosedGopInterval(PicNum interval) {
    closed_gop_interval_ = interval;
  }
  void SetDeblock(int deblock) { segment_header_.deblock = deblock; }
  void SetBetaOffset(int offset) { segment_header_.beta_offset = offset; }
  void SetTcOffset(int offset) { segment_header_.tc_offset = offset; }
  void SetQp(int qp) { segment_qp_ = qp; }
  void SetFlatLambda(bool flat_lambda) { flat_lambda_ = flat_lambda; }
  void SetChecksumMode(int mode) {
    segment_header_.checksum_mode = Checksum::Mode(mode);
  }
  void SetRestrictedMode(int mode);

  void SetEncoderSettings(EncoderSettings &&s) { encoder_settings_ = s; }

private:
  void EncodeOnePicture(std::shared_ptr<PictureEncoder> pic,
                        PicNum sub_gop_length);
  void ReconstructOnePicture(bool output_rec,
                             xvc_enc_pic_buffer *rec_pic);
  std::shared_ptr<PictureEncoder> GetNewPictureEncoder();

  void SetNalStats(xvc_enc_nal_unit *nal, std::shared_ptr<PictureEncoder> pic);

  int input_bitdepth_ = 8;
  int buffer_flag_ = 0;
  double framerate_ = 0;
  SegmentHeader segment_header_;
  PicNum sub_gop_end_poc_ = 0;
  PicNum sub_gop_start_poc_ = 0;
  bool prev_segment_open_gop_ = false;
  bool curr_segment_open_gop_ = false;
  PicNum poc_ = 0;
  PicNum doc_ = 0;
  SegmentNum soc_ = std::numeric_limits<SegmentNum>::max();
  PicNum pic_buffering_num_ = 1;
  PicNum segment_length_ = 1;
  PicNum closed_gop_interval_ = std::numeric_limits<PicNum>::max();
  int segment_qp_ = std::numeric_limits<int>::max();
  bool flat_lambda_ = false;
  EncoderSettings encoder_settings_;
  std::vector<std::shared_ptr<PictureEncoder>> pic_encoders_;
  std::vector<uint8_t> output_pic_bytes_;
  BitWriter bit_writer_;
  std::vector<xvc_enc_nal_unit> nal_units_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_ENCODER_H_
