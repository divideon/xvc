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

#ifndef XVC_ENC_LIB_ENCODER_H_
#define XVC_ENC_LIB_ENCODER_H_

#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/segment_header.h"
#include "xvc_enc_lib/bit_writer.h"
#include "xvc_enc_lib/picture_encoder.h"
#include "xvc_enc_lib/encoder_settings.h"
#include "xvc_enc_lib/encoder_simd_functions.h"

struct xvc_encoder {};

namespace xvc {

class Encoder : public xvc_encoder {
public:
  explicit Encoder(int internal_bitdepth);
  int Encode(const uint8_t *pic_bytes, xvc_enc_nal_unit **nal_units,
             xvc_enc_pic_buffer *rec_pic);
  int Flush(xvc_enc_nal_unit **nal_units, xvc_enc_pic_buffer *rec_pic);
  const SegmentHeader* GetCurrentSegment() const {
    return segment_header_.get();
  }

  void SetCpuCapabilities(std::set<CpuCapability> capabilities) {
    simd_ = EncoderSimdFunctions(capabilities,
                                 segment_header_->internal_bitdepth);
  }
  void SetResolution(int width, int height) {
    segment_header_->SetWidth(width);
    segment_header_->SetHeight(height);
  }
  void SetChromaFormat(ChromaFormat chroma_fmt) {
    segment_header_->chroma_format = chroma_fmt;
  }
  void SetColorMatrix(ColorMatrix color_matrix) {
    segment_header_->color_matrix = color_matrix;
  }
  int GetNumRefPics() const { return segment_header_->num_ref_pics; }
  void SetNumRefPics(int num) {
    segment_header_->num_ref_pics = num;
  }
  void SetInputBitdepth(int bitdepth) { input_bitdepth_ = bitdepth; }
  void SetFramerate(double rate) { framerate_ = rate; }
  void SetSubGopLength(PicNum sub_gop_length) {
    segment_header_->max_sub_gop_length = sub_gop_length;
    pic_buffering_num_ = sub_gop_length + segment_header_->num_ref_pics;
  }
  void SetSegmentLength(PicNum length) { segment_length_ = length; }
  void SetClosedGopInterval(PicNum interval) {
    assert(interval > 0);
    closed_gop_interval_ = interval;
  }
  void SetChromaQpOffsetTable(int table) {
    segment_header_->chroma_qp_offset_table = table;
  }
  void SetChromaQpOffsetU(int offset) {
    segment_header_->chroma_qp_offset_u = offset;
  }
  void SetChromaQpOffsetV(int offset) {
    segment_header_->chroma_qp_offset_v = offset;
  }
  void SetDeblock(int deblock) { segment_header_->deblock = deblock; }
  void SetBetaOffset(int offset) { segment_header_->beta_offset = offset; }
  void SetTcOffset(int offset) { segment_header_->tc_offset = offset; }
  void SetQp(int qp) {
    segment_qp_ =
      util::Clip3(qp, constants::kMinAllowedQp, constants::kMaxAllowedQp);
  }
  void SetFlatLambda(bool flat_lambda) { flat_lambda_ = flat_lambda; }
  void SetChecksumMode(Checksum::Mode mode) {
    segment_header_->checksum_mode = mode;
  }

  const EncoderSettings& GetEncoderSettings() { return encoder_settings_; }
  void SetEncoderSettings(const EncoderSettings &settings);

private:
  void EncodeOnePicture(std::shared_ptr<PictureEncoder> pic,
                        PicNum sub_gop_length);
  void ReconstructOnePicture(xvc_enc_pic_buffer *rec_pic);
  std::shared_ptr<PictureEncoder> GetNewPictureEncoder();

  void SetNalStats(const PictureData &pic_data, xvc_enc_nal_unit *nal);

  int input_bitdepth_ = 8;
  bool encode_with_buffer_flag_ = false;
  double framerate_ = 0;
  std::unique_ptr<SegmentHeader> segment_header_;
  std::unique_ptr<SegmentHeader> prev_segment_header_;
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
  EncoderSimdFunctions simd_;
  EncoderSettings encoder_settings_;
  std::vector<std::shared_ptr<PictureEncoder>> pic_encoders_;
  std::vector<uint8_t> output_pic_bytes_;
  BitWriter bit_writer_;
  std::vector<xvc_enc_nal_unit> nal_units_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_ENCODER_H_
