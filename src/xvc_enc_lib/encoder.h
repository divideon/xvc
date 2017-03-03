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
#include "xvc_enc_lib/bit_writer.h"
#include "xvc_enc_lib/picture_encoder.h"
#include "xvc_enc_lib/xvcenc.h"

struct xvc_encoder {};

namespace xvc {

class Encoder : public xvc_encoder {
public:
  explicit Encoder(int internal_bitdepth)
    : internal_bitdepth_(internal_bitdepth) {
  }
  int Encode(const uint8_t *pic_bytes, xvc_enc_nal_unit **nal_units,
             bool output_rec, xvc_enc_pic_buffer *rec_pic);
  int Flush(xvc_enc_nal_unit **nal_units, bool output_rec,
            xvc_enc_pic_buffer *rec_pic);

  void SetResolution(int width, int height) {
    pic_width_ = width;
    pic_height_ = height;
  }
  void SetChromaFormat(xvc_enc_chroma_format format) {
    chroma_format_ = ChromaFormat(format);
  }
  void SetInputBitdepth(int bitdepth) { input_bitdepth_ = bitdepth; }
  void SetFramerate(double rate) { framerate_ = rate; }
  void SetSubGopLength(PicNum length) { sub_gop_length_ = length; }
  void SetPicBufferingNum(int num) { pic_buffering_num_ = num; }
  void SetSegmentLength(PicNum length) { segment_length_ = length; }
  void SetClosedGopInterval(PicNum interval) {
    closed_gop_interval_ = interval;
  }
  void SetQP(int qp) { base_qp_ = qp; }
  void SetDeblock(int deblock) { deblock_ = deblock; }
  void SetBetaOffset(int offset) { beta_offset_ = offset; }
  void SetTcOffset(int offset) { tc_offset_ = offset; }
  void SetAllIntra(bool all_intra) { all_intra_ = all_intra; }
  void SetFlatLambda(bool flat_lambda) { flat_lambda_ = flat_lambda; }

private:
  void EncodeOnePicture(std::shared_ptr<PictureEncoder> pic);
  void ReconstructOnePicture(bool output_rec,
                             xvc_enc_pic_buffer *rec_pic);
  void EncodeSegmentHeader();
  std::shared_ptr<PictureEncoder> GetNewPictureEncoder();

  void PrepareRefPicLists(std::shared_ptr<PictureEncoder> pic);
  int FillRefPicsLowerPoc(int start_idx,
                          std::shared_ptr<PictureEncoder> curr_pic,
                          RefPicList ref_pic_list);
  int FillRefPicsHigherPoc(int start_idx,
                           std::shared_ptr<PictureEncoder> curr_pic,
                           RefPicList ref_pic_list);

  void SetNalStats(xvc_enc_nal_unit *nal, std::shared_ptr<PictureEncoder> pic);

  int pic_width_ = 0;
  int pic_height_ = 0;
  int input_bitdepth_ = 8;
  int buffer_flag_ = 0;
  ChromaFormat chroma_format_ = ChromaFormat::k420;
  int internal_bitdepth_ = 0;
  double framerate_ = 0;
  int base_qp_ = -1;
  int deblock_ = 0;
  int beta_offset_ = 0;
  int tc_offset_ = 0;
  PicNum sub_gop_end_poc_ = 0;
  PicNum sub_gop_start_poc_ = 0;
  bool prev_segment_open_gop_ = false;
  bool curr_segment_open_gop_ = false;
  PicNum poc_ = 0;
  PicNum doc_ = 0;
  SegmentNum soc_ = std::numeric_limits<SegmentNum>::max();
  PicNum pic_buffering_num_ = 1;
  PicNum sub_gop_length_ = 1;
  PicNum segment_length_ = 1;
  PicNum closed_gop_interval_ = std::numeric_limits<PicNum>::max();
  bool all_intra_ = false;
  bool flat_lambda_ = false;
  std::vector<std::shared_ptr<PictureEncoder>> pic_encoders_;
  std::vector<uint8_t> output_pic_bytes_;
  BitWriter bit_writer_;
  std::vector<xvc_enc_nal_unit> nal_units_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_ENCODER_H_
