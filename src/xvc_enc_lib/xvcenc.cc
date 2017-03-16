/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/xvcenc.h"

#include <algorithm>
#include <cstring>
#include <limits>

#include "xvc_common_lib/common.h"
#include "xvc_enc_lib/encoder.h"

#ifdef __cplusplus
extern "C" {
#endif
  xvc_encoder_parameters* xvc_enc_parameters_create() {
    xvc_encoder_parameters *parameters = new xvc_encoder_parameters;
    std::memset(parameters, 0, sizeof(xvc_encoder_parameters));
    return parameters;
  }

  xvc_enc_return_code xvc_enc_parameters_destroy(
    xvc_encoder_parameters *param) {
    if (param) {
      delete param;
    }
    return XVC_ENC_OK;
  }

  xvc_enc_return_code xvc_enc_parameters_set_default(
    xvc_encoder_parameters *param) {
    if (!param) {
      return XVC_ENC_INVALID_ARGUMENT;
    }
    param->chroma_format = XVC_ENC_CHROMA_FORMAT_420;
    param->input_bitdepth = 8;
    param->internal_bitdepth = sizeof(xvc::Sample) > 1 ? 10 : 8;
    param->framerate = 60;
    param->sub_gop_length = 0;
    param->max_keypic_distance = 640;
    param->closed_gop = 0;
    param->qp = 32;
    param->deblock = 1;
    param->beta_offset = 0;
    param->tc_offset = 0;
    param->all_intra = 0;
    param->flat_lambda = 0;
    return XVC_ENC_OK;
  }

  xvc_enc_return_code xvc_enc_parameters_check(xvc_encoder_parameters
                                               *param) {
    if (!param) {
      return XVC_ENC_INVALID_ARGUMENT;
    }
    if (param->width <= 0 ||
        param->height <= 0) {
      return XVC_ENC_SIZE_NOT_POSITIVE;
    }
    if (param->width % xvc::constants::kMinBlockSize ||
        param->height % xvc::constants::kMinBlockSize) {
      return XVC_ENC_SIZE_NOT_MULTIPLE_OF_BLOCKSIZE;
    }
    if (param->chroma_format < XVC_ENC_CHROMA_FORMAT_MONOCHROME ||
        param->chroma_format == XVC_ENC_CHROMA_FORMAT_422 ||
        param->chroma_format > XVC_ENC_CHROMA_FORMAT_444) {
      return XVC_ENC_UNSUPPORTED_CHROMA_FORMAT;
    }
    if (param->internal_bitdepth > 16 || param->internal_bitdepth < 8 ||
        param->input_bitdepth > 16 || param->input_bitdepth < 8) {
      return XVC_ENC_BITDEPTH_OUT_OF_RANGE;
    }
    if (param->internal_bitdepth > (8 * sizeof(xvc::Sample))) {
      return XVC_ENC_COMPILED_BITDEPTH_TOO_LOW;
    }
    if (param->framerate < (1.0 * xvc::constants::kTimeScale /
      (1 << xvc::constants::kFrameRateBitDepth)) ||
        param->framerate > xvc::constants::kTimeScale) {
      return XVC_ENC_FRAMERATE_OUT_OF_RANGE;
    }
    if (param->qp > xvc::constants::kMaxAllowedQp ||
        param->qp < xvc::constants::kMinAllowedQp) {
      return XVC_ENC_QP_OUT_OF_RANGE;
    }
    if (param->sub_gop_length > xvc::constants::kMaxSubGopLength) {
      return XVC_ENC_SUB_GOP_LENGTH_TOO_LARGE;
    }
    if (param->max_keypic_distance &&
        param->sub_gop_length > param->max_keypic_distance) {
      return XVC_ENC_SUB_GOP_LENGTH_TOO_LARGE;
    }
    if (param->deblock == 0 &&
      (param->beta_offset || param->tc_offset)) {
      return XVC_ENC_DEBLOCKING_SETTINGS_INVALID;
    }
    int d = xvc::constants::kDeblockOffsetBits - 1;
    if ((param->beta_offset >= (1 << d)) ||
      (param->beta_offset < -(1 << d)) ||
         (param->tc_offset >= (1 << d)) ||
         (param->tc_offset < -(1 << d))) {
      return XVC_ENC_DEBLOCKING_SETTINGS_INVALID;
    }
    if (param->closed_gop < 0) {
      return XVC_ENC_INVALID_PARAMETER;
    }
    if (param->closed_gop < 0) {
      return XVC_ENC_INVALID_PARAMETER;
    }
    if (param->all_intra < 0 || param->all_intra > 1) {
      return XVC_ENC_INVALID_ARGUMENT;
    }
    if (param->flat_lambda < 0 || param->flat_lambda > 1) {
      return XVC_ENC_INVALID_ARGUMENT;
    }
    return XVC_ENC_OK;
  }

  xvc_encoder* xvc_enc_encoder_create(xvc_encoder_parameters *param) {
    if (xvc_enc_parameters_check(param) != XVC_ENC_OK) {
      return nullptr;
    }
    if (param->sub_gop_length == 0) {
      param->sub_gop_length = !param->all_intra ? 16 : 1;
    }
    xvc::Encoder *encoder =
      new xvc::Encoder(param->internal_bitdepth, param->qp);
    encoder->SetResolution(param->width, param->height);
    encoder->SetChromaFormat(param->chroma_format);
    encoder->SetInputBitdepth(param->input_bitdepth);
    encoder->SetFramerate(param->framerate);
    encoder->SetSubGopLength(param->sub_gop_length);
    encoder->SetPicBufferingNum(param->sub_gop_length +
                                xvc::constants::kMaxNumLongTermPics);
    encoder->SetBetaOffset(param->beta_offset);
    encoder->SetTcOffset(param->tc_offset);
    if (param->beta_offset != 0 || param->tc_offset != 0) {
      encoder->SetDeblock(3);
    } else {
      encoder->SetDeblock(param->deblock);
    }
    encoder->SetAllIntra(param->all_intra != 0);
    encoder->SetFlatLambda(param->flat_lambda != 0);

    // Calculate what intra-period i.e. Gop-length i.e. segment-length to use.
    xvc::PicNum segment_length = 1;
    if (param->max_keypic_distance == 0) {
      segment_length = ((std::numeric_limits<xvc::PicNum>::max() /
                         param->sub_gop_length) * param->sub_gop_length);
    } else {
      segment_length =
        (param->max_keypic_distance / param->sub_gop_length) *
        param->sub_gop_length;
    }
    encoder->SetSegmentLength(segment_length);
    if (param->closed_gop > 0) {
      encoder->SetClosedGopInterval(segment_length*param->closed_gop);
    } else {
      encoder->SetClosedGopInterval(((std::numeric_limits<xvc::PicNum>::max() /
                                      param->sub_gop_length) *
                                     param->sub_gop_length));
    }
    return encoder;
  }

  xvc_enc_return_code xvc_enc_encoder_destroy(xvc_encoder *encoder) {
    if (encoder) {
      xvc::Encoder *lib_encoder = reinterpret_cast<xvc::Encoder*>(encoder);
      delete lib_encoder;
    }
    return XVC_ENC_OK;
  }

  xvc_enc_return_code xvc_enc_encoder_encode(xvc_encoder *encoder,
                                             uint8_t *picture_to_encode,
                                             xvc_enc_nal_unit **nal_units,
                                             int *num_nal_units,
                                             xvc_enc_pic_buffer *rec_pic) {
    if (!encoder || !picture_to_encode || !nal_units || !num_nal_units) {
      return XVC_ENC_INVALID_ARGUMENT;
    }
    xvc::Encoder *lib_encoder = reinterpret_cast<xvc::Encoder*>(encoder);
    xvc_enc_pic_buffer rec_pic_buffer;
    *num_nal_units = lib_encoder->Encode(picture_to_encode, nal_units,
      (rec_pic != nullptr), &rec_pic_buffer);
    if (rec_pic) {
      *rec_pic = rec_pic_buffer;
    }
    return XVC_ENC_OK;
  }

  xvc_enc_return_code xvc_enc_encoder_flush(xvc_encoder *encoder,
                                            xvc_enc_nal_unit **nal_units,
                                            int *num_nal_units,
                                            xvc_enc_pic_buffer *rec_pic) {
    if (!encoder || !nal_units || !num_nal_units) {
      return XVC_ENC_INVALID_ARGUMENT;
    }
    xvc::Encoder *lib_encoder = reinterpret_cast<xvc::Encoder*>(encoder);
    xvc_enc_pic_buffer rec_pic_buffer;
    *num_nal_units = lib_encoder->Flush(nal_units, (rec_pic != nullptr),
                                        &rec_pic_buffer);
    if (rec_pic) {
      *rec_pic = rec_pic_buffer;
    }
    return XVC_ENC_OK;
  }

  const char* xvc_enc_get_error_text(xvc_enc_return_code error_code) {
    switch (error_code) {
      case XVC_ENC_OK:
        return "No Error";
      case XVC_ENC_INVALID_ARGUMENT:
        return "Error. One or more invalid arguments provided to an xvc api"
          " function.";
      case XVC_ENC_SIZE_NOT_POSITIVE:
        return "Error. The input width and height must be positive.";
      case XVC_ENC_SIZE_NOT_MULTIPLE_OF_BLOCKSIZE:
        return "Error. The input resolution is not a multiple of 16";
      case XVC_ENC_UNSUPPORTED_CHROMA_FORMAT:
        return "Error. Unsupported chroma format.";
      case XVC_ENC_BITDEPTH_OUT_OF_RANGE:
        return  "Error. Bitdepth must be between 8 and 16, inclusive.";
      case XVC_ENC_COMPILED_BITDEPTH_TOO_LOW:
        return  "Error. This version of the xvc library is compiled for a"
          " lower bitdepth than what is requested. Please recompile the xvc"
          " library with higher bitdepth.";
      case XVC_ENC_FRAMERATE_OUT_OF_RANGE:
        return  "Error. Framerate must be between 0.0054 fps and 90,000 fps,"
          " inclusive.";
      case XVC_ENC_QP_OUT_OF_RANGE:
        return  "Error. QP must be between -64 and 63, inclusive.";
      case XVC_ENC_SUB_GOP_LENGTH_TOO_LARGE:
        return  "Error. Sub Gop length too large.";
      case XVC_ENC_DEBLOCKING_SETTINGS_INVALID:
        return  "Error. Beta offset and tc offset must be in the range from"
          " -32 to 31, inclusive and deblock must be set to 1 when beta offset"
          " and/or tc offset is used.";
      case XVC_ENC_INVALID_PARAMETER:
        return  "Error. Invalid parameter. Please check the encoder"
          " parameters.";
      default:
        return "Error in xvc library or xvc api.";
    }
  }

  static const xvc_encoder_api xvc_enc_api_internal = {
    &xvc_enc_parameters_create,
    &xvc_enc_parameters_destroy,
    &xvc_enc_parameters_set_default,
    &xvc_enc_parameters_check,
    &xvc_enc_encoder_create,
    &xvc_enc_encoder_destroy,
    &xvc_enc_encoder_encode,
    &xvc_enc_encoder_flush,
    &xvc_enc_get_error_text,
  };

  const xvc_encoder_api* xvc_encoder_api_get() {
    return &xvc_enc_api_internal;
  }

}  // extern C
