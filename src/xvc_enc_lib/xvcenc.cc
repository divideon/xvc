/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/xvcenc.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>

#include "xvc_common_lib/common.h"
#include "xvc_enc_lib/encoder.h"
#include "xvc_enc_lib/encoder_settings.h"

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
    param->color_matrix = XVC_ENC_COLOR_MATRIX_UNDEFINED;
    param->input_bitdepth = 8;
    param->internal_bitdepth = sizeof(xvc::Sample) > 1 ? 10 : 8;
    param->framerate = 60;
    param->sub_gop_length = 0;  // determined in xvc_enc_encoder_create
    param->max_keypic_distance = 640;
    param->closed_gop = 0;
    param->num_ref_pics = -1;  // determined in xvc_enc_set_encoder_settings
    param->restricted_mode = 0;
    param->checksum_mode = 1;
    param->deblock = 1;
    param->beta_offset = 0;
    param->tc_offset = 0;
    param->qp = 32;
    param->flat_lambda = 0;
    param->speed_mode = -1;  // determined in xvc_enc_encoder_create
    param->tune_mode = 0;
    param->simd_mask = static_cast<uint32_t>(-1);
    param->explicit_encoder_settings = nullptr;
    return XVC_ENC_OK;
  }

  xvc_enc_return_code xvc_enc_parameters_check(const xvc_encoder_parameters
                                               *param) {
    if (!param) {
      return XVC_ENC_INVALID_ARGUMENT;
    }
    if (param->width <= 0 ||
        param->height <= 0) {
      return XVC_ENC_SIZE_NOT_POSITIVE;
    }
    if (param->width % xvc::constants::kMinCuSize ||
        param->height % xvc::constants::kMinCuSize) {
      return XVC_ENC_SIZE_NOT_MULTIPLE_OF_BLOCKSIZE;
    }
    if (param->chroma_format < XVC_ENC_CHROMA_FORMAT_MONOCHROME ||
        param->chroma_format == XVC_ENC_CHROMA_FORMAT_422 ||
        param->chroma_format > XVC_ENC_CHROMA_FORMAT_444) {
      return XVC_ENC_UNSUPPORTED_CHROMA_FORMAT;
    }
    if (param->color_matrix < XVC_ENC_COLOR_MATRIX_UNDEFINED ||
        param->color_matrix > XVC_ENC_COLOR_MATRIX_2020) {
      return XVC_ENC_INVALID_PARAMETER;
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
    if (param->sub_gop_length > xvc::constants::kMaxSubGopLength) {
      return XVC_ENC_SUB_GOP_LENGTH_TOO_LARGE;
    }
    if (param->max_keypic_distance &&
        param->sub_gop_length > param->max_keypic_distance) {
      return XVC_ENC_SUB_GOP_LENGTH_TOO_LARGE;
    }
    if (param->closed_gop < 0) {
      return XVC_ENC_INVALID_PARAMETER;
    }
    if (param->num_ref_pics > xvc::constants::kMaxNumRefPics) {
      return XVC_ENC_TOO_MANY_REF_PICS;
    }
    if (param->num_ref_pics < -1) {
      return XVC_ENC_INVALID_PARAMETER;
    }
    if (param->restricted_mode < 0 || param->restricted_mode >=
        static_cast<int>(xvc::RestrictedMode::kTotalNumber)) {
      return XVC_ENC_INVALID_PARAMETER;
    }
    if (param->checksum_mode < 0 ||
        param->checksum_mode >=
        static_cast<int>(xvc::Checksum::Mode::kTotalNumber)) {
      return XVC_ENC_INVALID_PARAMETER;
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
    if (param->qp > xvc::constants::kMaxAllowedQp ||
        param->qp < xvc::constants::kMinAllowedQp) {
      return XVC_ENC_QP_OUT_OF_RANGE;
    }
    if (param->flat_lambda < 0 || param->flat_lambda > 1) {
      return XVC_ENC_INVALID_PARAMETER;
    }
    if (param->speed_mode < -1 ||
        param->speed_mode >= static_cast<int>(xvc::SpeedMode::kTotalNumber)) {
      return XVC_ENC_INVALID_PARAMETER;
    }
    if (param->tune_mode < 0 ||
        param->tune_mode >= static_cast<int>(xvc::TuneMode::kTotalNumber)) {
      return XVC_ENC_INVALID_PARAMETER;
    }
    return XVC_ENC_OK;
  }

  void xvc_enc_set_encoder_settings(xvc::Encoder *encoder,
                                    const xvc_encoder_parameters *param) {
    xvc::EncoderSettings encoder_settings;

    if (param->speed_mode >= 0) {
      // If speed mode has been set explictly
      encoder_settings.Initialize(xvc::SpeedMode(param->speed_mode));
    } else if (param->restricted_mode) {
      // If restricted mode has been set explictly
      encoder_settings.Initialize(xvc::RestrictedMode(param->restricted_mode));
    } else {
      encoder_settings.Initialize(xvc::SpeedMode::kSlow);
    }

    if (param->tune_mode > 0) {
      encoder_settings.Tune(xvc::TuneMode(param->tune_mode));
    }

    // Explicit speed settings override the settings
    if (param->explicit_encoder_settings) {
      std::string expl_values(param->explicit_encoder_settings);
      std::string setting;
      std::stringstream stream(expl_values);
      while (stream >> setting) {
        if (setting == "fast_intra_mode_eval_level") {
          stream >> encoder_settings.fast_intra_mode_eval_level;
        } else if (setting == "fast_merge_eval") {
          stream >> encoder_settings.fast_merge_eval;
        } else if (setting == "bipred_refinement_iterations") {
          stream >> encoder_settings.bipred_refinement_iterations;
        } else if (setting == "always_evaluate_intra_in_inter") {
          stream >> encoder_settings.always_evaluate_intra_in_inter;
        } else if (setting == "default_num_ref_pics") {
          stream >> encoder_settings.default_num_ref_pics;
        } else if (setting == "max_binary_split_depth") {
          stream >> encoder_settings.max_binary_split_depth;
        } else if (setting == "fast_quad_split_based_on_binary_split") {
          stream >> encoder_settings.fast_quad_split_based_on_binary_split;
        } else if (setting == "eval_prev_mv_search_result") {
          stream >> encoder_settings.eval_prev_mv_search_result;
        } else if (setting == "fast_inter_pred_bits") {
          stream >> encoder_settings.fast_inter_pred_bits;
        } else if (setting == "smooth_lambda_scaling") {
          stream >> encoder_settings.smooth_lambda_scaling;
        } else if (setting == "adaptive_qp") {
          stream >> encoder_settings.adaptive_qp;
        } else if (setting == "aqp_strength") {
          stream >> encoder_settings.aqp_strength;
        } else if (setting == "structural_ssd") {
          stream >> encoder_settings.structural_ssd;
        }
      }
    }

    encoder->SetEncoderSettings(std::move(encoder_settings));
  }

  void xvc_enc_set_segment_length(xvc::Encoder *encoder,
                                  const xvc_encoder_parameters *param,
                                  int sub_gop_length) {
    xvc::PicNum segment_length = 1;
    if (param->max_keypic_distance == 0) {
      segment_length = ((std::numeric_limits<xvc::PicNum>::max() /
                         sub_gop_length) * sub_gop_length);
    } else {
      segment_length =
        (param->max_keypic_distance / sub_gop_length) *
        sub_gop_length;
    }
    encoder->SetSegmentLength(segment_length);

    if (param->closed_gop > 0) {
      encoder->SetClosedGopInterval(segment_length*param->closed_gop);
    } else {
      encoder->SetClosedGopInterval(((std::numeric_limits<xvc::PicNum>::max() /
                                      sub_gop_length) *
                                     sub_gop_length));
    }
  }

  xvc_encoder* xvc_enc_encoder_create(const xvc_encoder_parameters *param) {
    if (xvc_enc_parameters_check(param) != XVC_ENC_OK) {
      return nullptr;
    }
    xvc::Encoder *encoder = new xvc::Encoder();
    xvc_enc_set_encoder_settings(encoder, param);

    encoder->SetCpuCapabilities(xvc::SimdCpu::GetMaskedCaps(param->simd_mask));
    encoder->SetResolution(param->width, param->height);
    encoder->SetChromaFormat(xvc::ChromaFormat(param->chroma_format));
    encoder->SetColorMatrix(xvc::ColorMatrix(param->color_matrix));
    encoder->SetInputBitdepth(param->input_bitdepth);
    encoder->SetInternalBitdepth(param->internal_bitdepth);
    encoder->SetFramerate(param->framerate);
    encoder->SetBetaOffset(param->beta_offset);
    encoder->SetTcOffset(param->tc_offset);
    if (param->beta_offset != 0 || param->tc_offset != 0) {
      encoder->SetDeblock(3);
    } else {
      encoder->SetDeblock(param->deblock);
    }
    encoder->SetQp(param->qp);
    if (param->num_ref_pics >= 0) {
      encoder->SetNumRefPics(param->num_ref_pics);
    }
    encoder->SetFlatLambda(param->flat_lambda != 0);
    encoder->SetChecksumMode(
      static_cast<xvc::Checksum::Mode>(param->checksum_mode));

    int sub_gop_length = param->sub_gop_length;
    if (sub_gop_length == 0) {
      sub_gop_length = encoder->GetNumRefPics() > 0 ? 16 : 1;
    }
    encoder->SetSubGopLength(sub_gop_length);
    xvc_enc_set_segment_length(encoder, param, sub_gop_length);

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
                                             const uint8_t *picture_to_encode,
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
        return "Error. The input resolution is not a multiple of 8.";
      case XVC_ENC_UNSUPPORTED_CHROMA_FORMAT:
        return "Error. Unsupported chroma format.";
      case XVC_ENC_BITDEPTH_OUT_OF_RANGE:
        return  "Error. Bitdepth must be between 8 and 16, inclusive.";
      case XVC_ENC_COMPILED_BITDEPTH_TOO_LOW:
        return  "Error. This version of the xvc library is compiled for a"
          " lower bitdepth than what is requested. Please recompile the xvc"
          " library with higher bitdepth.";
      case XVC_ENC_FRAMERATE_OUT_OF_RANGE:
        return  "Error. Framerate must be between 0.0054 fps and 90000 fps,"
          " inclusive.";
      case XVC_ENC_QP_OUT_OF_RANGE:
        return  "Error. QP must be between -64 and 63, inclusive.";
      case XVC_ENC_SUB_GOP_LENGTH_TOO_LARGE:
        return  "Error. Sub Gop length too large.";
      case XVC_ENC_DEBLOCKING_SETTINGS_INVALID:
        return  "Error. Beta offset and tc offset must be in the range from"
          " -32 to 31, inclusive and deblock must be set to 1 when beta offset"
          " and/or tc offset is used.";
      case XVC_ENC_TOO_MANY_REF_PICS:
        return  "Error. Maximum allowed number of reference pictures is 5.";
      case XVC_ENC_INVALID_PARAMETER:
        return  "Error. Invalid parameter. Please check the encoder"
          " parameters.";
      default:
        return "Unkown error";
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
