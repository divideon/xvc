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

#include "xvc_enc_lib/xvcenc.h"

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "xvc_common_lib/common.h"
#include "xvc_enc_lib/encoder.h"
#include "xvc_enc_lib/encoder_settings.h"

#ifdef __cplusplus
extern "C" {
#endif

  static const int kDefaultSubGopLength = 16;

  static xvc_encoder_parameters* xvc_enc_parameters_create() {
    xvc_encoder_parameters *parameters = new xvc_encoder_parameters;
    std::memset(parameters, 0, sizeof(xvc_encoder_parameters));
    return parameters;
  }

  static xvc_enc_return_code
    xvc_enc_parameters_destroy(xvc_encoder_parameters *param) {
    if (param) {
      delete param;
    }
    return XVC_ENC_OK;
  }

  static xvc_enc_return_code
    xvc_enc_parameters_set_default(xvc_encoder_parameters *param) {
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
    param->low_delay = 0;
    param->num_ref_pics = -1;  // determined in xvc_enc_set_encoder_settings
    param->restricted_mode = 0;
    param->checksum_mode = 0;
    // Following three parameters determined in xvc_enc_set_encoder_settings
    param->chroma_qp_offset_table = -1;
    param->chroma_qp_offset_u = std::numeric_limits<int>::min();
    param->chroma_qp_offset_v = std::numeric_limits<int>::min();
    param->deblock = 1;
    param->beta_offset = 0;
    param->tc_offset = 0;
    param->qp = 32;
    param->flat_lambda = 0;
    param->lambda_a = 0;
    param->lambda_b = 0;
    param->leading_pictures = 0;
    param->speed_mode = -1;  // determined in xvc_enc_encoder_create
    param->tune_mode = 0;
    param->threads = 0;
    param->simd_mask = static_cast<uint32_t>(-1);
    param->explicit_encoder_settings = nullptr;
    return XVC_ENC_OK;
  }

  static xvc_enc_return_code
    xvc_enc_parameters_apply_rd_preset(int preset,
                                       xvc_encoder_parameters *param) {
    if (!param) {
      return XVC_ENC_INVALID_ARGUMENT;
    }
    switch (preset) {
      case 0:
        // default
        param->flat_lambda = 0;
        param->leading_pictures = 0;
        break;

      case 1:
        param->leading_pictures = 1;
        break;

      case 2:
        param->flat_lambda = param->sub_gop_length > 0 ?
          param->sub_gop_length : kDefaultSubGopLength;
        break;

      case 3:
        // Lower qp with fixed offset (low motion bias)
        param->leading_pictures = 1;
        param->lambda_a = pow(2.0f, -5 / 3.0f);
        param->lambda_b = 1.0f / 22;
        break;

      default:
        return XVC_ENC_NO_SUCH_PRESET;
    }
    return XVC_ENC_OK;
  }

  static xvc_enc_return_code
    xvc_enc_parameters_check(const xvc_encoder_parameters *param) {
    if (!param) {
      return XVC_ENC_INVALID_ARGUMENT;
    }
    if (param->width < 2 * xvc::constants::kMinCuSize ||
        param->height < 2 * xvc::constants::kMinCuSize) {
      return XVC_ENC_SIZE_TOO_SMALL;
    }
    if (param->width >= (1 << xvc::constants::kPicSizeBits) ||
        param->height >= (1 << xvc::constants::kPicSizeBits)) {
      return XVC_ENC_SIZE_TOO_LARGE;
    }
    if (param->chroma_format < XVC_ENC_CHROMA_FORMAT_MONOCHROME ||
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
    if (param->low_delay < 0 || param->low_delay > 1) {
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
    if (param->deblock < 0 || param->deblock > 2) {
      return XVC_ENC_DEBLOCKING_SETTINGS_INVALID;
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
    if (param->flat_lambda < 0 ||
        param->flat_lambda >
        static_cast<int>(xvc::constants::kMaxSubGopLength)) {
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

  static xvc_enc_pic_buffer*
    xvc_enc_picture_create(xvc_encoder *encoder) {
    return new xvc_enc_pic_buffer();
  }

  static xvc_enc_return_code
    xvc_enc_picture_destroy(xvc_enc_pic_buffer *picture) {
    delete picture;
    return XVC_ENC_OK;
  }

  static void
    xvc_enc_set_encoder_settings(xvc::Encoder *encoder,
                                 const xvc_encoder_parameters *param) {
    xvc::EncoderSettings encoder_settings;
    if (param->speed_mode >= 0) {
      // If speed mode has been set explictly
      encoder_settings.Initialize(xvc::SpeedMode(param->speed_mode));
    } else {
      encoder_settings.Initialize(xvc::SpeedMode::kSlow);
    }
    if (param->restricted_mode) {
      // If restricted mode has been set explictly
      encoder_settings.Initialize(xvc::RestrictedMode(param->restricted_mode));
    }

    if (param->tune_mode > 0) {
      encoder_settings.Tune(xvc::TuneMode(param->tune_mode));
    }

    encoder_settings.leading_pictures = param->leading_pictures;
    encoder_settings.flat_lambda = param->flat_lambda;
    if (param->lambda_a != 0) {
      encoder_settings.lambda_scale_a = param->lambda_a;
    }
    if (param->lambda_b != 0) {
      encoder_settings.lambda_scale_b = param->lambda_b;
    }

    // Explicit speed settings override the settings
    if (param->explicit_encoder_settings) {
      std::string explicit_settings(param->explicit_encoder_settings);
      encoder_settings.ParseExplicitSettings(explicit_settings);
    }

    encoder->SetEncoderSettings(std::move(encoder_settings));
  }

  static void xvc_enc_set_segment_length(xvc::Encoder *encoder,
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

  static xvc_encoder*
    xvc_enc_encoder_create(const xvc_encoder_parameters *param) {
    if (xvc_enc_parameters_check(param) != XVC_ENC_OK) {
      return nullptr;
    }
    xvc::Encoder *encoder = new xvc::Encoder(param->internal_bitdepth,
                                             param->threads);
    xvc_enc_set_encoder_settings(encoder, param);

    encoder->SetCpuCapabilities(xvc::SimdCpu::GetMaskedCaps(param->simd_mask));
    encoder->SetResolution(param->width, param->height);
    encoder->SetChromaFormat(xvc::ChromaFormat(param->chroma_format));
    encoder->SetColorMatrix(xvc::ColorMatrix(param->color_matrix));
    encoder->SetInputBitdepth(param->input_bitdepth);
    encoder->SetFramerate(param->framerate);
    if (param->chroma_qp_offset_table >= 0) {
      encoder->SetChromaQpOffsetTable(param->chroma_qp_offset_table);
    }
    if (param->chroma_qp_offset_u > std::numeric_limits<int>::min()) {
      encoder->SetChromaQpOffsetU(param->chroma_qp_offset_u);
    }
    if (param->chroma_qp_offset_v > std::numeric_limits<int>::min()) {
      encoder->SetChromaQpOffsetV(param->chroma_qp_offset_v);
    }
    encoder->SetBetaOffset(param->beta_offset);
    encoder->SetTcOffset(param->tc_offset);
    if (param->deblock == 1 &&
      (param->beta_offset != 0 || param->tc_offset != 0)) {
      encoder->SetDeblockingMode(xvc::DeblockingMode::kCustom);
    } else {
      switch (param->deblock) {
        case 0:
          encoder->SetDeblockingMode(xvc::DeblockingMode::kDisabled);
          break;
        case 1:
          encoder->SetDeblockingMode(xvc::DeblockingMode::kEnabled);
          break;
        case 2:
          encoder->SetDeblockingMode(xvc::DeblockingMode::kPerPicture);
          break;
        default:
          assert(0);
      }
    }
    encoder->SetQp(param->qp);
    encoder->SetLowDelay(param->low_delay != 0);
    if (param->num_ref_pics >= 0) {
      encoder->SetNumRefPics(param->num_ref_pics);
    } else if (param->low_delay != 0) {
      encoder->SetNumRefPics(4);
    }
    encoder->SetChecksumMode(
      static_cast<xvc::Checksum::Mode>(param->checksum_mode));

    int sub_gop_length = param->sub_gop_length;
    if (sub_gop_length == 0) {
      sub_gop_length = encoder->GetNumRefPics() > 0 ? kDefaultSubGopLength : 1;
    }
    encoder->SetSubGopLength(sub_gop_length);
    xvc_enc_set_segment_length(encoder, param, sub_gop_length);

    return encoder;
  }

  static xvc_enc_return_code xvc_enc_encoder_destroy(xvc_encoder *encoder) {
    if (encoder) {
      xvc::Encoder *lib_encoder = reinterpret_cast<xvc::Encoder*>(encoder);
      delete lib_encoder;
    }
    return XVC_ENC_OK;
  }

  static xvc_enc_return_code
    xvc_enc_encoder_encode(xvc_encoder *encoder, const uint8_t *input_picture,
                           xvc_enc_nal_unit **nal_units, int *num_nal_units,
                           xvc_enc_pic_buffer *rec_pic) {
    if (!encoder || !input_picture || !nal_units || !num_nal_units) {
      return XVC_ENC_INVALID_ARGUMENT;
    }
    xvc::Encoder *lib_encoder = reinterpret_cast<xvc::Encoder*>(encoder);
    bool success = lib_encoder->Encode(input_picture, rec_pic);
    std::vector<xvc_enc_nal_unit> &output_nals = lib_encoder->GetOutputNals();
    if (output_nals.size() > 0) {
      *nal_units = &output_nals[0];
      *num_nal_units = static_cast<int>(output_nals.size());
    } else {
      *nal_units = nullptr;
      *num_nal_units = 0;
    }
    return success ? XVC_ENC_OK : XVC_ENC_INVALID_ARGUMENT;
  }

  static xvc_enc_return_code
    xvc_enc_encoder_encode2(xvc_encoder *encoder, const uint8_t *plane_bytes[3],
                            int plane_stride[3], xvc_enc_nal_unit **nal_units,
                            int *num_nal_units, xvc_enc_pic_buffer *rec_pic,
                            int64_t user_data) {
    if (!encoder || !plane_bytes || !nal_units || !num_nal_units) {
      return XVC_ENC_INVALID_ARGUMENT;
    }
    xvc::Encoder::PicPlanes pic_planes = {
      std::make_pair(plane_bytes[0], plane_stride[0]),
      std::make_pair(plane_bytes[1], plane_stride[1]),
      std::make_pair(plane_bytes[2], plane_stride[2])
    };
    xvc::Encoder *lib_encoder = reinterpret_cast<xvc::Encoder*>(encoder);
    bool success = lib_encoder->Encode(pic_planes, rec_pic, user_data);
    std::vector<xvc_enc_nal_unit> &output_nals = lib_encoder->GetOutputNals();
    if (output_nals.size() > 0) {
      *nal_units = &output_nals[0];
      *num_nal_units = static_cast<int>(output_nals.size());
    } else {
      *nal_units = nullptr;
      *num_nal_units = 0;
    }
    return success ? XVC_ENC_OK : XVC_ENC_INVALID_ARGUMENT;
  }

  static xvc_enc_return_code
    xvc_enc_encoder_flush(xvc_encoder *encoder, xvc_enc_nal_unit **nal_units,
                          int *num_nal_units, xvc_enc_pic_buffer *rec_pic) {
    if (!encoder || !nal_units || !num_nal_units) {
      return XVC_ENC_INVALID_ARGUMENT;
    }
    xvc::Encoder *lib_encoder = reinterpret_cast<xvc::Encoder*>(encoder);
    bool success = lib_encoder->Flush(rec_pic);
    std::vector<xvc_enc_nal_unit> &output_nals = lib_encoder->GetOutputNals();
    if (output_nals.size() > 0) {
      *nal_units = &output_nals[0];
      *num_nal_units = static_cast<int>(output_nals.size());
    } else {
      *nal_units = nullptr;
      *num_nal_units = 0;
    }
    return success ? XVC_ENC_OK : XVC_ENC_NO_MORE_OUTPUT;
  }

  static const char* xvc_enc_get_error_text(xvc_enc_return_code error_code) {
    switch (error_code) {
      case XVC_ENC_OK:
        return "No Error";
      case XVC_ENC_INVALID_ARGUMENT:
        return "Error. One or more invalid arguments provided to an xvc api"
          " function.";
      case XVC_ENC_SIZE_TOO_SMALL:
        return "Error. The input width and height must larger than or equal"
          " to 16.";
      case XVC_ENC_SIZE_TOO_LARGE:
        return "Error. The input width and height must be smaller than or equal"
          " to 65535.";
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
      case XVC_ENC_NO_SUCH_PRESET:
        return "No such multi-pass preset configuration exists";
      default:
        return "Unkown error";
    }
  }

  static const xvc_encoder_api xvc_enc_api_internal = {
    &xvc_enc_parameters_create,
    &xvc_enc_parameters_destroy,
    &xvc_enc_parameters_set_default,
    &xvc_enc_parameters_apply_rd_preset,
    &xvc_enc_parameters_check,
    &xvc_enc_picture_create,
    &xvc_enc_picture_destroy,
    &xvc_enc_encoder_create,
    &xvc_enc_encoder_destroy,
    &xvc_enc_encoder_encode,
    &xvc_enc_encoder_encode2,
    &xvc_enc_encoder_flush,
    &xvc_enc_get_error_text,
  };

  const xvc_encoder_api* xvc_encoder_api_get() {
    return &xvc_enc_api_internal;
  }

}  // extern C
