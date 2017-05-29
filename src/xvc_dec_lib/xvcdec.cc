/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_dec_lib/xvcdec.h"

#include <cstring>

#include "xvc_dec_lib/decoder.h"


#ifdef __cplusplus
extern "C" {
#endif
  xvc_decoder_parameters* xvc_dec_parameters_create() {
    xvc_decoder_parameters *parameters = new xvc_decoder_parameters;
    std::memset(parameters, 0, sizeof(xvc_decoder_parameters));
    return parameters;
  }

  xvc_dec_return_code xvc_dec_parameters_destroy(
    xvc_decoder_parameters *param) {
    if (param) {
      delete param;
    }
    return XVC_DEC_OK;
  }

  xvc_dec_return_code xvc_dec_parameters_set_default(
    xvc_decoder_parameters *param) {
    if (!param) {
      return XVC_DEC_INVALID_ARGUMENT;
    }
    param->output_width = 0;
    param->output_height = 0;
    param->output_chroma_format = XVC_DEC_CHROMA_FORMAT_UNDEFINED;
    param->output_color_matrix = XVC_DEC_COLOR_MATRIX_UNDEFINED;
    param->output_bitdepth = 0;
    param->max_framerate = xvc::constants::kTimeScale;
    return XVC_DEC_OK;
  }

  xvc_dec_return_code xvc_dec_parameters_check(xvc_decoder_parameters
                                               *param) {
    if (!param) {
      return XVC_DEC_INVALID_ARGUMENT;
    }
    if (param->output_width < 0 ||
        param->output_width == 1 ||
        param->output_height < 0 ||
        param->output_height == 1) {
      return XVC_DEC_INVALID_PARAMETER;
    }
    if (param->output_chroma_format != XVC_DEC_CHROMA_FORMAT_MONOCHROME &&
        param->output_chroma_format != XVC_DEC_CHROMA_FORMAT_420 &&
        param->output_chroma_format != XVC_DEC_CHROMA_FORMAT_422 &&
        param->output_chroma_format != XVC_DEC_CHROMA_FORMAT_444 &&
        param->output_chroma_format != XVC_DEC_CHROMA_FORMAT_ARGB &&
        param->output_chroma_format != XVC_DEC_CHROMA_FORMAT_UNDEFINED) {
      return XVC_DEC_INVALID_PARAMETER;
    }
    if (param->output_color_matrix != XVC_DEC_COLOR_MATRIX_UNDEFINED &&
        param->output_color_matrix != XVC_DEC_COLOR_MATRIX_601 &&
        param->output_color_matrix != XVC_DEC_COLOR_MATRIX_709 &&
        param->output_color_matrix != XVC_DEC_COLOR_MATRIX_2020) {
      return XVC_DEC_INVALID_PARAMETER;
    }
    if (param->output_bitdepth != 0 &&
      (param->output_bitdepth > 16 || param->output_bitdepth < 8)) {
      return XVC_DEC_BITDEPTH_OUT_OF_RANGE;
    }
    if (param->output_bitdepth != 0 &&
      (param->output_bitdepth > 16 || param->output_bitdepth < 8)) {
      return XVC_DEC_BITDEPTH_OUT_OF_RANGE;
    }
    if (param->max_framerate < (1.0 * xvc::constants::kTimeScale /
      (1 << xvc::constants::kFrameRateBitDepth)) ||
        param->max_framerate > xvc::constants::kTimeScale) {
      return XVC_DEC_FRAMERATE_OUT_OF_RANGE;
    }
    return XVC_DEC_OK;
  }

  xvc_decoder* xvc_dec_decoder_create(xvc_decoder_parameters *param) {
    if (xvc_dec_parameters_check(param) != XVC_DEC_OK) {
      return nullptr;
    }
    xvc::Decoder *decoder = new xvc::Decoder;
    decoder->SetOutputWidth(param->output_width);
    decoder->SetOutputHeight(param->output_height);
    decoder->SetOutputChromaFormat(param->output_chroma_format);
    decoder->SetOutputColorMatrix(param->output_color_matrix);
    decoder->SetOutputBitdepth(param->output_bitdepth);
    decoder->SetDecoderTicks(static_cast<int>(xvc::constants::kTimeScale
                                              / param->max_framerate));
    return decoder;
  }

  xvc_dec_return_code xvc_dec_decoder_destroy(xvc_decoder *decoder) {
    if (decoder) {
      xvc::Decoder *lib_decoder = reinterpret_cast<xvc::Decoder*>(decoder);
      delete lib_decoder;
    }
    return XVC_DEC_OK;
  }

  xvc_dec_return_code xvc_dec_decoder_decode_nal(xvc_decoder *decoder,
                                                 const uint8_t *nal_unit,
                                                 size_t nal_unit_size) {
    if (!decoder || !nal_unit || nal_unit_size < 1) {
      return XVC_DEC_INVALID_ARGUMENT;
    }
    xvc::Decoder *lib_decoder = reinterpret_cast<xvc::Decoder*>(decoder);
    lib_decoder->DecodeNal(nal_unit, nal_unit_size);
    if (lib_decoder->GetState() == xvc::Decoder::State::kDecoderVersionTooLow) {
      return XVC_DEC_BITSTREAM_VERSION_HIGHER_THAN_DECODER;
    } else if (lib_decoder->GetState() ==
               xvc::Decoder::State::kBitstreamBitdepthTooHigh) {
      return XVC_DEC_BITSTREAM_BITDEPTH_TOO_HIGH;
    } else if (lib_decoder->GetState() ==
               xvc::Decoder::State::kNoSegmentHeader) {
      return XVC_DEC_NO_SEGMENT_HEADER_DECODED;
    }
    return XVC_DEC_OK;
  }

  xvc_dec_return_code xvc_dec_decoder_get_picture(xvc_decoder *decoder,
                                          xvc_decoded_picture *pic_bytes) {
    if (!decoder || !pic_bytes) {
      return XVC_DEC_INVALID_ARGUMENT;
    }
    xvc::Decoder *lib_decoder = reinterpret_cast<xvc::Decoder*>(decoder);
    if (lib_decoder->GetState() == xvc::Decoder::State::kNoSegmentHeader) {
      return XVC_DEC_NO_SEGMENT_HEADER_DECODED;
    }
    if (lib_decoder->HasPictureReadyForOutput() &&
        lib_decoder->GetDecodedPicture(pic_bytes)) {
      return XVC_DEC_OK;
    } else {
      return XVC_DEC_NO_DECODED_PIC;
    }
  }

  xvc_dec_return_code xvc_dec_decoder_flush(xvc_decoder *decoder,
                                            xvc_decoded_picture *pic_bytes) {
    if (!decoder || !pic_bytes) {
      return XVC_DEC_INVALID_ARGUMENT;
    }
    xvc::Decoder *lib_decoder = reinterpret_cast<xvc::Decoder*>(decoder);
    if (lib_decoder->GetNumDecodedPics() > 0) {
      lib_decoder->FlushBufferedTailPics();
      return lib_decoder->GetDecodedPicture(pic_bytes) ?
        XVC_DEC_OK : XVC_DEC_NO_DECODED_PIC;
    } else {
      return XVC_DEC_NO_DECODED_PIC;
    }
  }

  xvc_dec_return_code xvc_dec_decoder_check_conformance(xvc_decoder *decoder,
                                                int *num) {
    if (!decoder || !num) {
      return XVC_DEC_INVALID_ARGUMENT;
    }
    xvc::Decoder *lib_decoder = reinterpret_cast<xvc::Decoder*>(decoder);
    *num = static_cast<uint32_t>(lib_decoder->GetNumCorruptedPics());
    if (*num > 0) {
      return XVC_DEC_NOT_CONFORMING;
    }
    return XVC_DEC_OK;
  }

  const char* xvc_dec_get_error_text(xvc_dec_return_code error_code) {
    switch (error_code) {
      case  XVC_DEC_OK:
        return "";
      case XVC_DEC_NO_DECODED_PIC:
        return "No decoded picture";
      case XVC_DEC_NOT_CONFORMING:
        return "Non-conforming bitstream";
      case XVC_DEC_INVALID_ARGUMENT:
        return "Invalid api argument";
      case XVC_DEC_FRAMERATE_OUT_OF_RANGE:
        return "Invalid framerate";
      case XVC_DEC_BITDEPTH_OUT_OF_RANGE:
        return "Invalid bitdepth";
      case XVC_DEC_BITSTREAM_VERSION_HIGHER_THAN_DECODER:
        return "The xvc version indicated in the segment header is "
          "higher than the xvc version of the decoder."
          "Please update the xvc decoder to the latest version.";
      case XVC_DEC_NO_SEGMENT_HEADER_DECODED:
        return "No segment header decoded";
      case XVC_DEC_BITSTREAM_BITDEPTH_TOO_HIGH:
        return "The bitstream is of higher bitdepth than what the "
          "decoder has been compiled for. "
          "Please recompile the decoder to support higher bitdepth "
          "(by setting XVC_HIGH_BITDEPTH equal to 1).";
      case XVC_DEC_INVALID_PARAMETER:
        return "Invalid parameter";
      default:
        return "Unkown error";
    }
  }

  static const xvc_decoder_api xvc_dec_api_internal = {
    &xvc_dec_parameters_create,
    &xvc_dec_parameters_destroy,
    &xvc_dec_parameters_set_default,
    &xvc_dec_parameters_check,
    &xvc_dec_decoder_create,
    &xvc_dec_decoder_destroy,
    &xvc_dec_decoder_decode_nal,
    &xvc_dec_decoder_get_picture,
    &xvc_dec_decoder_flush,
    &xvc_dec_decoder_check_conformance,
    &xvc_dec_get_error_text,
  };

  const xvc_decoder_api* xvc_decoder_api_get() {
    return &xvc_dec_api_internal;
  }

}  // extern C
