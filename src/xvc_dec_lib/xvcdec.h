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

#ifndef XVC_DEC_LIB_XVCDEC_H_
#define XVC_DEC_LIB_XVCDEC_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#if defined (_WIN32)
#if defined(xvc_dec_lib_EXPORTS)
#define XVC_DEC_API __declspec(dllexport)
#elif defined(XVC_SHARED_LIB)
#define XVC_DEC_API __declspec(dllimport)
#endif
#endif
#ifndef XVC_DEC_API
#define XVC_DEC_API
#endif

#define XVC_DEC_API_VERSION   1

  typedef enum {
    XVC_DEC_OK = 0,
    XVC_DEC_NO_DECODED_PIC = 1,
    XVC_DEC_NOT_CONFORMING = 10,
    XVC_DEC_INVALID_ARGUMENT = 20,
    XVC_DEC_INVALID_PARAMETER = 30,
    XVC_DEC_FRAMERATE_OUT_OF_RANGE,
    XVC_DEC_BITDEPTH_OUT_OF_RANGE,
    XVC_DEC_BITSTREAM_VERSION_HIGHER_THAN_DECODER,
    XVC_DEC_NO_SEGMENT_HEADER_DECODED,
    XVC_DEC_BITSTREAM_BITDEPTH_TOO_HIGH,
    XVC_DEC_BITSTREAM_VERSION_LOWER_THAN_SUPPORTED_BY_DECODER,
  } xvc_dec_return_code;

  typedef enum {
    XVC_DEC_CHROMA_FORMAT_MONOCHROME = 0,
    XVC_DEC_CHROMA_FORMAT_420 = 1,
    XVC_DEC_CHROMA_FORMAT_422 = 2,
    XVC_DEC_CHROMA_FORMAT_444 = 3,
    XVC_DEC_CHROMA_FORMAT_ARGB = 4,
    XVC_DEC_CHROMA_FORMAT_UNDEFINED = 255,
  } xvc_dec_chroma_format;

  typedef enum {
    XVC_DEC_COLOR_MATRIX_UNDEFINED = 0,
    XVC_DEC_COLOR_MATRIX_601 = 1,
    XVC_DEC_COLOR_MATRIX_709 = 2,
    XVC_DEC_COLOR_MATRIX_2020 = 3,
  } xvc_dec_color_matrix;

  // Decoded picture statistics
  // Lifecycle managed by xvc_decoded_picture
  typedef struct xvc_dec_pic_stats {
    uint32_t nal_unit_type;
    uint32_t poc;
    uint32_t doc;
    uint32_t soc;
    uint32_t tid;
    int32_t l0[5];
    int32_t l1[5];
    int32_t bitdepth;
    int32_t bitstream_bitdepth;
    int32_t width;
    int32_t height;
    int32_t qp;
    xvc_dec_chroma_format chroma_format;
    xvc_dec_color_matrix color_matrix;
    double framerate;
    double bitstream_framerate;
    int32_t conforming;
  } xvc_dec_pic_stats;

  // Represents a decoded picture
  // Lifecycle managed by api->picture_create & api->picture_destory
  // Populated using api->decoder_get_picture
  typedef struct xvc_decoded_picture {
    const char* bytes;  // Address of first picture sample
    size_t size;        // Number of pic bytes for all planes (incl. padding)
    const char *planes[3];  // Address of first sample for each plane
    int stride[3];          // Width in bytes for each plane (including padding)
    xvc_dec_pic_stats stats;
    int64_t user_data;
  } xvc_decoded_picture;

  // xvc decoder instance
  // Lifecycle managed by api->decoder_create & api->decoder_destroy
  typedef struct xvc_decoder xvc_decoder;

  // xvc decoder configuration
  // Lifecycle managed by api->parameters_create & api->parameters_destroy
  typedef struct xvc_decoder_parameters {
    int output_width;
    int output_height;
    xvc_dec_chroma_format output_chroma_format;
    xvc_dec_color_matrix output_color_matrix;
    int output_bitdepth;
    double max_framerate;
    int threads;
    uint32_t simd_mask;
    int dither;
  } xvc_decoder_parameters;

  // xvc decoder api
  // Lifecycle managed by xvc_decoder_api_get
  typedef struct xvc_decoder_api {
    // Parameters
    xvc_decoder_parameters* (*parameters_create)(void);
    xvc_dec_return_code(*parameters_destroy)(
      xvc_decoder_parameters *param);
    xvc_dec_return_code(*parameters_set_default)(
      xvc_decoder_parameters *param);
    xvc_dec_return_code(*parameters_check)(xvc_decoder_parameters *param);
    // Decoded picture
    xvc_decoded_picture* (*picture_create)(xvc_decoder *decoder);
    xvc_dec_return_code(*picture_destroy)(xvc_decoded_picture* pic);
    // Decoder
    xvc_decoder* (*decoder_create)(xvc_decoder_parameters *decoder);
    xvc_dec_return_code(*decoder_destroy)(xvc_decoder *decoder);
    xvc_dec_return_code(*decoder_update_parameters)(xvc_decoder *decoder,
                                                    xvc_decoder_parameters
                                                    *param);
    // Decode the specified nal unit
    // user_data = optional application data to be set together with the
    // decoded picture. Note that user data is only returned for picture nals.
    xvc_dec_return_code(*decoder_decode_nal)(xvc_decoder *decoder,
                                             const uint8_t *nal_unit,
                                             size_t nal_unit_size,
                                             int64_t user_data);
    // Get next output picture that is available in display order.
    // Pointers to picture sample data are only valid until next API call.
    xvc_dec_return_code(*decoder_get_picture)(xvc_decoder *decoder,
                                              xvc_decoded_picture *out_pic);
    xvc_dec_return_code(*decoder_flush)(xvc_decoder *decoder);
    xvc_dec_return_code(*decoder_check_conformance)(xvc_decoder *decoder,
                                                    int *num);
    // Misc
    const char*(*xvc_dec_get_error_text)(xvc_dec_return_code error_code);
  } xvc_decoder_api;

  // Starting point for using the xvc decoder api
  XVC_DEC_API const xvc_decoder_api* xvc_decoder_api_get(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // XVC_DEC_LIB_XVCDEC_H_

