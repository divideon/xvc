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

#ifndef XVC_DEC_LIB_XVCDEC_H_
#define XVC_DEC_LIB_XVCDEC_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
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
  typedef struct xvc_decoded_picture {
    char* bytes;      // Adress of first picture sample
    size_t size;      // Number of picture bytes for all planes (incl. padding)
    char *planes[3];  // Adress of first sample for each plane
    int stride[3];    // Width in bytes for each plane (including padding)
    xvc_dec_pic_stats stats;
    int64_t user_data;  //
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
    xvc_dec_return_code(*decoder_get_picture)(xvc_decoder *decoder,
                                              xvc_decoded_picture *out_pic);
    xvc_dec_return_code(*decoder_flush)(xvc_decoder *decoder,
                                        xvc_decoded_picture *out_pic);
    xvc_dec_return_code(*decoder_check_conformance)(xvc_decoder *decoder,
                                                    int *num);
    // Misc
    const char*(*xvc_dec_get_error_text)(xvc_dec_return_code error_code);
  } xvc_decoder_api;

  // Starting point for using the xvc decoder api
  const xvc_decoder_api* xvc_decoder_api_get(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // XVC_DEC_LIB_XVCDEC_H_

