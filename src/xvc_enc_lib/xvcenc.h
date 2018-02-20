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

#ifndef XVC_ENC_LIB_XVCENC_H_
#define XVC_ENC_LIB_XVCENC_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XVC_ENC_API_VERSION   1

  typedef enum {
    XVC_ENC_OK = 0,
    XVC_ENC_INVALID_ARGUMENT = 10,
    XVC_ENC_INVALID_PARAMETER = 20,
    XVC_ENC_SIZE_TOO_SMALL,
    XVC_ENC_UNSUPPORTED_CHROMA_FORMAT,
    XVC_ENC_BITDEPTH_OUT_OF_RANGE,
    XVC_ENC_COMPILED_BITDEPTH_TOO_LOW,
    XVC_ENC_FRAMERATE_OUT_OF_RANGE,
    XVC_ENC_QP_OUT_OF_RANGE,
    XVC_ENC_SUB_GOP_LENGTH_TOO_LARGE,
    XVC_ENC_DEBLOCKING_SETTINGS_INVALID,
    XVC_ENC_TOO_MANY_REF_PICS,
    XVC_ENC_SIZE_TOO_LARGE,
  } xvc_enc_return_code;

  typedef enum {
    XVC_ENC_CHROMA_FORMAT_MONOCHROME = 0,
    XVC_ENC_CHROMA_FORMAT_420 = 1,
    XVC_ENC_CHROMA_FORMAT_422 = 2,
    XVC_ENC_CHROMA_FORMAT_444 = 3,
    XVC_ENC_CHROMA_FORMAT_UNDEFINED = 255,
  } xvc_enc_chroma_format;

  typedef enum {
    XVC_ENC_COLOR_MATRIX_UNDEFINED = 0,
    XVC_ENC_COLOR_MATRIX_601 = 1,
    XVC_ENC_COLOR_MATRIX_709 = 2,
    XVC_ENC_COLOR_MATRIX_2020 = 3,
  } xvc_enc_color_matrix;

  // Statistics for picture encoded by nal
  // Lifecycle managed by xvc_enc_nal_unit
  typedef struct xvc_enc_nal_stats {
    uint32_t nal_unit_type;
    uint32_t poc;
    uint32_t doc;
    uint32_t soc;
    uint32_t tid;
    int32_t qp;
    int32_t l0[5];
    int32_t l1[5];
  } xvc_enc_nal_stats;

  // NAL unit representing the coded bitstream
  // Lifecycle managed by api
  typedef struct xvc_enc_nal_unit {
    uint8_t *bytes;
    size_t size;
    int buffer_flag;
    xvc_enc_nal_stats stats;
  } xvc_enc_nal_unit;

  // Represents one reconstructed picture
  // Lifecycle managed by api->picture_create & api->picture_destroy
  typedef struct xvc_enc_pic_buffer {
    uint8_t *pic;
    size_t size;
  } xvc_enc_pic_buffer;

  // xvc encoder instance
  // Lifecycle managed by api->encoder_create & api->encoder_destroy
  typedef struct xvc_encoder xvc_encoder;

  // xvc encoder configuration
  // Lifecycle managed by api->parameters_create & api->parameters_destroy
  typedef struct xvc_encoder_parameters {
    int width;
    int height;
    xvc_enc_chroma_format chroma_format;
    xvc_enc_color_matrix color_matrix;
    uint32_t input_bitdepth;
    uint32_t internal_bitdepth;
    double framerate;
    uint32_t sub_gop_length;
    uint32_t max_keypic_distance;
    int closed_gop;
    int num_ref_pics;
    int restricted_mode;
    int chroma_qp_offset_table;
    int chroma_qp_offset_u;
    int chroma_qp_offset_v;
    int deblock;
    int beta_offset;
    int tc_offset;
    int qp;
    int flat_lambda;
    int leading_pictures;
    int speed_mode;
    int tune_mode;
    int checksum_mode;
    uint32_t simd_mask;
    char* explicit_encoder_settings;
  } xvc_encoder_parameters;

  // xvc encoder api
  // Lifecycle managed by xvc_encoder_api_get
  typedef struct xvc_encoder_api {
    // Parameters
    xvc_encoder_parameters* (*parameters_create)(void);
    xvc_enc_return_code(*parameters_destroy)(xvc_encoder_parameters
                                             *param);
    xvc_enc_return_code(*parameters_set_default)(
      xvc_encoder_parameters *param);
    xvc_enc_return_code(*parameters_check)(const xvc_encoder_parameters *param);
    // Reconstructed picture
    xvc_enc_pic_buffer* (*picture_create)(xvc_encoder *encoder);
    xvc_enc_return_code(*picture_destroy)(xvc_enc_pic_buffer *picture);
    // Encoder
    xvc_encoder* (*encoder_create)(const xvc_encoder_parameters *param);
    xvc_enc_return_code(*encoder_destroy)(xvc_encoder *encoder);
    xvc_enc_return_code(*encoder_encode)(xvc_encoder *encoder,
                                         const uint8_t *picture_to_encode,
                                         xvc_enc_nal_unit **nal_units,
                                         int *num_nal_units,
                                         xvc_enc_pic_buffer *rec_pic);
    xvc_enc_return_code(*encoder_flush)(xvc_encoder *encoder,
                                        xvc_enc_nal_unit **nal_units,
                                        int *num_nal_units,
                                        xvc_enc_pic_buffer *rec_pic);
    // Misc
    const char*(*xvc_enc_get_error_text)(xvc_enc_return_code error_code);
  } xvc_encoder_api;

  // Starting point for using the xvc encoder api
  const xvc_encoder_api* xvc_encoder_api_get(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // XVC_ENC_LIB_XVCENC_H_
