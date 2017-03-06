/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_LIB_XVCENC_H_
#define XVC_ENC_LIB_XVCENC_H_

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

  enum xvc_enc_return_code {
    XVC_ENC_OK = 0,
    XVC_ENC_INVALID_ARGUMENT,
    XVC_ENC_SIZE_NOT_POSITIVE,
    XVC_ENC_SIZE_NOT_MULTIPLE_OF_BLOCKSIZE,
    XVC_ENC_UNSUPPORTED_CHROMA_FORMAT,
    XVC_ENC_BITDEPTH_OUT_OF_RANGE,
    XVC_ENC_COMPILED_BITDEPTH_TOO_LOW,
    XVC_ENC_FRAMERATE_OUT_OF_RANGE,
    XVC_ENC_QP_OUT_OF_RANGE,
    XVC_ENC_SUB_GOP_LENGTH_TOO_LARGE,
    XVC_ENC_DEBLOCKING_SETTINGS_INVALID,
    XVC_ENC_INVALID_PARAMETER,
  };

  enum xvc_enc_chroma_format {
    XVC_ENC_CHROMA_FORMAT_MONOCHROME = 0,
    XVC_ENC_CHROMA_FORMAT_420 = 1,
    XVC_ENC_CHROMA_FORMAT_422 = 2,
    XVC_ENC_CHROMA_FORMAT_444 = 3,
    XVC_ENC_CHROMA_FORMAT_UNDEFINED = 255,
  };

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

  typedef struct xvc_enc_nal_unit {
    uint8_t *bytes;
    size_t size;
    int buffer_flag;
    xvc_enc_nal_stats stats;
  } xvc_enc_nal_unit;

  typedef struct xvc_enc_pic_buffer {
    uint8_t *pic;
    size_t size;
  } xvc_enc_pic_buffer;


  typedef struct xvc_encoder xvc_encoder;

  typedef struct xvc_encoder_parameters {
    int width;
    int height;
    xvc_enc_chroma_format chroma_format;
    uint32_t input_bitdepth;
    uint32_t internal_bitdepth;
    double framerate;
    uint32_t sub_gop_length;
    uint32_t max_keypic_distance;
    int closed_gop;
    int qp;
    int deblock;
    int beta_offset;
    int tc_offset;
    int all_intra;
    int flat_lambda;
  } xvc_encoder_parameters;

  typedef struct xvc_encoder_api {
    xvc_encoder_parameters* (*parameters_create)();
    xvc_enc_return_code(*parameters_destroy)(xvc_encoder_parameters
                                             *param);
    xvc_enc_return_code(*parameters_set_default)(
      xvc_encoder_parameters *param);
    xvc_enc_return_code(*parameters_check)(xvc_encoder_parameters *param);

    xvc_encoder* (*encoder_create)(xvc_encoder_parameters *param);
    xvc_enc_return_code(*encoder_destroy)(xvc_encoder *encoder);
    xvc_enc_return_code(*encoder_encode)(xvc_encoder *encoder,
                                         uint8_t *picture_to_encode,
                                         xvc_enc_nal_unit **nal_units,
                                         int *num_nal_units,
                                         xvc_enc_pic_buffer *rec_pic);
    xvc_enc_return_code(*encoder_flush)(xvc_encoder *encoder,
                                        xvc_enc_nal_unit **nal_units,
                                        int *num_nal_units,
                                        xvc_enc_pic_buffer *rec_pic);
    const char*(*xvc_enc_get_error_text)(xvc_enc_return_code error_code);
  } xvc_encoder_api;

  const xvc_encoder_api* xvc_encoder_api_get();

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // XVC_ENC_LIB_XVCENC_H_
