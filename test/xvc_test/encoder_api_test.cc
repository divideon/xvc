/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "googletest/include/gtest/gtest.h"

#include "xvc_common_lib/common.h"
#include "xvc_enc_lib/xvcenc.h"

namespace {

TEST(EncoderAPI, NullPtrCalls) {
  const xvc_encoder_api *api = xvc_encoder_api_get();
  EXPECT_EQ(XVC_ENC_OK, api->parameters_destroy(nullptr));
  EXPECT_EQ(XVC_ENC_INVALID_ARGUMENT, api->parameters_set_default(nullptr));
  EXPECT_EQ(XVC_ENC_INVALID_ARGUMENT, api->parameters_check(nullptr));
  EXPECT_EQ(nullptr, api->encoder_create(nullptr));
  EXPECT_EQ(XVC_ENC_OK, api->encoder_destroy(nullptr));
}

TEST(EncoderAPI, ParamCheck) {
  const xvc_encoder_api *api = xvc_encoder_api_get();
  xvc_encoder_parameters *params = api->parameters_create();

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->width = 0;
  params->height = 0;
  EXPECT_EQ(XVC_ENC_SIZE_NOT_POSITIVE, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->height = 11;
  params->width = 64;
  EXPECT_EQ(XVC_ENC_SIZE_NOT_MULTIPLE_OF_BLOCKSIZE,
            api->parameters_check(params));
  // Width and height is set and remain valid for the remaining tests.
  params->width = 176;
  params->height = 144;

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->internal_bitdepth = 17;
  EXPECT_EQ(XVC_ENC_BITDEPTH_OUT_OF_RANGE, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->internal_bitdepth = 10;
  if (sizeof(xvc::Sample) == 2) {
    EXPECT_EQ(XVC_ENC_OK, api->parameters_check(params));
  } else {
    EXPECT_EQ(XVC_ENC_COMPILED_BITDEPTH_TOO_LOW, api->parameters_check(params));
  }

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->framerate = 0.005;
  EXPECT_EQ(XVC_ENC_FRAMERATE_OUT_OF_RANGE, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->qp = 65;
  EXPECT_EQ(XVC_ENC_QP_OUT_OF_RANGE, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->sub_gop_length = 128;
  EXPECT_EQ(XVC_ENC_SUB_GOP_LENGTH_TOO_LARGE, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->sub_gop_length = 8;
  params->max_keypic_distance = 4;
  EXPECT_EQ(XVC_ENC_SUB_GOP_LENGTH_TOO_LARGE, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->closed_gop = -1;
  EXPECT_EQ(XVC_ENC_INVALID_PARAMETER, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->closed_gop = -1;
  EXPECT_EQ(XVC_ENC_INVALID_PARAMETER, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->max_keypic_distance = 0;
  EXPECT_EQ(XVC_ENC_OK, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->chroma_format = XVC_ENC_CHROMA_FORMAT_UNDEFINED;
  EXPECT_EQ(XVC_ENC_UNSUPPORTED_CHROMA_FORMAT, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->chroma_format = XVC_ENC_CHROMA_FORMAT_422;
  EXPECT_EQ(XVC_ENC_UNSUPPORTED_CHROMA_FORMAT, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->chroma_format = XVC_ENC_CHROMA_FORMAT_MONOCHROME;
  EXPECT_EQ(XVC_ENC_OK, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->chroma_format = XVC_ENC_CHROMA_FORMAT_420;
  EXPECT_EQ(XVC_ENC_OK, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->chroma_format = XVC_ENC_CHROMA_FORMAT_444;
  EXPECT_EQ(XVC_ENC_OK, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->num_ref_pics = 5;
  EXPECT_EQ(XVC_ENC_OK, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->num_ref_pics = 6;
  EXPECT_EQ(XVC_ENC_TOO_MANY_REF_PICS, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->restricted = 1;
  EXPECT_EQ(XVC_ENC_OK, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->restricted = -1;
  EXPECT_EQ(XVC_ENC_INVALID_PARAMETER, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->restricted = 2;
  EXPECT_EQ(XVC_ENC_INVALID_PARAMETER, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->deblock = 0;
  params->beta_offset = 2;
  EXPECT_EQ(XVC_ENC_DEBLOCKING_SETTINGS_INVALID, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->deblock = 0;
  params->tc_offset = -2;
  EXPECT_EQ(XVC_ENC_DEBLOCKING_SETTINGS_INVALID, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->beta_offset = -40;
  EXPECT_EQ(XVC_ENC_DEBLOCKING_SETTINGS_INVALID, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->tc_offset = 32;
  EXPECT_EQ(XVC_ENC_DEBLOCKING_SETTINGS_INVALID, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->beta_offset = 31;
  params->tc_offset = -32;
  EXPECT_EQ(XVC_ENC_OK, api->parameters_check(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  EXPECT_EQ(XVC_ENC_OK, api->parameters_check(params));
  EXPECT_EQ(XVC_ENC_OK, api->parameters_destroy(params));
}

TEST(EncoderAPI, EncoderCreate) {
  const xvc_encoder_api *api = xvc_encoder_api_get();
  xvc_encoder_parameters *params = api->parameters_create();

  EXPECT_EQ(nullptr, api->encoder_create(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  EXPECT_EQ(nullptr, api->encoder_create(params));

  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->width = 176;
  params->height = 144;
  xvc_encoder *encoder = api->encoder_create(params);
  EXPECT_EQ(XVC_ENC_OK, api->parameters_destroy(params));
  ASSERT_NE(encoder, nullptr);
  EXPECT_EQ(XVC_ENC_OK, api->encoder_destroy(encoder));
}

TEST(EncoderAPI, EncoderEncode) {
  const xvc_encoder_api *api = xvc_encoder_api_get();
  xvc_encoder_parameters *params = api->parameters_create();
  uint8_t pic[1] = { 1 };
  xvc_enc_nal_unit *nal_units;
  int num_nal_units;
  xvc_enc_pic_buffer pic_recon_buffer = { 0 };
  xvc_enc_pic_buffer *pic_recon_ptr = &pic_recon_buffer;
  EXPECT_EQ(XVC_ENC_INVALID_ARGUMENT,
            api->encoder_encode(nullptr, &pic[0], &nal_units, &num_nal_units,
                                pic_recon_ptr));
  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->width = 176;
  params->height = 144;
  xvc_encoder *encoder = api->encoder_create(params);
  EXPECT_EQ(XVC_ENC_OK, api->parameters_destroy(params));
  EXPECT_EQ(XVC_ENC_INVALID_ARGUMENT, api->encoder_encode(encoder, nullptr,
                                                          &nal_units,
                                                          &num_nal_units,
                                                          pic_recon_ptr));
  EXPECT_EQ(XVC_ENC_INVALID_ARGUMENT, api->encoder_encode(encoder, &pic[0],
                                                          nullptr,
                                                          &num_nal_units,
                                                          pic_recon_ptr));
  EXPECT_EQ(XVC_ENC_INVALID_ARGUMENT, api->encoder_encode(encoder, &pic[0],
                                                          &nal_units,
                                                          nullptr,
                                                          pic_recon_ptr));
  EXPECT_EQ(XVC_ENC_OK, api->encoder_destroy(encoder));
}


TEST(EncoderAPI, EncoderFlush) {
  const xvc_encoder_api *api = xvc_encoder_api_get();

  xvc_encoder_parameters *params = api->parameters_create();
  xvc_enc_nal_unit *nal_units;
  int num_nal_units;
  xvc_enc_pic_buffer pic_recon_buffer = { 0 };
  xvc_enc_pic_buffer *pic_recon_ptr = &pic_recon_buffer;
  EXPECT_EQ(XVC_ENC_INVALID_ARGUMENT,
            api->encoder_flush(nullptr, &nal_units, &num_nal_units,
                               pic_recon_ptr));
  EXPECT_EQ(XVC_ENC_OK, api->parameters_set_default(params));
  params->width = 176;
  params->height = 144;
  xvc_encoder *encoder = api->encoder_create(params);
  EXPECT_EQ(XVC_ENC_OK, api->parameters_destroy(params));
  EXPECT_EQ(XVC_ENC_INVALID_ARGUMENT, api->encoder_flush(encoder,
                                                         nullptr,
                                                         &num_nal_units,
                                                         pic_recon_ptr));
  EXPECT_EQ(XVC_ENC_INVALID_ARGUMENT, api->encoder_flush(encoder,
                                                         &nal_units,
                                                         nullptr,
                                                         pic_recon_ptr));
  EXPECT_EQ(XVC_ENC_OK, api->encoder_destroy(encoder));
}

}   // namespace
