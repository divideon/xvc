/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "googletest/include/gtest/gtest.h"

#include "xvc_dec_lib/xvcdec.h"

namespace {

TEST(DecoderAPI, NullPtrCalls) {
  const xvc_decoder_api *api = xvc_decoder_api_get();
  EXPECT_EQ(XVC_DEC_OK, api->parameters_destroy(nullptr));
  EXPECT_EQ(XVC_DEC_INVALID_ARGUMENT, api->parameters_set_default(nullptr));
  EXPECT_EQ(XVC_DEC_INVALID_ARGUMENT, api->parameters_check(nullptr));
  EXPECT_EQ(nullptr, api->decoder_create(nullptr));
  EXPECT_EQ(XVC_DEC_OK, api->decoder_destroy(nullptr));
}

TEST(DecoderAPI, ParamCheck) {
  const xvc_decoder_api *api = xvc_decoder_api_get();
  xvc_decoder_parameters *params = api->parameters_create();

  EXPECT_EQ(XVC_DEC_OK, api->parameters_set_default(params));
  params->output_width = 1;
  EXPECT_EQ(XVC_DEC_INVALID_PARAMETER, api->parameters_check(params));

  EXPECT_EQ(XVC_DEC_OK, api->parameters_set_default(params));
  params->output_height = -1;
  EXPECT_EQ(XVC_DEC_INVALID_PARAMETER, api->parameters_check(params));

  EXPECT_EQ(XVC_DEC_OK, api->parameters_set_default(params));
  params->output_chroma_format = static_cast<xvc_dec_chroma_format>(5);
  EXPECT_EQ(XVC_DEC_INVALID_PARAMETER, api->parameters_check(params));

  EXPECT_EQ(XVC_DEC_OK, api->parameters_set_default(params));
  params->output_bitdepth = 64;
  EXPECT_EQ(XVC_DEC_BITDEPTH_OUT_OF_RANGE, api->parameters_check(params));

  EXPECT_EQ(XVC_DEC_OK, api->parameters_set_default(params));
  params->max_framerate = 92000;
  EXPECT_EQ(XVC_DEC_FRAMERATE_OUT_OF_RANGE, api->parameters_check(params));

  EXPECT_EQ(XVC_DEC_OK, api->parameters_set_default(params));
  EXPECT_EQ(XVC_DEC_OK, api->parameters_check(params));
  EXPECT_EQ(XVC_DEC_OK, api->parameters_destroy(params));
}

TEST(DecoderAPI, DecoderCreate) {
  const xvc_decoder_api *api = xvc_decoder_api_get();
  xvc_decoder_parameters *params = api->parameters_create();
  EXPECT_EQ(nullptr, api->decoder_create(params));

  EXPECT_EQ(XVC_DEC_OK, api->parameters_set_default(params));
  xvc_decoder *decoder = api->decoder_create(params);
  EXPECT_EQ(XVC_DEC_OK, api->parameters_destroy(params));
  EXPECT_TRUE(decoder);
  EXPECT_EQ(XVC_DEC_OK, api->decoder_destroy(decoder));
}

TEST(DecoderAPI, DecoderDecodeNal) {
  const xvc_decoder_api *api = xvc_decoder_api_get();
  std::vector<uint8_t> nal_bytes_;
  nal_bytes_.push_back(0);
  size_t nal_size = 1;
  EXPECT_EQ(XVC_DEC_INVALID_ARGUMENT, api->decoder_decode_nal(nullptr,
                                                              &nal_bytes_[0],
                                                              nal_size));
  xvc_decoder_parameters *params = api->parameters_create();
  EXPECT_EQ(XVC_DEC_OK, api->parameters_set_default(params));
  xvc_decoder *decoder = api->decoder_create(params);
  EXPECT_EQ(XVC_DEC_OK, api->parameters_destroy(params));
  EXPECT_EQ(XVC_DEC_INVALID_ARGUMENT, api->decoder_decode_nal(decoder,
                                                              nullptr,
                                                              nal_size));
  EXPECT_EQ(XVC_DEC_INVALID_ARGUMENT, api->decoder_decode_nal(decoder,
                                                              &nal_bytes_[0],
                                                              0));
  EXPECT_EQ(XVC_DEC_OK, api->decoder_destroy(decoder));
}

TEST(DecoderAPI, DecoderGetDecodedPic) {
  const xvc_decoder_api *api = xvc_decoder_api_get();
  xvc_decoded_picture decoded_pic;
  EXPECT_EQ(XVC_DEC_INVALID_ARGUMENT, api->decoder_get_picture(nullptr,
                                                               &decoded_pic));
  xvc_decoder_parameters *params = api->parameters_create();
  EXPECT_EQ(XVC_DEC_OK, api->parameters_set_default(params));
  xvc_decoder *decoder = api->decoder_create(params);
  EXPECT_EQ(XVC_DEC_OK, api->parameters_destroy(params));
  EXPECT_EQ(XVC_DEC_INVALID_ARGUMENT, api->decoder_get_picture(decoder,
                                                               nullptr));
  EXPECT_EQ(XVC_DEC_NO_SEGMENT_HEADER_DECODED,
            api->decoder_get_picture(decoder, &decoded_pic));
  EXPECT_EQ(XVC_DEC_OK, api->decoder_destroy(decoder));
}

TEST(DecoderAPI, DecoderFlush) {
  const xvc_decoder_api *api = xvc_decoder_api_get();
  xvc_decoded_picture decoded_pic;
  EXPECT_EQ(XVC_DEC_INVALID_ARGUMENT, api->decoder_flush(nullptr,
                                                         &decoded_pic));
  xvc_decoder_parameters *params = api->parameters_create();
  EXPECT_EQ(XVC_DEC_OK, api->parameters_set_default(params));
  xvc_decoder *decoder = api->decoder_create(params);
  EXPECT_EQ(XVC_DEC_OK, api->parameters_destroy(params));
  EXPECT_EQ(XVC_DEC_INVALID_ARGUMENT, api->decoder_flush(decoder, nullptr));
  EXPECT_EQ(XVC_DEC_NO_DECODED_PIC, api->decoder_flush(decoder, &decoded_pic));
  EXPECT_EQ(XVC_DEC_OK, api->decoder_destroy(decoder));
}

TEST(DecoderAPI, DecoderCheckConformance) {
  const xvc_decoder_api *api = xvc_decoder_api_get();
  int num;
  EXPECT_EQ(XVC_DEC_INVALID_ARGUMENT,
            api->decoder_check_conformance(nullptr, &num));
  xvc_decoder_parameters *params = api->parameters_create();
  EXPECT_EQ(XVC_DEC_OK, api->parameters_set_default(params));
  xvc_decoder *decoder = api->decoder_create(params);
  EXPECT_EQ(XVC_DEC_OK, api->parameters_destroy(params));
  EXPECT_EQ(XVC_DEC_INVALID_ARGUMENT, api->decoder_check_conformance(decoder,
                                                                     nullptr));
  EXPECT_EQ(XVC_DEC_OK, api->decoder_check_conformance(decoder, &num));
  EXPECT_EQ(XVC_DEC_OK, api->decoder_destroy(decoder));
}

}   // namespace
