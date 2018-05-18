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

#ifndef XVC_ENC_APP_Y4M_READER_H_
#define XVC_ENC_APP_Y4M_READER_H_

#include <fstream>

#include "xvc_enc_lib/xvcenc.h"

namespace xvc_app {

struct PictureFormat {
  int width = 0;
  int height = 0;
  double framerate = 0;
  int input_bitdepth = 8;
  xvc_enc_chroma_format chroma_format = XVC_ENC_CHROMA_FORMAT_420;
};

class Y4mReader {
public:
  explicit Y4mReader(std::istream *ifs) : ifs_(ifs) {}
  bool Read(PictureFormat *pic_format, std::streamoff *start_skip,
            std::streamoff *picture_skip);
private:
  std::istream *ifs_;
};

}  // namespace xvc_app

#endif  // XVC_ENC_APP_Y4M_READER_H_
