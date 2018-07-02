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
