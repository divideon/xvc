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

#include "xvc_enc_app/y4m_reader.h"

#include <cassert>
#include <cstring>

namespace xvc_app {

bool Y4mReader::Read(PictureFormat *out_format, std::streamoff *start_skip,
                     std::streamoff *picture_skip) {
  PictureFormat pic_fmt;
  char buf[80];

  std::streamsize len;
  for (len = 0; len < sizeof(buf) - 1; len++) {
    int ret = ifs_->get();
    if (ret == std::istream::traits_type::eof()) {
      return false;
    }
    buf[len] = static_cast<char>(ret);
    if (buf[len] == '\n') {
      break;
    }
  }
  buf[len] = 0;

  if (strncmp(buf, "YUV4MPEG2 ", 10)) {
    return false;
  }

  std::streamoff pos = 10;
  int den, num;
  char *end;
  while (pos < len) {
    if (buf[pos] == '\n') {
      break;
    }
    if (buf[pos] == ' ') {
      pos++;
      continue;
    }
    switch (buf[pos++]) {
      case 'W':  // picture width
        pic_fmt.width = static_cast<int>(strtol(buf + pos, &end, 10));
        pos = end - buf;
        break;
      case 'H':  // picture height
        pic_fmt.height = static_cast<int>(strtol(buf + pos, &end, 10));
        pos = end - buf;
        break;
      case 'F':  // framerate
        den = static_cast<int>(strtol(buf + pos, &end, 10));
        pos = end - buf + 1;
        num = static_cast<int>(strtol(buf + pos, &end, 10));
        pos = end - buf;
        pic_fmt.framerate = static_cast<double>(den) / num;
        break;
      case 'I':  // interlacing
        assert(buf[pos] == 'p');
        pos++;
        break;
      case 'A':  // sample aspect ratio
        strtol(buf + pos, &end, 10);
        pos = end - buf + 1;
        strtol(buf + pos, &end, 10);
        pos = end - buf;
        break;
      case 'C':  // color space
        if (!strncmp(buf + pos, "420p10", 6)) {
          pic_fmt.input_bitdepth = 10;
          pic_fmt.chroma_format = XVC_ENC_CHROMA_FORMAT_420;
          pos += 6;
        } else if (!strncmp(buf + pos, "420", 3)) {
          pic_fmt.input_bitdepth = 8;
          pic_fmt.chroma_format = XVC_ENC_CHROMA_FORMAT_420;
          pos += 3;
        } else if (!strncmp(buf + pos, "422p10", 6)) {
          pic_fmt.input_bitdepth = 10;
          pic_fmt.chroma_format = XVC_ENC_CHROMA_FORMAT_422;
          pos += 6;
        } else if (!strncmp(buf + pos, "422", 3)) {
          pic_fmt.input_bitdepth = 8;
          pic_fmt.chroma_format = XVC_ENC_CHROMA_FORMAT_422;
          pos += 3;
        } else if (!strncmp(buf + pos, "444p10", 6)) {
          pic_fmt.input_bitdepth = 10;
          pic_fmt.chroma_format = XVC_ENC_CHROMA_FORMAT_444;
          pos += 6;
        } else if (!strncmp(buf + pos, "444", 3)) {
          pic_fmt.input_bitdepth = 8;
          pic_fmt.chroma_format = XVC_ENC_CHROMA_FORMAT_444;
          pos += 3;
        } else if (!strncmp(buf + pos, "mono", 4)) {
          pic_fmt.input_bitdepth = 8;
          pic_fmt.chroma_format = XVC_ENC_CHROMA_FORMAT_MONOCHROME;
          pos += 4;
        } else {
          pic_fmt.chroma_format = XVC_ENC_CHROMA_FORMAT_UNDEFINED;
        }
        break;
      case 'X':  // YUV4MPEG2 comment, ignored
      default:
        while (pos < len && buf[pos] != ' ' && buf[pos] != '\n') {
          pos++;
        }
        break;
    }
  }

  assert(pic_fmt.width != 0 && pic_fmt.height != 0);
  assert(pic_fmt.input_bitdepth != 0);
  assert(pic_fmt.chroma_format != XVC_ENC_CHROMA_FORMAT_UNDEFINED);
  if (out_format) {
    *out_format = pic_fmt;
  }
  if (start_skip) {
    *start_skip = pos + 1;
  }
  if (picture_skip) {
    *picture_skip = 6;  // Skip "FRAME\n" before each picture
  }
  return true;
}

}  // namespace xvc_app
