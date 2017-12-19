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

#include "xvc_enc_app/y4m_reader.h"

#include <cassert>
#include <cstring>

namespace xvc_app {

bool Y4mReader::Read(int &width, int &height, double &framerate,
                     int &input_bitdepth, std::streamoff &start_skip,
                     std::streamoff *picture_skip) {
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
        width = static_cast<int>(strtol(buf + pos, &end, 10));
        pos = end - buf;
        break;
      case 'H':  // picture height
        height = static_cast<int>(strtol(buf + pos, &end, 10));
        pos = end - buf;
        break;
      case 'F':  // framerate
        den = static_cast<int>(strtol(buf + pos, &end, 10));
        pos = end - buf + 1;
        num = static_cast<int>(strtol(buf + pos, &end, 10));
        pos = end - buf;
        framerate = static_cast<double>(den) / num;
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
          input_bitdepth = 10;
          pos += 6;
        } else if (!strncmp(buf + pos, "420", 3)) {
          input_bitdepth = 8;
          pos += 3;
        } else {
          assert(0);
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

  start_skip = pos + 1;
  *picture_skip = 6;  // Skip "FRAME\n" before each picture
  return true;
}

}  // namespace xvc_app
