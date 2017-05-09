/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_app/y4m.h"

#include <cassert>
#include <cstring>

namespace xvc_app {

bool Y4M::ParseY4M(int &width, int &height, double &framerate,
                   int &input_bitdepth, std::streamoff &start_skip,
                   std::streamoff *picture_skip) {
  char buf[256];
  ifs_.read(buf, sizeof(buf) - 1);
  std::streamsize len = ifs_.gcount();
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
      case 'C':  // colour space
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

  assert(!strncmp(buf + pos, "\nFRAME\n", 7));
  start_skip = pos + 1;
  *picture_skip = 6;
  return true;
}

}  // namespace xvc_app
