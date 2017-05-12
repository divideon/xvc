/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_DEC_APP_Y4M_WRITER_H_
#define XVC_DEC_APP_Y4M_WRITER_H_

#include <iostream>

#include "xvc_dec_lib/xvcdec.h"

namespace xvc_app {

class Y4mWriter {
public:
  void WriteHeader(const xvc_dec_pic_stats &stats, std::ostream *output);

private:
  void WriteGlobalHeader(const xvc_dec_pic_stats &stats, std::ostream *output);

  bool global_header_written_ = false;
};

}  // namespace xvc_app

#endif  // XVC_DEC_APP_Y4M_WRITER_H_
