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
