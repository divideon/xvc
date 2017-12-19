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

namespace xvc_app {

class Y4mReader {
public:
  explicit Y4mReader(std::istream *ifs) : ifs_(ifs) {}
  bool Read(int &width, int &height, double &framerate,
            int &input_bitdepth, std::streamoff &start_skip,
            std::streamoff *picture_skip);
private:
  std::istream *ifs_;
};

}  // namespace xvc_app

#endif  // XVC_ENC_APP_Y4M_READER_H_
