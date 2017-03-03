/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_ENC_APP_Y4M_H_
#define XVC_ENC_APP_Y4M_H_

#include <fstream>

namespace xvc_app {

class Y4M {
public:
  explicit Y4M(std::ifstream &ifs) : ifs_(ifs) {}
  bool ParseY4M(int &width, int &height, double &framerate,
                int &input_bitdepth, std::streamoff &start_skip,
                std::streamoff *picture_skip);
private:
  std::ifstream &ifs_;
};

}  // namespace xvc_app

#endif  // XVC_ENC_APP_Y4M_H_
