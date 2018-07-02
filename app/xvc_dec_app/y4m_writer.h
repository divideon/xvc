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
