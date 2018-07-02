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

#ifndef XVC_ENC_LIB_SEGMENT_HEADER_WRITER_H_
#define XVC_ENC_LIB_SEGMENT_HEADER_WRITER_H_

#include "xvc_common_lib/segment_header.h"

#include "xvc_enc_lib/bit_writer.h"

namespace xvc {

class SegmentHeaderWriter {
public:
  static void Write(const SegmentHeader &segment_header, BitWriter *bit_writer,
                    double framerate);

private:
  static void WriteRestrictions(const Restrictions &restrictions,
                                BitWriter *bit_writer);
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_SEGMENT_HEADER_WRITER_H_
