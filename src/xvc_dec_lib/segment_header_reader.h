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

#ifndef XVC_DEC_LIB_SEGMENT_HEADER_READER_H_
#define XVC_DEC_LIB_SEGMENT_HEADER_READER_H_

#include "xvc_common_lib/segment_header.h"

#include "xvc_dec_lib/decoder.h"
#include "xvc_dec_lib/bit_reader.h"

namespace xvc {

class SegmentHeaderReader {
public:
  static Decoder::State Read(SegmentHeader* segment_header,
                             BitReader *bit_reader,
                             SegmentNum segment_counter,
                             bool* accept_xvc_bit_zero);
  static bool SupportedBitstreamVersion(uint32_t major_version,
                                        uint32_t minor_version);

private:
  static Restrictions ReadRestrictions(const SegmentHeader &segment_header,
                                       BitReader *bit_reader);
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_SEGMENT_HEADER_READER_H_
