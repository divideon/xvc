/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
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
                             BitReader *bitreader,
                             SegmentNum segment_counter);
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_SEGMENT_HEADER_READER_H_
