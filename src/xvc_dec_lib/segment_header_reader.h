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
                             SegmentNum segment_counter);
};

}   // namespace xvc

#endif  // XVC_DEC_LIB_SEGMENT_HEADER_READER_H_
