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
#ifndef XVC_ENC_LIB_SEGMENT_HEADER_WRITER_H_
#define XVC_ENC_LIB_SEGMENT_HEADER_WRITER_H_

#include "xvc_common_lib/segment_header.h"

#include "xvc_enc_lib/bit_writer.h"

namespace xvc {

class SegmentHeaderWriter {
public:
  static void Write(SegmentHeader* segment_header,
                    BitWriter *bit_writer,
                    double framerate,
                    int open_gop);
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_SEGMENT_HEADER_WRITER_H_
