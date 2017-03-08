/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
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
