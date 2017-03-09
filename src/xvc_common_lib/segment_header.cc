/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/segment_header.h"

namespace xvc {

static const int kMaxPicNumVal = xvc::constants::kTimeScale + 1;

static const PicNum kDocToPoc[17][17] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 3, 1, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 4, 2, 1, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 5, 1, 2, 3, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 6, 2, 4, 1, 3, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 7, 1, 2, 3, 4, 5, 6, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 8, 4, 2, 6, 1, 3, 5, 7, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 9, 1, 2, 3, 4, 5, 6, 7, 8, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 10, 2, 4, 6, 8, 1, 3, 5, 7, 9, 0, 0, 0, 0, 0, 0 },
  { 0, 11, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 0, 0, 0, 0 },
  { 0, 12, 4, 8, 2, 6, 10, 1, 3, 5, 7, 9, 11, 0, 0, 0, 0 },
  { 0, 13, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 0, 0, 0 },
  { 0, 14, 2, 4, 6, 8, 10, 12, 1, 3, 5, 7, 9, 11, 13, 0, 0 },
  { 0, 15, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0 },
  { 0, 16, 8, 4, 12, 2, 6, 10, 14, 1, 3, 5, 7, 9, 11, 13, 15 },
};

static const PicNum kPocToDoc[17][17] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 2, 3, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 3, 2, 4, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 2, 3, 4, 5, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 4, 2, 5, 3, 6, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 2, 3, 4, 5, 6, 7, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 5, 3, 6, 2, 7, 4, 8, 1, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 2, 3, 4, 5, 6, 7, 8, 9, 1, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 6, 2, 7, 3, 8, 4, 9, 5, 10, 1, 0, 0, 0, 0, 0, 0 },
  { 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 1, 0, 0, 0, 0, 0 },
  { 0, 7, 4, 8, 2, 9, 5, 10, 3, 11, 6, 12, 1, 0, 0, 0, 0 },
  { 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 1, 0, 0, 0 },
  { 0, 8, 2, 9, 3, 10, 4, 11, 5, 12, 6, 13, 7, 14, 1, 0, 0 },
  { 0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 1, 0 },
  { 0, 9, 5, 10, 3, 11, 6, 12, 2, 13, 7, 14, 4, 15, 8, 16, 1 },
};

static const int kDocToTid[17][17] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 1, 2, 2, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 2, 2, 3, 3, 3, 3, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 1, 1, 2, 2, 2, 3, 3, 3, 3, 3, 3, 0, 0, 0, 0 },
  { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0 },
  { 0, 0, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 0, 0 },
  { 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0 },
  { 0, 0, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4 },
};

static const PicNum kDocToPoc32[33] = { 0, 32, 16, 8, 24, 4, 12, 20, 28, 2,
6, 10, 14, 18, 22, 26, 30, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27,
29, 31 };

static const PicNum kPocToDoc32[33] = { 0, 17, 9, 18, 5, 19, 10, 20, 3, 21,
11, 22, 6, 23, 12, 24, 2, 25, 13, 26, 7, 27, 14, 28, 4, 29, 15, 30, 8, 31, 16,
32, 1 };

static const int kDocToTid32[33] = { 0, 0, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4,
4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5 };

static const PicNum kDocToPoc64[65] = { 0, 64, 32, 16, 48, 8, 24, 40, 56, 4,
12, 20, 28, 36, 44, 52, 60, 2, 6, 10, 14, 18, 22, 26, 30, 34, 38, 42, 46, 50,
54, 58, 62, 1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 25, 27, 29, 31, 33, 35,
37, 39, 41, 43, 45, 47, 49, 51, 53, 55, 57, 59, 61, 63 };

static const PicNum kPocToDoc64[65] = { 0, 33, 17, 34, 9, 35, 18, 36, 5, 37,
19, 38, 10, 39, 20, 40, 3, 41, 21, 42, 11, 43, 22, 44, 6, 45, 23, 46, 12, 47,
24, 48, 2, 49, 25, 50, 13, 51, 26, 52, 7, 53, 27, 54, 14, 55, 28, 56, 4, 57,
29, 58, 15, 59, 30, 60, 8, 61, 31, 62, 16, 63, 32, 64, 1 };

static const int kDocToTid64[65] = { 0, 0, 1, 2, 2, 3, 3, 3, 3, 4, 4, 4,
4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6,
6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6,
6 };

static const PicNum kPicsInSubbitstream[17][5] = {
  { 0, 0, 0, 0, 0 },
  { 1, kMaxPicNumVal, kMaxPicNumVal, kMaxPicNumVal, kMaxPicNumVal },
  { 1, 2, kMaxPicNumVal, kMaxPicNumVal, kMaxPicNumVal },
  { 1, 3, kMaxPicNumVal, kMaxPicNumVal, kMaxPicNumVal },
  { 1, 2, 4, kMaxPicNumVal, kMaxPicNumVal },
  { 1, 5, 5, kMaxPicNumVal, kMaxPicNumVal },
  { 1, 3, 6, kMaxPicNumVal, kMaxPicNumVal },
  { 1, 7, 7, kMaxPicNumVal, kMaxPicNumVal },
  { 1, 2, 4, 8, kMaxPicNumVal },
  { 1, 9, 9, 9, kMaxPicNumVal },
  { 1, 5, 10, 10, kMaxPicNumVal },
  { 1, 11, 11, 11, kMaxPicNumVal },
  { 1, 3, 6, 12, kMaxPicNumVal },
  { 1, 13, 13, 13, kMaxPicNumVal },
  { 1, 7, 14, 14, kMaxPicNumVal },
  { 1, 15, 15, 15, kMaxPicNumVal },
  { 1, 2, 4, 8, 16 },
};
static const PicNum kPicsInSubbitstream32[6] = { 1, 2, 4, 8, 16, 32 };
static const PicNum kPicsInSubbitstream64[7] = { 1, 2, 4, 8, 16, 32, 64 };

PicNum SegmentHeader::CalcPocFromDoc(PicNum doc, PicNum sub_gop_length,
                                     PicNum sub_gop_start_poc_) {
  if (doc < 1) {
    return 0;
  }
  PicNum doc_rem = ((doc - sub_gop_start_poc_ - 1) % sub_gop_length) + 1;
  return DocToPoc(sub_gop_length, doc_rem) + sub_gop_start_poc_;
}

PicNum SegmentHeader::CalcDocFromPoc(PicNum poc, PicNum sub_gop_length,
                                     PicNum sub_gop_start_poc_) {
  if (poc < 1) {
    return 0;
  }
  PicNum poc_rem = ((poc - sub_gop_start_poc_ - 1) % sub_gop_length) + 1;
  return PocToDoc(sub_gop_length, poc_rem) + sub_gop_start_poc_;
}

int SegmentHeader::CalcTidFromDoc(PicNum doc, PicNum sub_gop_length,
                                  PicNum sub_gop_start_poc_) {
  if (doc < 1) {
    return 0;
  }
  PicNum doc_rem = ((doc - sub_gop_start_poc_ - 1) % sub_gop_length) + 1;
  return DocToTid(sub_gop_length, doc_rem);
}

int SegmentHeader::GetMaxTid(int decoder_ticks, int bitstream_ticks,
                             PicNum sub_gop_length) {
  if (sub_gop_length <= 16) {
    for (int t = 4; t >= 0; t--) {
      if (kPicsInSubbitstream[sub_gop_length][t] * decoder_ticks
          <= sub_gop_length * bitstream_ticks) {
        return t;
      }
    }
  }
  if (sub_gop_length == 32) {
    for (int t = 5; t >= 0; t--) {
      if (kPicsInSubbitstream32[t] * decoder_ticks
          <= sub_gop_length * bitstream_ticks) {
        return t;
      }
    }
  }
  if (sub_gop_length == 64) {
    for (int t = 6; t >= 0; t--) {
      if (kPicsInSubbitstream64[t] * decoder_ticks
          <= sub_gop_length * bitstream_ticks) {
        return t;
      }
    }
  }
  if (decoder_ticks <= bitstream_ticks) {
    return 6;
  }

  return 0;
}

double SegmentHeader::GetFramerate(int max_tid, int bitstream_ticks,
                                   PicNum sub_gop_length) {
  if (bitstream_ticks == 0 || sub_gop_length == 0) {
    return 0;
  }
  if (sub_gop_length <= 16) {
    return (1.0 * kPicsInSubbitstream[sub_gop_length][max_tid] *
            constants::kTimeScale)
      / (sub_gop_length * bitstream_ticks);
  }
  if (sub_gop_length == 32) {
    return (1.0 * kPicsInSubbitstream32[max_tid] * constants::kTimeScale)
      / (sub_gop_length * bitstream_ticks);
  }
  if (sub_gop_length == 64) {
    return (1.0 * kPicsInSubbitstream64[max_tid] * constants::kTimeScale)
      / (sub_gop_length * bitstream_ticks);
  }
  if (max_tid == 0) {
    return (1.0 * constants::kTimeScale) / (sub_gop_length * bitstream_ticks);
  }
  return (1.0 * constants::kTimeScale) / (bitstream_ticks);
}

PicNum SegmentHeader::DocToPoc(PicNum sub_gop_length, PicNum doc) {
  if (sub_gop_length <= 16) {
    return kDocToPoc[sub_gop_length][doc];
  } else if (sub_gop_length == 32) {
    return kDocToPoc32[doc];
  } else if (sub_gop_length == 64) {
    return kDocToPoc64[doc];
  } else if (doc == 0) {
    return 0;
  } else if (doc == 1) {
    return sub_gop_length;
  } else {
    return doc - 1;
  }
}

PicNum SegmentHeader::PocToDoc(PicNum sub_gop_length, PicNum poc) {
  if (sub_gop_length <= 16) {
    return kPocToDoc[sub_gop_length][poc];
  } else if (sub_gop_length == 32) {
    return kPocToDoc32[poc];
  } else if (sub_gop_length == 64) {
    return kPocToDoc64[poc];
  } else if (poc == 0) {
    return 0;
  } else if (static_cast<int>(poc) == sub_gop_length) {
    return 1;
  } else {
    return poc + 1;
  }
}

int SegmentHeader::DocToTid(PicNum sub_gop_length, PicNum doc) {
  if (sub_gop_length <= 16) {
    return kDocToTid[sub_gop_length][doc];
  } else if (sub_gop_length == 32) {
    return kDocToTid32[doc];
  } else if (sub_gop_length == 64) {
    return kDocToTid64[doc];
  } else if (doc == 0) {
    return 0;
  } else if (doc == 1) {
    return 0;
  } else {
    return 1;
  }
}


}   // namespace xvc
