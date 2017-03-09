/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_common_lib/picture_data.h"

#include <cassert>
#include <fstream>


#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/common.h"
#include "xvc_common_lib/resample.h"


namespace xvc {

PictureData::PictureData(ChromaFormat chroma_format, int width, int height,
                         int bitdepth)
  : rec_pic_(std::make_shared<YuvPicture>(chroma_format, width, height,
                                          bitdepth, true)),
  pic_width_(width),
  pic_height_(height),
  bitdepth_(bitdepth),
  chroma_fmt_(chroma_format),
  chroma_shift_x_(rec_pic_->GetSizeShiftX(YuvComponent::kU)),
  chroma_shift_y_(rec_pic_->GetSizeShiftY(YuvComponent::kU)),
  ctu_num_x_((pic_width_ + constants::kCtuSize - 1) /
             constants::kCtuSize),
  ctu_num_y_((pic_height_ + constants::kCtuSize - 1) /
             constants::kCtuSize) {
  int depth = 0;
  for (int y = 0; y < ctu_num_y_; y++) {
    for (int x = 0; x < ctu_num_x_; x++) {
      ctu_list_.push_back(new CodingUnit(*this, depth,
                                         x * constants::kCtuSize,
                                         y * constants::kCtuSize,
                                         constants::kCtuSize,
                                         constants::kCtuSize));
    }
  }
  int num_cu_pic_x = (pic_width_ + constants::kMaxBlockSize - 1) /
    constants::kMinBlockSize;
  int num_cu_pic_y = (pic_height_ + constants::kMaxBlockSize - 1) /
    constants::kMinBlockSize;
  cu_pic_stride_ = num_cu_pic_x + 1;
  cu_pic_table_.resize(cu_pic_stride_ * (num_cu_pic_y + 1));
  std::fill(cu_pic_table_.begin(), cu_pic_table_.end(), nullptr);
}

PictureData::~PictureData() {
  for (CodingUnit *ctu : ctu_list_) {
    ReleaseCu(ctu);
  }
}

void PictureData::Init(const QP &pic_qp) {
  pic_qp_.reset(new QP(pic_qp));
  std::fill(cu_pic_table_.begin(), cu_pic_table_.end(), nullptr);
  for (CodingUnit *ctu : ctu_list_) {
    ReleaseSubCuRecursively(ctu);
  }
  RefPicList tmvp_ref_list = DetermineTmvpRefList(&tmvp_ref_idx_);
  tmvp_ref_list_inv_ = ReferencePictureLists::Inverse(tmvp_ref_list);
  PicturePredictionType pic_type =
    ref_pic_lists_.GetRefPicType(tmvp_ref_list_inv_, tmvp_ref_idx_);
  tmvp_valid_ = pic_type == PicturePredictionType::kUni ||
    pic_type == PicturePredictionType::kBi;
}

int PictureData::DerivePictureQp(int segment_qp) const {
  if (GetPredictionType() == PicturePredictionType::kIntra) {
    return segment_qp;
  }
  return segment_qp + tid_ + 1;
}

std::shared_ptr<YuvPicture> PictureData::GetAlternativeRecPic(
  ChromaFormat chroma_format, int width, int height, int bitdepth) {
  if (alt_rec_pic_)
    return alt_rec_pic_;

  alt_rec_pic_ = std::make_shared<YuvPicture>(chroma_format, width, height,
                                              bitdepth, true);
  for (int c = 0; c < util::GetNumComponents(chroma_format); c++) {
    YuvComponent comp = YuvComponent(c);
    uint8_t* dst =
      reinterpret_cast<uint8_t*>(alt_rec_pic_->GetSamplePtr(comp, 0, 0));
    uint8_t* src =
      reinterpret_cast<uint8_t*>(rec_pic_->GetSamplePtr(comp, 0, 0));
    resample::Resample<Sample, Sample>
      (dst, alt_rec_pic_->GetWidth(comp), alt_rec_pic_->GetHeight(comp),
       alt_rec_pic_->GetStride(comp), alt_rec_pic_->GetBitdepth(),
       src, rec_pic_->GetWidth(comp), rec_pic_->GetHeight(comp),
       rec_pic_->GetStride(comp), rec_pic_->GetBitdepth());
  }

  alt_rec_pic_->PadBorder();
  return alt_rec_pic_;
}

CodingUnit* PictureData::SetCtu(int rsaddr, CodingUnit *cu) {
  if (ctu_list_[rsaddr] == cu) {
    return nullptr;
  }
  CodingUnit *old = ctu_list_[rsaddr];
  ctu_list_[rsaddr] = cu;
  return old;
}

CodingUnit *PictureData::CreateCu(int depth, int posx, int posy, int width,
                                  int height) const {
  if (posx >= pic_width_ || posy >= pic_height_) {
    return nullptr;
  }
  return new CodingUnit(*this, depth, posx, posy, width, height);
}

void PictureData::ReleaseCu(CodingUnit *cu) const {
  ReleaseSubCuRecursively(cu);
  delete cu;
}

void PictureData::MarkUsedInPic(CodingUnit *cu) {
  int index_x = cu->GetPosX(YuvComponent::kY) / constants::kMinBlockSize;
  int index_y = cu->GetPosY(YuvComponent::kY) / constants::kMinBlockSize;
  int num_x = cu->GetWidth(YuvComponent::kY) / constants::kMinBlockSize;
  int num_y = cu->GetHeight(YuvComponent::kY) / constants::kMinBlockSize;
  for (int y = 0; y < num_y; y++) {
    CodingUnit **ptr =
      &cu_pic_table_[(index_y + y) * cu_pic_stride_ + index_x];
    std::fill(ptr, ptr + num_x, cu);
  }
}

void PictureData::ClearMarkCuInPic(CodingUnit *cu) {
  int index_x = cu->GetPosX(YuvComponent::kY) / constants::kMinBlockSize;
  int index_y = cu->GetPosY(YuvComponent::kY) / constants::kMinBlockSize;
  int num_x = cu->GetWidth(YuvComponent::kY) / constants::kMinBlockSize;
  int num_y = cu->GetHeight(YuvComponent::kY) / constants::kMinBlockSize;
  for (int y = 0; y < num_y; y++) {
    CodingUnit **ptr =
      &cu_pic_table_[(index_y + y) * cu_pic_stride_ + index_x];
    std::fill(ptr, ptr + num_x, nullptr);
  }
}

PicturePredictionType PictureData::GetPredictionType() const {
  switch (nal_type_) {
    case NalUnitType::kIntraAccessPicture:
    case NalUnitType::kIntraPicture:
      return PicturePredictionType::kIntra;

    case NalUnitType::kPredictedAccessPicture:
    case NalUnitType::kPredictedPicture:
      return PicturePredictionType::kUni;

    case NalUnitType::kBipredictedAccessPicture:
    case NalUnitType::kBipredictedPicture:
      return PicturePredictionType::kBi;

    default:
      assert(0);
      return PicturePredictionType::kInvalid;
  }
}

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

static const int kMaxPicNumVal = xvc::constants::kTimeScale + 1;

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

PicNum PictureData::DocToPoc(const PicNum sub_gop_length, const PicNum doc) {
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

PicNum PictureData::PocToDoc(const PicNum sub_gop_length, const PicNum poc) {
  if (sub_gop_length <= 16) {
    return kPocToDoc[sub_gop_length][poc];
  } else if (sub_gop_length == 32) {
    return kPocToDoc32[poc];
  } else if (sub_gop_length == 64) {
    return kPocToDoc64[poc];
  } else if (poc == 0) {
    return 0;
  } else if (poc == sub_gop_length) {
    return 1;
  } else {
    return poc + 1;
  }
}

int PictureData::DocToTid(const PicNum sub_gop_length, const PicNum doc) {
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

void PictureData::CalcPocFromDoc(PicNum sub_gop_length,
                                 PicNum sub_gop_start_poc_) {
  if (doc_ < 1) {
    poc_ = 0;
    return;
  }
  PicNum doc_rem = ((doc_ - sub_gop_start_poc_ - 1) % sub_gop_length) + 1;
  poc_ = DocToPoc(sub_gop_length, doc_rem) + sub_gop_start_poc_;
}

void PictureData::CalcDocFromPoc(PicNum sub_gop_length,
                                 PicNum sub_gop_start_poc_) {
  if (poc_ < 1) {
    doc_ = 0;
    return;
  }
  PicNum poc_rem = ((poc_ - sub_gop_start_poc_ - 1) % sub_gop_length) + 1;
  doc_ = PocToDoc(sub_gop_length, poc_rem) + sub_gop_start_poc_;
}

void PictureData::CalcTidFromDoc(PicNum sub_gop_length,
                                 PicNum sub_gop_start_poc_) {
  if (doc_ < 1) {
    tid_ = 0;
    return;
  }
  PicNum doc_rem = ((doc_ - sub_gop_start_poc_ - 1) % sub_gop_length) + 1;
  tid_ = DocToTid(sub_gop_length, doc_rem);
}

int PictureData::GetMaxTid(int decoder_ticks, int bitstream_ticks,
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

double PictureData::GetFramerate(int max_tid, int bitstream_ticks,
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

RefPicList PictureData::DetermineTmvpRefList(int *tmvp_ref_idx) {
  const int ref_idx = 0;
  *tmvp_ref_idx = ref_idx;
  if (GetPredictionType() != PicturePredictionType::kBi) {
    return RefPicList::kL0;
  }
#if HM_STRICT
  int tid_l0 = ref_pic_lists_.GetRefPicTid(RefPicList::kL0, ref_idx);
  int tid_l1 = ref_pic_lists_.GetRefPicTid(RefPicList::kL1, ref_idx);
  return (tid_l0 <= tid_l1) ? RefPicList::kL0 : RefPicList::kL1;
#else
  if (ref_pic_lists_.GetRefPicType(RefPicList::kL0, 0) ==
      PicturePredictionType::kIntra) {
    return RefPicList::kL1;
  }
  if (ref_pic_lists_.GetRefPicType(RefPicList::kL1, 0) ==
      PicturePredictionType::kIntra) {
    return RefPicList::kL0;
  }
  const QP *qp_l0 = ref_pic_lists_.GetRefPicQp(RefPicList::kL0, ref_idx);
  const QP *qp_l1 = ref_pic_lists_.GetRefPicQp(RefPicList::kL1, ref_idx);
  return (*qp_l0 <= *qp_l1) ? RefPicList::kL0 : RefPicList::kL1;
#endif
}

void PictureData::ReleaseSubCuRecursively(CodingUnit *cu) const {
  for (CodingUnit *sub_cu : cu->GetSubCu()) {
    if (sub_cu) {
      ReleaseSubCuRecursively(sub_cu);
      delete sub_cu;
    }
  }
  cu->GetSubCu().fill(nullptr);
  cu->SetSplit(false);
}

}   // namespace xvc
