/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_PICTURE_DATA_H_
#define XVC_COMMON_LIB_PICTURE_DATA_H_

#include <memory>
#include <vector>

#include "xvc_common_lib/picture_types.h"
#include "xvc_common_lib/reference_picture_lists.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

enum class OutputStatus {
  kHasNotBeenOutput,
  kHasBeenOutput,
};

class CodingUnit;

class PictureData {
public:
  PictureData(ChromaFormat chroma_format, int width, int height, int bitdepth);
  ~PictureData();

  void Init(const QP &pic_qp);

  // General
  PicturePredictionType GetPredictionType() const;
  const QP* GetPicQp() const { return pic_qp_.get(); }
  bool IsIntraPic() const {
    return GetPredictionType() == PicturePredictionType::kIntra;
  }

  // Sample info
  int GetPictureWidth(YuvComponent comp) const {
    return comp == kY ? pic_width_ : pic_width_ >> chroma_shift_x_;
  }
  int GetPictureHeight(YuvComponent comp) const {
    return comp == kY ? pic_height_ : pic_height_ >> chroma_shift_y_;
  }
  int GetBitdepth() const { return bitdepth_; }
  ChromaFormat GetChromaFormat() const { return chroma_fmt_; }
  int GetChromaShiftX() const { return chroma_shift_x_; }
  int GetChromaShiftY() const { return chroma_shift_y_; }
  int GetNumComponents() const {
    return util::GetNumComponents(chroma_fmt_);
  }

  // CU data
  CodingUnit *GetCtu(int rsaddr) { return ctu_list_[rsaddr]; }
  const CodingUnit *GetCtu(int rsaddr) const { return ctu_list_[rsaddr]; }
  CodingUnit* SetCtu(int rsaddr, CodingUnit *cu);
  int GetNumberOfCtu() const {
    return static_cast<int>(ctu_list_.size());
  }
  const CodingUnit *GetCuAt(int posx, int posy) const {
    return cu_pic_table_[(posy / constants::kMinBlockSize) * cu_pic_stride_ +
      (posx / constants::kMinBlockSize)];
  }
  CodingUnit *CreateCu(int depth, int posx, int posy, int width,
                       int height) const;
  void ReleaseCu(CodingUnit *cu) const;
  void MarkUsedInPic(CodingUnit *cu);
  void ClearMarkCuInPic(CodingUnit *cu);

  // High level syntax
  void SetNalType(NalUnitType type) { nal_type_ = type; }
  NalUnitType GetNalType() const { return nal_type_; }
  void SetOutputStatus(OutputStatus status) { output_status_ = status; }
  OutputStatus GetOutputStatus() { return output_status_; }
  void SetPoc(PicNum poc) { poc_ = poc; }
  PicNum GetPoc() const { return poc_; }
  void SetDoc(PicNum doc) { doc_ = doc; }
  PicNum GetDoc() const { return doc_; }
  void SetSoc(SegmentNum soc) { soc_ = soc; }
  SegmentNum GetSoc() const { return soc_; }
  void SetTid(int tid) { tid_ = tid; }
  int GetTid() const { return tid_; }

  // Per picture data
  ReferencePictureLists* GetRefPicLists() { return &ref_pic_lists_; }
  const ReferencePictureLists *GetRefPicLists() const {
    return &ref_pic_lists_;
  }
  bool GetTmvpValid() const { return tmvp_valid_; }
  RefPicList GetTmvpRefList() const { return tmvp_ref_list_; }
  int GetTmvpRefIdx() const { return tmvp_ref_idx_; }
  void SetDeblock(bool deblock) { deblock_ = deblock; }
  bool GetDeblock() const { return deblock_; }
  void SetBetaOffset(int offset) { beta_offset_ = offset; }
  int GetBetaOffset() const { return beta_offset_; }
  void SetTcOffset(int offset) { tc_offset_ = offset; }
  int GetTcOffset() const { return tc_offset_; }

  // Helper
  static bool IsSameDimension(const PictureData &pic1,
                              const PictureData &pic2) {
    const YuvComponent luma = YuvComponent::kY;
    return pic1.GetPictureWidth(luma) == pic2.GetPictureWidth(luma) &&
      pic1.GetPictureHeight(luma) == pic2.GetPictureHeight(luma) &&
      pic1.GetChromaFormat() == pic2.GetChromaFormat() &&
      pic1.GetBitdepth() == pic2.GetBitdepth();
  }

private:
  RefPicList DetermineTmvpRefList(int *tmvp_ref_idx);
  void ReleaseSubCuRecursively(CodingUnit *cu) const;

  std::vector<CodingUnit*> ctu_list_;
  std::vector<CodingUnit*> cu_pic_table_;
  ptrdiff_t cu_pic_stride_;
  int pic_width_;
  int pic_height_;
  int bitdepth_;
  ChromaFormat chroma_fmt_;
  int chroma_shift_x_;
  int chroma_shift_y_;
  int ctu_num_x_;
  int ctu_num_y_;
  PicNum poc_ = 0;
  PicNum doc_ = 0;
  SegmentNum soc_ = 0;
  int tid_ = 0;
  std::unique_ptr<QP> pic_qp_;
  NalUnitType nal_type_ = NalUnitType::kIntraPicture;
  OutputStatus output_status_ = OutputStatus::kHasNotBeenOutput;
  ReferencePictureLists ref_pic_lists_;
  bool tmvp_valid_ = false;
  RefPicList tmvp_ref_list_ = RefPicList::kTotalNumber;
  int tmvp_ref_idx_ = -1;
  bool deblock_ = true;
  int beta_offset_ = 0;
  int tc_offset_ = 0;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_PICTURE_DATA_H_
