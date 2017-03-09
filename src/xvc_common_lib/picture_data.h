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
#include "xvc_common_lib/yuv_pic.h"

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

  std::shared_ptr<YuvPicture> GetRecPic() { return rec_pic_; }
  const QP* GetPicQp() const { return pic_qp_.get(); }
  int DerivePictureQp(int segment_qp) const;

  std::shared_ptr<YuvPicture> GetAlternativeRecPic(
    ChromaFormat chroma_format, int width, int height, int bitdepth);

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

  void SetOutputStatus(OutputStatus status) { output_status_ = status; }
  OutputStatus GetOutputStatus() { return output_status_; }

  void SetPicType(NalUnitType type) { nal_type_ = type; }
  NalUnitType GetPicType() const { return nal_type_; }
  PicturePredictionType GetPredictionType() const;
  bool IsIntraPic() const {
    return GetPredictionType() == PicturePredictionType::kIntra;
  }

  void SetPoc(PicNum poc) { poc_ = poc; }
  PicNum GetPoc() const { return poc_; }
  void SetDoc(PicNum doc) { doc_ = doc; }
  PicNum GetDoc() const { return doc_; }
  void SetSoc(SegmentNum soc) { soc_ = soc; }
  SegmentNum GetSoc() const { return soc_; }
  void SetTid(int tid) { tid_ = tid; }
  int GetTid() const { return tid_; }

  void CalcDocFromPoc(PicNum sub_gop_length, PicNum sub_gop_start_poc_);
  void CalcTidFromDoc(PicNum sub_gop_length, PicNum sub_gop_start_poc_);
  void CalcPocFromDoc(PicNum sub_gop_length, PicNum sub_gop_start_poc_);
  static int GetMaxTid(int decoder_ticks, int bitstream_ticks,
                       PicNum sub_gop_length);
  static double GetFramerate(int max_tid, int bitstream_ticks,
                             PicNum sub_gop_length);
  ReferencePictureLists* GetRefPicLists() { return &ref_pic_lists_; }
  const ReferencePictureLists *GetRefPicLists() const {
    return &ref_pic_lists_;
  }
  bool GetTmvpValid() const { return tmvp_valid_; }
  RefPicList GetTmvpRefList() const { return tmvp_ref_list_inv_; }
  int GetTmvpRefIdx() const { return tmvp_ref_idx_; }
  void SetDeblock(bool deblock) { deblock_ = deblock; }
  bool GetDeblock() const { return deblock_; }
  void SetBetaOffset(int offset) { beta_offset_ = offset; }
  int GetBetaOffset() const { return beta_offset_; }
  void SetTcOffset(int offset) { tc_offset_ = offset; }
  int GetTcOffset() const { return tc_offset_; }

private:
  static PicNum DocToPoc(const PicNum sub_gop_length, const PicNum doc);
  static PicNum PocToDoc(const PicNum sub_gop_length, const PicNum poc);
  static int DocToTid(const PicNum sub_gop_length, const PicNum doc);

  RefPicList DetermineTmvpRefList(int *tmvp_ref_idx);
  void ReleaseSubCuRecursively(CodingUnit *cu) const;

  std::shared_ptr<YuvPicture> rec_pic_;
  std::shared_ptr<YuvPicture> alt_rec_pic_;
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
  RefPicList tmvp_ref_list_inv_ = RefPicList::kTotalNumber;
  int tmvp_ref_idx_ = -1;
  bool deblock_ = true;
  int beta_offset_ = 0;
  int tc_offset_ = 0;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_PICTURE_DATA_H_
