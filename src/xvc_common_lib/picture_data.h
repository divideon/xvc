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
  bool HasSecondaryCuTree() { return num_cu_trees_ > 1; }
  int GetMaxNumComponents() const {
    return max_num_components_;
  }
  const std::vector<YuvComponent>& GetComponents(CuTree cu_tree) const {
    return cu_tree_components_[static_cast<int>(cu_tree)];
  }
  int GetMaxDepth(CuTree cu_tree) const {
    return cu_tree == CuTree::Primary ?
      constants::kMaxCuDepth : constants::kMaxCuDepthChroma;
  }
  int GetMaxBinarySplitDepth(CuTree cu_tree) const {
    return !IsIntraPic() ? constants::kMaxBinarySplitDepthInter :
      (cu_tree == CuTree::Primary ? constants::kMaxBinarySplitDepthIntra1 :
       constants::kMaxBinarySplitDepthIntra2);
  }
  int GetMaxBinarySplitSize(CuTree cu_tree) const {
    return !IsIntraPic() ? constants::kMaxBinarySplitSizeInter :
      (cu_tree == CuTree::Primary ? constants::kMaxBinarySplitSizeIntra1 :
       constants::kMaxBinarySplitSizeIntra2);
  }

  // CU data
  CodingUnit *GetCtu(CuTree cu_tree, int rsaddr) {
    return ctu_rs_list_[static_cast<int>(cu_tree)][rsaddr];
  }
  const CodingUnit *GetCtu(CuTree cu_tree, int rsaddr) const {
    return ctu_rs_list_[static_cast<int>(cu_tree)][rsaddr];
  }
  CodingUnit* SetCtu(CuTree cu_tree, int rsaddr, CodingUnit *cu);
  int GetNumberOfCtu() const {
    return static_cast<int>(ctu_rs_list_[0].size());
  }
  const CodingUnit* GetCuAt(CuTree cu_tree, int posx, int posy) const {
    ptrdiff_t cu_idx = (posy / constants::kMinBlockSize) * cu_pic_stride_ +
      (posx / constants::kMinBlockSize);
    return cu_pic_table_[static_cast<int>(cu_tree)][cu_idx];
  }
  const CodingUnit* GetLumaCu(const CodingUnit *cu) const;
  CodingUnit* CreateCu(CuTree cu_tree, int depth, int posx, int posy,
                       int width, int height) const;
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
  bool IsHighestLayer() const { return highest_layer_; }
  void SetHighestLayer(bool highest_layer) { highest_layer_ = highest_layer; }

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
  void AllocateAllCtu(CuTree cu_tree);
  void ReleaseSubCuRecursively(CodingUnit *cu) const;

  std::array<std::vector<CodingUnit*>,
    constants::kMaxNumCuTrees> ctu_rs_list_;
  std::array<std::vector<CodingUnit*>,
    constants::kMaxNumCuTrees> cu_pic_table_;
  std::array<std::vector<YuvComponent>,
    constants::kMaxNumCuTrees> cu_tree_components_;
  // Holds coefficients for a single ctu, then reused for next one
  mutable std::unique_ptr<CoeffCtuBuffer> ctu_coeff_;
  ptrdiff_t cu_pic_stride_;
  int pic_width_;
  int pic_height_;
  int bitdepth_;
  ChromaFormat chroma_fmt_;
  int max_num_components_;
  int chroma_shift_x_;
  int chroma_shift_y_;
  int num_cu_trees_;
  int ctu_num_x_;
  int ctu_num_y_;
  PicNum poc_ = 0;
  PicNum doc_ = 0;
  SegmentNum soc_ = 0;
  int tid_ = 0;
  bool highest_layer_ = false;
  std::unique_ptr<QP> pic_qp_;
  std::vector<QP> qps_;
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
