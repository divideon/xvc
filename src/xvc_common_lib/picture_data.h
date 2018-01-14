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

#ifndef XVC_COMMON_LIB_PICTURE_DATA_H_
#define XVC_COMMON_LIB_PICTURE_DATA_H_

#include <memory>
#include <vector>

#include "xvc_common_lib/picture_types.h"
#include "xvc_common_lib/reference_picture_lists.h"
#include "xvc_common_lib/segment_header.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

class CodingUnit;

class PictureData {
public:
  PictureData(ChromaFormat chroma_format, int width, int height, int bitdepth);
  ~PictureData();

  void Init(const SegmentHeader &segment, const Qp &pic_qp,
            bool recalculate_lambda);

  // General
  PicturePredictionType GetPredictionType() const;
  const Qp* GetPicQp() const {
    return pic_qp_.get();
  }
  const Qp* GetQp(int raw_qp) const {
    return &qps_[util::Clip3(raw_qp, 0, constants::kMaxAllowedQp)];
  }
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
    return max_binary_split_depth_;
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
  CodingUnit* GetCuAtForModification(CuTree cu_tree, int posx, int posy) {
    ptrdiff_t cu_idx = (posy / constants::kMinBlockSize) * cu_pic_stride_ +
      (posx / constants::kMinBlockSize);
    return cu_pic_table_[static_cast<int>(cu_tree)][cu_idx];
  }
  const CodingUnit* GetLumaCu(const CodingUnit *cu) const;
  CodingUnit* CreateCu(CuTree cu_tree, int depth, int posx, int posy,
                       int width, int height);
  void ReleaseCu(CodingUnit *cu);
  void MarkUsedInPic(CodingUnit *cu);
  void ClearMarkCuInPic(CodingUnit *cu);

  // High level syntax
  void SetNalType(NalUnitType type) { nal_type_ = type; }
  NalUnitType GetNalType() const { return nal_type_; }
  void SetPoc(PicNum poc) { poc_ = poc; }
  PicNum GetPoc() const { return poc_; }
  void SetDoc(PicNum doc) { doc_ = doc; }
  PicNum GetDoc() const { return doc_; }
  void SetSoc(SegmentNum soc) { soc_ = soc; }
  SegmentNum GetSoc() const { return soc_; }
  void SetTid(int tid) { tid_ = tid; }
  int GetTid() const { return tid_; }
  void SetSubGopLength(PicNum sub_gop_length) {
    sub_gop_length_ = sub_gop_length;
  }
  PicNum GetSubGopLength() const { return sub_gop_length_; }
  bool IsHighestLayer() const { return highest_layer_; }
  void SetHighestLayer(bool highest_layer) { highest_layer_ = highest_layer; }

  // Per picture data
  ReferencePictureLists* GetRefPicLists() { return &ref_pic_lists_; }
  const ReferencePictureLists *GetRefPicLists() const {
    return &ref_pic_lists_;
  }
  bool GetForceBipredL1MvdZero() const { return force_bipred_l1_mvd_zero_; }
  bool GetTmvpValid() const { return tmvp_valid_; }
  RefPicList GetTmvpRefList() const { return tmvp_ref_list_; }
  int GetTmvpRefIdx() const { return tmvp_ref_idx_; }
  void SetAdaptiveQp(bool adaptive_qp) { adaptive_qp_ = adaptive_qp; }
  bool GetAdaptiveQp() const { return adaptive_qp_; }
  void SetDeblock(bool deblock) { deblock_ = deblock; }
  bool GetDeblock() const { return deblock_; }
  void SetBetaOffset(int offset) { beta_offset_ = offset; }
  int GetBetaOffset() const { return beta_offset_; }
  void SetTcOffset(int offset) { tc_offset_ = offset; }
  int GetTcOffset() const { return tc_offset_; }
  bool GetUseLocalIlluminationCompensation() const { return lic_active_; }
  void SetUseLocalIlluminationCompensation(bool active) {
    lic_active_ = active;
  }

private:
  bool DetermineForceBipredL1MvdZero();
  RefPicList DetermineTmvpRefList(int *tmvp_ref_idx);
  void AllocateAllCtu(CuTree cu_tree);

  std::array<std::vector<CodingUnit*>,
    constants::kMaxNumCuTrees> ctu_rs_list_;
  std::array<std::vector<CodingUnit*>,
    constants::kMaxNumCuTrees> cu_pic_table_;
  std::array<std::vector<YuvComponent>,
    constants::kMaxNumCuTrees> cu_tree_components_;
  // Non owning pointers to CU objects that were preivously used in rdo
  std::vector<CodingUnit*> cu_alloc_free_list_;
  // Chunks of allocated memory, the inner arrays are static and never resized
  std::vector<std::vector<CodingUnit>> cu_alloc_buffers_;
  // Holds coefficients for a single ctu, then reused for next one
  std::unique_ptr<CoeffCtuBuffer> ctu_coeff_;
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
  int cu_alloc_batch_size_;
  size_t cu_alloc_list_index_ = 0;
  size_t cu_alloc_item_index_ = 0;
  PicNum poc_ = static_cast<PicNum>(-1);
  PicNum doc_ = static_cast<PicNum>(-1);
  SegmentNum soc_ = static_cast<SegmentNum>(-1);
  int tid_ = -1;
  PicNum sub_gop_length_ = 0;
  bool highest_layer_ = false;
  int max_binary_split_depth_ = 0;
  std::unique_ptr<Qp> pic_qp_;
  std::vector<Qp> qps_;
  NalUnitType nal_type_ = NalUnitType::kIntraPicture;
  ReferencePictureLists ref_pic_lists_;
  bool force_bipred_l1_mvd_zero_ = false;
  bool tmvp_valid_ = false;
  RefPicList tmvp_ref_list_ = RefPicList::kTotalNumber;
  int tmvp_ref_idx_ = -1;
  bool adaptive_qp_ = false;
  bool deblock_ = true;
  int beta_offset_ = 0;
  int tc_offset_ = 0;
  bool lic_active_ = false;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_PICTURE_DATA_H_
