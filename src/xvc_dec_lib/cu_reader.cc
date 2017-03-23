/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_dec_lib/cu_reader.h"

#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

void CuReader::ReadCu(CodingUnit *cu, SyntaxReader *reader) {
  bool split;
  if (cu->GetDepth() < constants::kMaxCuDepth) {
    if (cu->IsFullyWithinPicture()) {
      split = reader->ReadSplitFlag(*cu);
    } else {
      split = true;
    }
  } else {
    split = false;
  }
  if (split) {
    cu->SplitQuad();
    for (CodingUnit *sub_cu : cu->GetSubCu()) {
      if (sub_cu) {
        ReadCu(sub_cu, reader);
      }
    }
  } else {
    cu->SetSplit(false);
    pic_data_->MarkUsedInPic(cu);
    for (int c = 0; c < pic_data_->GetNumComponents(); c++) {
      const YuvComponent comp = YuvComponent(c);
      ReadComponent(cu, comp, reader);
    }
  }
}

void CuReader::ReadComponent(CodingUnit *cu, YuvComponent comp,
                             SyntaxReader *reader) {
  if (util::IsLuma(comp)) {
    if (!pic_data_->IsIntraPic()) {
      bool skip_flag = reader->ReadSkipFlag(*cu);
      cu->SetSkipFlag(skip_flag);
      if (skip_flag) {
        cu->SetPredMode(PredictionMode::kInter);
        cu->SetMergeFlag(true);
        int merge_idx = reader->ReadMergeIdx();
        cu->SetMergeIdx(merge_idx);
        cu->SetCbf(comp, false);
        return;
      }
      PredictionMode pred_mode = reader->ReadPredMode();
      cu->SetPredMode(pred_mode);
    } else {
      cu->SetPredMode(PredictionMode::kIntra);
      cu->SetSkipFlag(false);
    }
#if HM_STRICT
    PartitionType partition_type = reader->ReadPartitionType(*cu);
    cu->SetPartitionType(partition_type);
#endif
  } else if (cu->GetSkipFlag()) {
    cu->SetCbf(comp, false);
    return;
  }

  if (cu->IsIntra()) {
    ReadIntraPrediction(cu, comp, reader);
  } else {
    ReadInterPrediction(cu, comp, reader);
  }
  ReadCoefficients(cu, comp, reader);
}

void CuReader::ReadIntraPrediction(CodingUnit *cu, YuvComponent comp,
                                   SyntaxReader *reader) {
  if (util::IsLuma(comp)) {
    IntraPredictorLuma mpm = intra_pred_.GetPredictorLuma(*cu);
    IntraMode intra_mode = reader->ReadIntraMode(mpm);
    cu->SetIntraModeLuma(intra_mode);
  } else if (util::IsFirstChroma(comp)) {
    IntraMode luma_mode = cu->GetIntraMode(YuvComponent::kY);
    IntraPredictorChroma chroma_pred =
      intra_pred_.GetPredictorsChroma(luma_mode);
    IntraChromaMode chroma_mode = IntraChromaMode::kDMChroma;
    if (!Restrictions::Get().disable_intra_chroma_predictor) {
      chroma_mode = reader->ReadIntraChromaMode(chroma_pred);
    }
    cu->SetIntraModeChroma(chroma_mode);
  }
}

void CuReader::ReadInterPrediction(CodingUnit *cu, YuvComponent comp,
                                   SyntaxReader *reader) {
  if (util::IsLuma(comp)) {
    bool merge = reader->ReadMergeFlag();
    cu->SetMergeFlag(merge);
    if (merge) {
      int merge_idx = reader->ReadMergeIdx();
      cu->SetMergeIdx(merge_idx);
      return;
    }
    if (pic_data_->GetPredictionType() == PicturePredictionType::kBi) {
      InterDir inter_dir = reader->ReadInterDir(*cu);
      cu->SetInterDir(inter_dir);
    } else {
      cu->SetInterDir(InterDir::kL0);
    }
    for (int i = 0; i < static_cast<int>(RefPicList::kTotalNumber); i++) {
      RefPicList ref_pic_list = static_cast<RefPicList>(i);
      if (!ReferencePictureLists::IsRefPicListUsed(ref_pic_list,
                                                   cu->GetInterDir())) {
        continue;
      }
      int num_refs_available =
        pic_data_->GetRefPicLists()->GetNumRefPics(ref_pic_list);
      assert(num_refs_available > 0);
      cu->SetRefIdx(reader->ReadInterRefIdx(num_refs_available), ref_pic_list);
      cu->SetMvDelta(reader->ReadInterMvd(), ref_pic_list);
      cu->SetMvpIdx(reader->ReadInterMvpIdx(), ref_pic_list);
    }
  }
}

void CuReader::ReadCoefficients(CodingUnit *cu, YuvComponent comp,
                                SyntaxReader *reader) {
  bool signal_root_cbf = cu->IsInter() &&
    !Restrictions::Get().disable_transform_root_cbf &&
    (!cu->GetMergeFlag() || Restrictions::Get().disable_inter_skip_mode);
  if (signal_root_cbf) {
    if (util::IsLuma(comp)) {
      bool root_cbf = reader->ReadRootCbf();
      cu->SetRootCbf(root_cbf);
      if (!root_cbf) {
        if (cu->GetMergeFlag()) {
          cu->SetSkipFlag(true);
        }
        cu->SetCbf(YuvComponent::kY, false);
        cu->SetCbf(YuvComponent::kU, false);
        cu->SetCbf(YuvComponent::kV, false);
      }
    }
    if (!cu->GetRootCbf()) {
      return;
    }
  }

  bool cbf;
  if (Restrictions::Get().disable_transform_cbf) {
    cbf = true;
  } else if (cu->IsIntra()) {
    cbf = reader->ReadCbf(*cu, comp);
  } else if (util::IsLuma(comp)) {
    bool cbf_u = cbf = reader->ReadCbf(*cu, YuvComponent::kU);
    bool cbf_v = cbf = reader->ReadCbf(*cu, YuvComponent::kU);
    cu->SetCbf(YuvComponent::kU, cbf_u);
    cu->SetCbf(YuvComponent::kV, cbf_v);
    if (cbf_u || cbf_v || Restrictions::Get().disable_transform_root_cbf) {
      cbf = reader->ReadCbf(*cu, comp);
    } else {
      cbf = true;   // implicitly signaled through root cbf
    }
    if (Restrictions::Get().disable_inter_skip_mode &&
        cu->GetMergeFlag() && !cbf && !cbf_u && !cbf_v) {
      cu->SetSkipFlag(true);
    }
  } else {
    cbf = cu->GetCbf(comp);   // signaled from luma
  }
  cu->SetCbf(comp, cbf);
  Coeff *cu_coeff_buf = cu->GetCoeff(comp);
  ptrdiff_t cu_coeff_stride = cu->GetCoeffStride();
  // coefficient parsing is sparse so zero out in any case
  int width = cu->GetWidth(comp);
  int height = cu->GetHeight(comp);
  for (int y = 0; y < height; y++) {
    std::fill(cu_coeff_buf, cu_coeff_buf + width, 0);
    cu_coeff_buf += cu_coeff_stride;
  }
  if (cbf) {
    reader->ReadCoefficients(*cu, comp, cu->GetCoeff(comp), cu_coeff_stride);
  }
}

}   // namespace xvc


