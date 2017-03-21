/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/cu_writer.h"

#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

void CuWriter::WriteCu(const CodingUnit &cu, SyntaxWriter *writer) const {
  bool split = cu.IsSplit();
  if (cu.GetDepth() < constants::kMaxCuDepth) {
    if (cu.IsFullyWithinPicture()) {
      const CodingUnit *left = cu.GetCodingUnitLeft();
      const CodingUnit *above = cu.GetCodingUnitAbove();
      writer->WriteSplitFlag(cu.GetDepth(), left, above, split);
    } else {
      assert(split);
    }
  } else {
    assert(!split);
  }
  if (cu.IsSplit()) {
    for (int i = 0; i < constants::kQuadSplit; i++) {
      const CodingUnit *sub_cu = cu.GetSubCu(i);
      if (sub_cu) {
        WriteCu(*sub_cu, writer);
      }
    }
  } else {
    for (int c = 0; c < pic_data_.GetNumComponents(); c++) {
      const YuvComponent comp = YuvComponent(c);
      WriteComponent(cu, comp, writer);
    }
  }
}

void CuWriter::WriteComponent(const CodingUnit &cu, YuvComponent comp,
                              SyntaxWriter *writer) const {
  if (util::IsLuma(comp)) {
    if (!pic_data_.IsIntraPic()) {
      writer->WriteSkipFlag(cu, cu.GetSkipFlag());
      if (cu.GetSkipFlag() && !Restrictions::Get().disable_inter_skip_mode) {
        assert(cu.GetMergeIdx() >= 0);
        writer->WriteMergeIdx(cu.GetMergeIdx());
        return;
      }
      writer->WritePredMode(cu.GetPredMode());
    } else {
      assert(cu.GetPredMode() == PredictionMode::kIntra);
    }
#if HM_STRICT
    writer->WritePartitionType(cu, cu.GetPartitionType());
#endif
  } else if (cu.GetSkipFlag()) {
    return;
  }

  if (cu.IsIntra()) {
    WriteIntraPrediction(cu, comp, writer);
  } else {
    WriteInterPrediction(cu, comp, writer);
  }
  WriteCoefficients(cu, comp, writer);
}

void CuWriter::WriteIntraPrediction(const CodingUnit &cu, YuvComponent comp,
                                    SyntaxWriter *writer) const {
  IntraMode luma_mode = cu.GetIntraMode(YuvComponent::kY);
  if (util::IsLuma(comp)) {
    IntraPredictorLuma mpm = intra_pred_->GetPredictorLuma(cu);
    writer->WriteIntraMode(luma_mode, mpm);
  } else if (util::IsFirstChroma(comp)) {
    IntraPredictorChroma
      chroma_preds = intra_pred_->GetPredictorsChroma(luma_mode);
    if (!Restrictions::Get().disable_intra_chroma_predictor) {
      writer->WriteIntraChromaMode(cu.GetIntraChromaMode(), chroma_preds);
    }
  }
}

void CuWriter::WriteInterPrediction(const CodingUnit &cu, YuvComponent comp,
                                    SyntaxWriter *writer) const {
  if (util::IsLuma(comp)) {
    writer->WriteMergeFlag(cu.GetMergeFlag());
    if (cu.GetMergeFlag()) {
      assert(cu.GetHasAnyCbf() || Restrictions::Get().disable_inter_skip_mode);
      writer->WriteMergeIdx(cu.GetMergeIdx());
      return;
    }
    if (pic_data_.GetPredictionType() == PicturePredictionType::kBi) {
      writer->WriteInterDir(cu, cu.GetInterDir());
    } else {
      assert(cu.GetInterDir() == InterDir::kL0);
    }
    for (int i = 0; i < static_cast<int>(RefPicList::kTotalNumber); i++) {
      RefPicList ref_pic_list = static_cast<RefPicList>(i);
      if (!ReferencePictureLists::IsRefPicListUsed(ref_pic_list,
                                                   cu.GetInterDir())) {
        continue;
      }
      int num_refs_available =
        pic_data_.GetRefPicLists()->GetNumRefPics(ref_pic_list);
      assert(num_refs_available > 0);
      writer->WriteInterRefIdx(cu.GetRefIdx(ref_pic_list), num_refs_available);
      writer->WriteInterMvd(cu.GetMvDelta(ref_pic_list));
      writer->WriteInterMvpIdx(cu.GetMvpIdx(ref_pic_list));
    }
  }
}

void CuWriter::WriteCoefficients(const CodingUnit &cu, YuvComponent comp,
                                 SyntaxWriter *writer) const {
  bool signal_root_cbf = cu.IsInter() &&
    !Restrictions::Get().disable_transform_root_cbf &&
    (!cu.GetMergeFlag() || Restrictions::Get().disable_inter_skip_mode);
  if (signal_root_cbf) {
    bool root_cbf = cu.GetRootCbf();
    if (util::IsLuma(comp)) {
      writer->WriteRootCbf(root_cbf);
    }
    if (!root_cbf) {
      return;
    }
  }

  bool cbf = cu.GetCbf(comp);
  if (Restrictions::Get().disable_transform_cbf) {
    assert(cbf);
  } else if (cu.IsIntra()) {
    writer->WriteCbf(cu, comp, cbf);
  } else if (util::IsLuma(comp)) {
    // For inter the luma comp will write all cbf flags
    writer->WriteCbf(cu, YuvComponent::kU, cu.GetCbf(YuvComponent::kU));
    writer->WriteCbf(cu, YuvComponent::kV, cu.GetCbf(YuvComponent::kV));
    if (cu.GetCbf(YuvComponent::kU) || cu.GetCbf(YuvComponent::kV) ||
        Restrictions::Get().disable_transform_root_cbf) {
      writer->WriteCbf(cu, comp, cbf);
    } else {
      assert(cbf);  // implicit signaling through root cbf
    }
  } else {
    // signaled by luma
  }
  if (cbf) {
    const Coeff *cu_coeff_buf = cu.GetCoeff(comp);
    ptrdiff_t cu_coeff_stride = cu.GetCoeffStride();
    writer->WriteCoefficients(cu, comp, cu_coeff_buf, cu_coeff_stride);
  }
}

}   // namespace xvc
