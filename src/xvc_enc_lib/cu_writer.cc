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

#include "xvc_enc_lib/cu_writer.h"

#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

void CuWriter::WriteCu(const CodingUnit &cu, SplitRestriction split_restriction,
                       SyntaxWriter *writer) {
  WriteSplit(cu, split_restriction, writer);
  if (cu.GetSplit() != SplitType::kNone) {
    SplitRestriction sub_split_restriction = SplitRestriction::kNone;
    for (int i = 0; i < constants::kQuadSplit; i++) {
      const CodingUnit *sub_cu = cu.GetSubCu(i);
      if (sub_cu) {
        WriteCu(*sub_cu, sub_split_restriction, writer);
        sub_split_restriction =
          sub_cu->DeriveSiblingSplitRestriction(cu.GetSplit());
      }
    }
  } else {
    for (YuvComponent comp : pic_data_.GetComponents(cu.GetCuTree())) {
      WriteComponent(cu, comp, writer);
    }
  }
}

void CuWriter::WriteSplit(const CodingUnit &cu,
                          SplitRestriction split_restriction,
                          SyntaxWriter *writer) {
  SplitType split_type = cu.GetSplit();
  int binary_depth = cu.GetBinaryDepth();
  int max_depth = pic_data_.GetMaxDepth(cu.GetCuTree());
  if (cu.GetDepth() < max_depth && binary_depth == 0) {
    if (cu.IsFullyWithinPicture()) {
      writer->WriteSplitQuad(cu, max_depth, split_type);
    } else {
      assert(split_type != SplitType::kNone);
    }
  }
  if (split_type != SplitType::kQuad) {
    if (cu.IsBinarySplitValid()) {
      writer->WriteSplitBinary(cu, split_restriction, split_type);
    }
  }
}

void CuWriter::WriteComponent(const CodingUnit &cu, YuvComponent comp,
                              SyntaxWriter *writer) {
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
    if (Restrictions::Get().disable_ext_implicit_partition_type) {
      writer->WritePartitionType(cu, cu.GetPartitionType());
    }
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
                                    SyntaxWriter *writer) {
  const CodingUnit *luma_cu = pic_data_.GetLumaCu(&cu);
  IntraMode luma_mode = luma_cu->GetIntraMode(YuvComponent::kY);
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
                                    SyntaxWriter *writer) {
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
                                 SyntaxWriter *writer) {
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
    ctu_has_coeffs_ = true;
    DataBuffer<const Coeff> cu_coeff = cu.GetCoeff(comp);
    writer->WriteCoefficients(cu, comp, cu_coeff.GetDataPtr(),
                              cu_coeff.GetStride());
  }
}

}   // namespace xvc
