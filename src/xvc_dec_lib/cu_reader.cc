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

#include "xvc_dec_lib/cu_reader.h"

#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

void CuReader::ReadCu(CodingUnit *cu, SplitRestriction split_restriction,
                      SyntaxReader *reader) {
  SplitType split = ReadSplit(cu, split_restriction, reader);
  if (split != SplitType::kNone) {
    cu->Split(split);
    SplitRestriction sub_split_restriction = SplitRestriction::kNone;
    for (CodingUnit *sub_cu : cu->GetSubCu()) {
      if (sub_cu) {
        sub_cu->SetQp(cu->GetQp().GetQpRaw(YuvComponent::kY));
        ReadCu(sub_cu, sub_split_restriction, reader);
        sub_split_restriction = sub_cu->DeriveSiblingSplitRestriction(split);
      }
    }
  } else {
    cu->SetSplit(SplitType::kNone);
    pic_data_->MarkUsedInPic(cu);
    for (YuvComponent comp : pic_data_->GetComponents(cu->GetCuTree())) {
      ReadComponent(cu, comp, reader);
    }
  }
}

SplitType
CuReader::ReadSplit(CodingUnit *cu, SplitRestriction split_restriction,
                    SyntaxReader *reader) {
  SplitType split = SplitType::kNone;
  int binary_depth = cu->GetBinaryDepth();
  int max_depth = pic_data_->GetMaxDepth(cu->GetCuTree());
  if (cu->GetDepth() < max_depth && binary_depth == 0) {
    if (cu->IsFullyWithinPicture()) {
      split = reader->ReadSplitQuad(*cu, max_depth);
    } else {
      split = SplitType::kQuad;
    }
  }
  if (split != SplitType::kQuad) {
    if (cu->IsBinarySplitValid()) {
      split = reader->ReadSplitBinary(*cu, split_restriction);
    }
  }
  return split;
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
    if (Restrictions::Get().disable_ext_implicit_partition_type) {
      PartitionType partition_type = reader->ReadPartitionType(*cu);
      cu->SetPartitionType(partition_type);
    }
  } else if (cu->GetSkipFlag()) {
    cu->SetCbf(comp, false);
    return;
  }

  if (cu->IsIntra()) {
    ReadIntraPrediction(cu, comp, reader);
  } else {
    ReadInterPrediction(cu, comp, reader);
  }
  ReadResidualData(cu, comp, reader);
}

void CuReader::ReadIntraPrediction(CodingUnit *cu, YuvComponent comp,
                                   SyntaxReader *reader) {
  if (util::IsLuma(comp)) {
    IntraPredictorLuma mpm = intra_pred_.GetPredictorLuma(*cu);
    IntraMode intra_mode = reader->ReadIntraMode(mpm);
    cu->SetIntraModeLuma(intra_mode);
  } else if (util::IsFirstChroma(comp)) {
    const CodingUnit *luma_cu = pic_data_->GetLumaCu(cu);
    IntraMode luma_mode = luma_cu->GetIntraMode(YuvComponent::kY);
    IntraPredictorChroma chroma_pred =
      intra_pred_.GetPredictorsChroma(luma_mode);
    IntraChromaMode chroma_mode = IntraChromaMode::kDmChroma;
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
      cu->SetMergeIdx(reader->ReadMergeIdx());
      return;
    }
    if (pic_data_->GetPredictionType() == PicturePredictionType::kBi) {
      cu->SetInterDir(reader->ReadInterDir(*cu));
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
      if (!cu->GetForceMvdZero(ref_pic_list)) {
        cu->SetMvDelta(reader->ReadInterMvd(), ref_pic_list);
      } else {
        cu->SetMvDelta(MotionVector(0, 0), ref_pic_list);
      }
      cu->SetMvpIdx(reader->ReadInterMvpIdx(), ref_pic_list);
    }
    if (!cu->HasZeroMvd()) {
      cu->SetFullpelMv(reader->ReadInterFullpelMvFlag(*cu));
    }
    if (pic_data_->GetUseLocalIlluminationCompensation()) {
      cu->SetUseLic(reader->ReadLicFlag());
    }
  }
}

void CuReader::ReadResidualData(CodingUnit *cu, YuvComponent comp,
                                SyntaxReader *reader) {
  bool cbf = ReadCbfInvariant(cu, comp, reader);
  CoeffBuffer cu_coeff_buf = cu->GetCoeff(comp);
  // coefficient parsing is sparse so zero out in any case
  cu_coeff_buf.ZeroOut(cu->GetWidth(comp), cu->GetHeight(comp));
  if (cbf) {
    ctu_has_coeffs_ = true;
    ReadResidualDataInternal(cu, comp, reader);
  }
}

void CuReader::ReadResidualDataInternal(CodingUnit *cu, YuvComponent comp,
                                        SyntaxReader *reader) const {
  CoeffBuffer cu_coeff_buf = cu->GetCoeff(comp);
  bool use_transform_select = false;
  if (util::IsLuma(comp)) {
    use_transform_select = reader->ReadTransformSelectEnable(*cu);
    if (!use_transform_select) {
      cu->SetTransformFromSelectIdx(comp, -1);
    }
  }
  bool transform_skip = reader->ReadTransformSkip(*cu, comp);
  cu->SetTransformSkip(comp, transform_skip);
  int num_coeff =
    reader->ReadCoefficients(*cu, comp, cu_coeff_buf.GetDataPtr(),
                             cu_coeff_buf.GetStride());
  if (util::IsLuma(comp) && use_transform_select) {
    int tx_select_idx = 0;
    if (!transform_skip &&
      (cu->IsInter() || num_coeff >= constants::kTransformSelectMinSigCoeffs)) {
      tx_select_idx = reader->ReadTransformSelectIdx(*cu);
    }
    cu->SetTransformFromSelectIdx(comp, tx_select_idx);
  }
}

bool CuReader::ReadCbfInvariant(CodingUnit *cu, YuvComponent comp,
                                SyntaxReader *reader) const {
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
        return false;
      }
    } else if (!cu->GetRootCbf()) {
      return false;
    }
  }

  bool cbf;
  if (Restrictions::Get().disable_transform_cbf) {
    cbf = true;
  } else if (cu->IsIntra()) {
    cbf = reader->ReadCbf(*cu, comp);
  } else if (util::IsLuma(comp)) {
    // luma will read cbf for all components
    bool cbf_u = cbf = reader->ReadCbf(*cu, YuvComponent::kU);
    bool cbf_v = cbf = reader->ReadCbf(*cu, YuvComponent::kU);
    cu->SetCbf(YuvComponent::kU, cbf_u);
    cu->SetCbf(YuvComponent::kV, cbf_v);
    if (cbf_u || cbf_v || Restrictions::Get().disable_transform_root_cbf) {
      cbf = reader->ReadCbf(*cu, comp);
    } else {
      // implicitly signaled through root cbf
      cbf = true;
    }
    if (Restrictions::Get().disable_inter_skip_mode &&
        cu->GetMergeFlag() && !cbf && !cbf_u && !cbf_v) {
      cu->SetSkipFlag(true);
    }
  } else {
    cbf = cu->GetCbf(comp);   // signaled from luma
  }
  cu->SetCbf(comp, cbf);
  return cbf;
}

}   // namespace xvc


