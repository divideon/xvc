/******************************************************************************
* Copyright (C) 2018, Divideon.
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 2.1 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
* This library is also available under a commercial license.
* Please visit https://xvc.io/license/ for more information.
******************************************************************************/

#include "xvc_dec_lib/cu_reader.h"

#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

CuReader::CuReader(PictureData *pic_data, const IntraPrediction &intra_pred)
  : restrictions_(Restrictions::Get()),
  pic_data_(pic_data),
  intra_pred_(intra_pred) {
}

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
        ReadMergePrediction(cu, comp, reader);
        return;
      }
      PredictionMode pred_mode = reader->ReadPredMode();
      cu->SetPredMode(pred_mode);
    } else {
      cu->SetPredMode(PredictionMode::kIntra);
      cu->SetSkipFlag(false);
    }
    if (restrictions_.disable_ext_implicit_partition_type) {
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
    if (!restrictions_.disable_intra_chroma_predictor) {
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
      ReadMergePrediction(cu, comp, reader);
      return;
    }
    if (pic_data_->GetPredictionType() == PicturePredictionType::kBi) {
      cu->SetInterDir(reader->ReadInterDir(*cu));
    } else {
      cu->SetInterDir(InterDir::kL0);
    }
    if (cu->CanUseAffine()) {
      cu->SetUseAffine(reader->ReadAffineFlag(*cu, false));
    } else {
      cu->SetUseAffine(false);
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
      if (cu->GetForceMvdZero(ref_pic_list)) {
        cu->SetMvDelta(MvDelta(0, 0), ref_pic_list);
      } else if (cu->GetUseAffine()) {
        cu->SetMvdAffine(0, reader->ReadInterMvd(), ref_pic_list);
        cu->SetMvdAffine(1, reader->ReadInterMvd(), ref_pic_list);
      } else {
        cu->SetMvDelta(reader->ReadInterMvd(), ref_pic_list);
      }
      cu->SetMvpIdx(reader->ReadInterMvpIdx(*cu), ref_pic_list);
    }
    if (!cu->HasZeroMvd() &&
        !cu->GetUseAffine()) {
      cu->SetFullpelMv(reader->ReadInterFullpelMvFlag(*cu));
    }
    if (pic_data_->GetUseLocalIlluminationCompensation() &&
        !cu->GetUseAffine()) {
      cu->SetUseLic(reader->ReadLicFlag());
    }
  }
}

void CuReader::ReadMergePrediction(CodingUnit *cu, YuvComponent comp,
                                   SyntaxReader *reader) {
  if (cu->CanAffineMerge()) {
    cu->SetUseAffine(reader->ReadAffineFlag(*cu, true));
  }
  if (cu->GetUseAffine()) {
    cu->SetMergeIdx(0);
  } else {
    cu->SetMergeIdx(reader->ReadMergeIdx());
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
  cu->SetDcCoeffOnly(comp, num_coeff == 1 && *cu_coeff_buf.GetDataPtr());
}

bool CuReader::ReadCbfInvariant(CodingUnit *cu, YuvComponent comp,
                                SyntaxReader *reader) const {
  if (cu->IsInter() &&
    (!cu->GetMergeFlag() || restrictions_.disable_inter_skip_mode)) {
    if (util::IsLuma(comp)) {
      const bool root_cbf = reader->ReadRootCbf();
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
  if (cu->IsIntra()) {
    cbf = reader->ReadCbf(*cu, comp);
  } else if (util::IsLuma(comp)) {
    // luma will read cbf for all components
    bool cbf_u = reader->ReadCbf(*cu, YuvComponent::kU);
    bool cbf_v = reader->ReadCbf(*cu, YuvComponent::kU);
    cu->SetCbf(YuvComponent::kU, cbf_u);
    cu->SetCbf(YuvComponent::kV, cbf_v);
    if (cbf_u || cbf_v || restrictions_.disable_transform_root_cbf) {
      cbf = reader->ReadCbf(*cu, comp);
    } else {
      // implicitly signaled through root cbf
      cbf = true;
    }
    if (restrictions_.disable_inter_skip_mode &&
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


