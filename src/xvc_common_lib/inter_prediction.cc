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

#include "xvc_common_lib/inter_prediction.h"

#include <algorithm>

#include "xvc_common_lib/utils.h"
#include "xvc_common_lib/reference_picture_lists.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/simd_cpu.h"

namespace xvc {

const std::array<std::array<int16_t, InterPrediction::kNumTapsLuma>, 4>
InterPrediction::kLumaFilter = { {
  { 0, 0, 0, 64, 0, 0, 0, 0 },
  { -1, 4, -10, 58, 17, -5, 1, 0 },
  { -1, 4, -11, 40, 40, -11, 4, -1 },
  { 0, 1, -5, 17, 58, -10, 4, -1 }
} };

const std::array<std::array<int16_t, InterPrediction::kNumTapsChroma>, 8>
InterPrediction::kChromaFilter = { {
  { 0, 64, 0, 0 },
  { -2, 58, 10, -2 },
  { -4, 54, 16, -2 },
  { -6, 46, 28, -4 },
  { -4, 36, 36, -4 },
  { -4, 28, 46, -6 },
  { -2, 16, 54, -4 },
  { -2, 10, 58, -2 }
} };

const std::array<std::array<uint8_t, 2>, 12>
InterPrediction::kMergeCandL0L1Idx = { {
  { 0, 1 }, { 1, 0 }, { 0, 2 }, { 2, 0 }, { 1, 2 }, { 2, 1 },
  { 0, 3 }, { 3, 0 }, { 1, 3 }, { 3, 1 }, { 2, 3 }, { 3, 2 },
} };

InterPredictorList
InterPrediction::GetMvPredictors(const CodingUnit &cu, RefPicList ref_list,
                                 int ref_idx) {
  InterPredictorList list;
  if (Restrictions::Get().disable_inter_mvp) {
    const CodingUnit *tmp = cu.GetCodingUnitLeft();
    if (tmp && tmp->IsInter()) {
      list[0] = tmp->GetMv(ref_list);
      list[1] = tmp->GetMv(ref_list);
    } else {
      tmp = cu.GetCodingUnitAbove();
      if (tmp && tmp->IsInter()) {
        list[0] = tmp->GetMv(ref_list);
        list[1] = tmp->GetMv(ref_list);
      } else {
        list[0] = MotionVector(0, 0);
        list[1] = MotionVector(0, 0);
      }
    }
    return list;
  }
  PicNum ref_poc = cu.GetRefPicLists()->GetRefPoc(ref_list, ref_idx);
  int i = 0;

  const CodingUnit *tmp = cu.GetCodingUnitLeftBelow();
  if (!tmp || !tmp->IsInter()) {
    tmp = cu.GetCodingUnitLeftCorner();
  }
  bool smvp_added = tmp && tmp->IsInter();

  // Left
  if (GetMvpCand(cu.GetCodingUnitLeftBelow(),
                 ref_list, ref_idx, ref_poc, &list[i])) {
    i++;
  } else if (GetMvpCand(cu.GetCodingUnitLeftCorner(),
                        ref_list, ref_idx, ref_poc, &list[i])) {
    i++;
  } else if (GetScaledMvpCand(cu.GetCodingUnitLeftBelow(),
                              ref_list, ref_idx, &list[i])) {
    i++;
  } else if (GetScaledMvpCand(cu.GetCodingUnitLeftCorner(),
                              ref_list, ref_idx, &list[i])) {
    i++;
  }

  // Above
  if (GetMvpCand(cu.GetCodingUnitAboveRight(),
                 ref_list, ref_idx, ref_poc, &list[i])) {
    i++;
  } else if (GetMvpCand(cu.GetCodingUnitAboveCorner(),
                        ref_list, ref_idx, ref_poc, &list[i])) {
    i++;
  } else if (GetMvpCand(cu.GetCodingUnitAboveLeft(),
                        ref_list, ref_idx, ref_poc, &list[i])) {
    i++;
  }
  if (!smvp_added) {
    if (GetScaledMvpCand(cu.GetCodingUnitAboveRight(),
                         ref_list, ref_idx, &list[i])) {
      i++;
    } else if (GetScaledMvpCand(cu.GetCodingUnitAboveCorner(),
                                ref_list, ref_idx, &list[i])) {
      i++;
    } else if (GetScaledMvpCand(cu.GetCodingUnitAboveLeft(),
                                ref_list, ref_idx, &list[i])) {
      i++;
    }
  }

  if (i == 2 && list[0] == list[1]) {
    i = 1;
  }

  if (constants::kTemporalMvPrediction && cu.GetPicData()->GetTmvpValid() &&
      !Restrictions::Get().disable_inter_tmvp_mvp && i < 2) {
    if (GetTemporalMvPredictor(cu, ref_list, ref_idx, &list[i])) {
      i++;
    }
  }

  if (i == 0) {
    list[i++] = MotionVector(0, 0);
  }
  if (i == 1) {
    list[i++] = MotionVector(0, 0);
  }
  return list;
}

static bool HasDifferentMotion(const CodingUnit &cu1, const CodingUnit &cu2) {
  if (cu1.GetInterDir() != cu2.GetInterDir()) {
    return true;
  }
  for (int i = 0; i < static_cast<int>(RefPicList::kTotalNumber); i++) {
    RefPicList ref_list = RefPicList(i);
    if (!cu1.HasMv(ref_list)) {
      continue;
    }
    if (cu1.GetRefIdx(ref_list) != cu2.GetRefIdx(ref_list) ||
        cu1.GetMv(ref_list) != cu2.GetMv(ref_list)) {
      return true;
    }
  }
  return false;
}

InterMergeCandidateList
InterPrediction::GetMergeCandidates(const CodingUnit &cu, int merge_cand_idx) {
  const bool pic_bipred = cu.GetPicType() == PicturePredictionType::kBi;
  const int kL0 = static_cast<int>(RefPicList::kL0);
  const int kL1 = static_cast<int>(RefPicList::kL1);
  InterMergeCandidateList list;
  int num = 0;

  const CodingUnit *left = cu.GetCodingUnitLeftCorner();
  bool has_a1 = left && left->IsInter();
  if (has_a1) {
    list[num] = GetMergeCandidateFromCu(*left);
    if (num++ == merge_cand_idx) {
      return list;
    }
  }

  const CodingUnit *above = cu.GetCodingUnitAboveCorner();
  bool has_b1 = above && above->IsInter();
  if (has_b1 && (!has_a1 || HasDifferentMotion(*left, *above))) {
    list[num] = GetMergeCandidateFromCu(*above);
    if (num++ == merge_cand_idx) {
      return list;
    }
  }

  const CodingUnit *above_right = cu.GetCodingUnitAboveRight();
  bool has_b0 = above_right && above_right->IsInter();
  if (has_b0 && (!has_b1 || HasDifferentMotion(*above, *above_right))) {
    list[num] = GetMergeCandidateFromCu(*above_right);
    if (num++ == merge_cand_idx) {
      return list;
    }
  }

  const CodingUnit *left_below = cu.GetCodingUnitLeftBelow();
  bool has_a0 = left_below && left_below->IsInter();
  if (has_a0 && (!has_a1 || HasDifferentMotion(*left, *left_below))) {
    list[num] = GetMergeCandidateFromCu(*left_below);
    if (num++ == merge_cand_idx) {
      return list;
    }
  }

  const CodingUnit *above_left = cu.GetCodingUnitAboveLeft();
  bool has_b2 = above_left && above_left->IsInter();
  if (has_b2 && num < 4
      && (!has_a1 || HasDifferentMotion(*left, *above_left))
      && (!has_b1 || HasDifferentMotion(*above, *above_left))) {
    list[num] = GetMergeCandidateFromCu(*above_left);
    if (num++ == merge_cand_idx) {
      return list;
    }
  }

  if (constants::kTemporalMvPrediction && num < static_cast<int>(list.size()) &&
      !Restrictions::Get().disable_inter_tmvp_merge &&
      cu.GetPicData()->GetTmvpValid()) {
    bool found_any =
      GetTemporalMvPredictor(cu, RefPicList::kL0, 0, &list[num].mv[0]);
    list[num].ref_idx[0] = 0;
    list[num].inter_dir = InterDir::kL0;
    if (pic_bipred &&
        GetTemporalMvPredictor(cu, RefPicList::kL1, 0, &list[num].mv[1])) {
      list[num].ref_idx[1] = 0;
      list[num].inter_dir = found_any ? InterDir::kBi : InterDir::kL1;
      found_any = true;
    }
    if (found_any) {
      if (num++ == merge_cand_idx) {
        return list;
      }
    }
  }

  if (pic_bipred && !Restrictions::Get().disable_inter_merge_bipred) {
    auto ref_pic_lists = cu.GetRefPicLists();
    int max_num_bi_cand = num * (num - 1);
    for (int i = 0; i < max_num_bi_cand &&
         num < static_cast<int>(list.size()); i++) {
      int cand_l0_idx = kMergeCandL0L1Idx[i][0];
      int cand_l1_idx = kMergeCandL0L1Idx[i][1];
      if (list[cand_l0_idx].inter_dir == InterDir::kL1 ||
          list[cand_l1_idx].inter_dir == InterDir::kL0) {
        continue;
      }
      PicNum poc_l0 =
        ref_pic_lists->GetRefPoc(RefPicList::kL0, list[cand_l0_idx].ref_idx[0]);
      PicNum poc_l1 =
        ref_pic_lists->GetRefPoc(RefPicList::kL1, list[cand_l1_idx].ref_idx[1]);
      if (poc_l0 != poc_l1 ||
          list[cand_l0_idx].mv[0] != list[cand_l1_idx].mv[1]) {
        list[num].inter_dir = InterDir::kBi;
        list[num].mv[0] = list[cand_l0_idx].mv[0];
        list[num].mv[1] = list[cand_l1_idx].mv[1];
        list[num].ref_idx[0] = list[cand_l0_idx].ref_idx[0];
        list[num].ref_idx[1] = list[cand_l1_idx].ref_idx[1];
        if (num++ == merge_cand_idx) {
          return list;
        }
      }
    }
  }

  auto ref_list = cu.GetRefPicLists();
  int max_num_refs = !pic_bipred ? ref_list->GetNumRefPics(RefPicList::kL0) :
    std::min(ref_list->GetNumRefPics(RefPicList::kL0),
             ref_list->GetNumRefPics(RefPicList::kL1));
  int ref_idx = 0;
  for (; num < static_cast<int>(list.size()); num++) {
    list[num].inter_dir = pic_bipred ? InterDir::kBi : InterDir::kL0;
    list[num].mv[kL0] = MotionVector(0, 0);
    list[num].mv[kL1] = MotionVector(0, 0);
    list[num].ref_idx[kL0] = ref_idx < max_num_refs ? ref_idx : 0;
    list[num].ref_idx[kL1] = ref_idx < max_num_refs ? ref_idx : 0;
    ref_idx++;
    if (num == merge_cand_idx) {
      return list;
    }
  }
  return list;
}

void InterPrediction::CalculateMV(CodingUnit *cu) {
  if (cu->GetMergeFlag() || cu->GetSkipFlag()) {
    int merge_idx = cu->GetMergeIdx();
    InterMergeCandidateList merge_list = GetMergeCandidates(*cu, merge_idx);
    ApplyMerge(cu, merge_list[merge_idx]);
  } else {
    for (int i = 0; i < static_cast<int>(RefPicList::kTotalNumber); i++) {
      RefPicList ref_list = static_cast<RefPicList>(i);
      if (cu->HasMv(ref_list)) {
        const MotionVector &mvd = cu->GetMvDelta(ref_list);
        int ref_idx = cu->GetRefIdx(ref_list);
        int mvp_idx = cu->GetMvpIdx(ref_list);
        InterPredictorList mvp_list = GetMvPredictors(*cu, ref_list, ref_idx);
        MotionVector &mvp = mvp_list[mvp_idx];
        cu->SetMv(MotionVector(mvp.x + mvd.x, mvp.y + mvd.y), ref_list);
      } else {
        // Zero out for deblocking
        cu->SetMv(MotionVector(), ref_list);
        cu->SetRefIdx(-1, ref_list);
      }
    }
  }
}

void InterPrediction::ApplyMerge(CodingUnit *cu,
                                 const MergeCandidate &merge_cand) {
  cu->SetInterDir(merge_cand.inter_dir);
  for (int i = 0; i < static_cast<int>(RefPicList::kTotalNumber); i++) {
    RefPicList ref_list = static_cast<RefPicList>(i);
    cu->SetMv(merge_cand.mv[i], ref_list);
    cu->SetRefIdx(merge_cand.ref_idx[i], ref_list);
  }
}

void InterPrediction::MotionCompensation(const CodingUnit &cu,
                                         YuvComponent comp, Sample *pred,
                                         ptrdiff_t pred_stride) {
  auto ref_pic_lists = cu.GetRefPicLists();
  if (cu.GetInterDir() != InterDir::kBi) {
    RefPicList ref_list =
      cu.GetInterDir() == InterDir::kL0 ? RefPicList::kL0 : RefPicList::kL1;
    MotionVector mv = cu.GetMv(ref_list);
    int ref_idx = cu.GetRefIdx(ref_list);
    const YuvPicture *ref_pic = ref_pic_lists->GetRefPic(ref_list, ref_idx);
    ClipMV(cu, *ref_pic, &mv.x, &mv.y);
    MotionCompensationMv(cu, comp, *ref_pic, mv.x, mv.y, pred, pred_stride);
  } else {
    // L0
    MotionVector mv_l0 = cu.GetMv(RefPicList::kL0);
    const YuvPicture *ref_pic_l0 =
      ref_pic_lists->GetRefPic(RefPicList::kL0, cu.GetRefIdx(RefPicList::kL0));
    ClipMV(cu, *ref_pic_l0, &mv_l0.x, &mv_l0.y);
    int16_t *dst_pred_l0 = &bipred_temp_[0][0];
    MotionCompensationBi(cu, comp, *ref_pic_l0, mv_l0, dst_pred_l0,
                         constants::kMaxBlockSize);
    // L1
    MotionVector mv_l1 = cu.GetMv(RefPicList::kL1);
    const YuvPicture *ref_pic_l1 =
      ref_pic_lists->GetRefPic(RefPicList::kL1, cu.GetRefIdx(RefPicList::kL1));
    ClipMV(cu, *ref_pic_l1, &mv_l1.x, &mv_l1.y);
    int16_t *dst_pred_l1 = &bipred_temp_[1][0];
    MotionCompensationBi(cu, comp, *ref_pic_l1, mv_l1, dst_pred_l1,
                         constants::kMaxBlockSize);

    AddAvgBi(cu, comp, dst_pred_l0, constants::kMaxBlockSize,
             dst_pred_l1, constants::kMaxBlockSize, pred, pred_stride);
  }
}

void
InterPrediction::MotionCompensationMv(const CodingUnit &cu, YuvComponent comp,
                                      const YuvPicture &ref_pic,
                                      int mv_x, int mv_y,
                                      Sample *pred, ptrdiff_t pred_stride) {
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  int frac_x, frac_y;
  auto ref_buffer =
    GetFullpelRef(cu, comp, ref_pic, mv_x, mv_y, &frac_x, &frac_y);
  if (frac_x == 0 && frac_y == 0) {
    SampleBuffer pred_buffer(pred, pred_stride);
    pred_buffer.CopyFrom(width, height, ref_buffer);
    return;
  }
  if (util::IsLuma(comp)) {
    FilterLuma(width, height, frac_x, frac_y, ref_buffer.GetDataPtr(),
               ref_buffer.GetStride(), pred, pred_stride);
  } else {
    FilterChroma(width, height, frac_x, frac_y, ref_buffer.GetDataPtr(),
                 ref_buffer.GetStride(), pred, pred_stride);
  }
}

void InterPrediction::ClipMV(const CodingUnit &cu, const YuvPicture &ref_pic,
                             int *mv_x, int *mv_y) const {
  const YuvComponent comp = YuvComponent::kY;
  const int offset = 8;
  const int mvshift = constants::kMvPrecisionShift;
  int pos_x = cu.GetPosX(comp);
  int pos_y = cu.GetPosY(comp);
  int pic_min_x = -((constants::kMaxBlockSize + offset + pos_x - 1) << mvshift);
  int pic_min_y = -((constants::kMaxBlockSize + offset + pos_y - 1) << mvshift);
  int pic_max_x = (ref_pic.GetWidth(comp) + offset - pos_x - 1) << mvshift;
  int pic_max_y = (ref_pic.GetHeight(comp) + offset - pos_y - 1) << mvshift;
  *mv_x = util::Clip3(*mv_x, pic_min_x, pic_max_x);
  *mv_y = util::Clip3(*mv_y, pic_min_y, pic_max_y);
}

void
InterPrediction::DetermineMinMaxMv(const CodingUnit &cu,
                                   const YuvPicture &ref_pic,
                                   int center_x, int center_y, int search_range,
                                   MotionVector *mv_min,
                                   MotionVector *mv_max) const {
  const int mvscale = constants::kMvPrecisionShift;
  ClipMV(cu, ref_pic, &center_x, &center_y);
  int search_range_qpel = search_range << mvscale;
  int search_min_x = center_x - search_range_qpel;
  int search_min_y = center_y - search_range_qpel;
  int search_max_x = center_x + search_range_qpel;
  int search_max_y = center_y + search_range_qpel;
  ClipMV(cu, ref_pic, &search_min_x, &search_min_y);
  ClipMV(cu, ref_pic, &search_max_x, &search_max_y);
  *mv_min = MotionVector(search_min_x >> mvscale, search_min_y >> mvscale);
  *mv_max = MotionVector(search_max_x >> mvscale, search_max_y >> mvscale);
}

void InterPrediction::ScaleMv(PicNum poc_current1, PicNum poc_ref1,
                              PicNum poc_current2, PicNum poc_ref2,
                              MotionVector *mv) {
  if (poc_current2 == poc_ref2) {
    return;
  }
  int diff1 =
    util::Clip3(static_cast<int>(poc_current1 - poc_ref1), -128, 127);
  int diff2 =
    util::Clip3(static_cast<int>(poc_current2 - poc_ref2), -128, 127);
  int iX = (16384 + abs(diff2 / 2)) / diff2;
  int scale_factor = util::Clip3((diff1 * iX + 32) >> 6, -4096, 4095);
  mv->x = util::Clip3((scale_factor * mv->x + 127 +
    (scale_factor * mv->x < 0)) >> 8, -32768, 32767);
  mv->y = util::Clip3((scale_factor * mv->y + 127 +
    (scale_factor * mv->y < 0)) >> 8, -32768, 32767);
}

bool InterPrediction::GetMvpCand(const CodingUnit *cu, RefPicList ref_list,
                                 int ref_idx, PicNum ref_poc,
                                 MotionVector *mv_out) {
  if (!cu || !cu->IsInter()) {
    return false;
  }
  if (cu->HasMv(ref_list) && cu->GetRefIdx(ref_list) == ref_idx) {
    *mv_out = cu->GetMv(ref_list);
    return true;
  }
  RefPicList other_list = ReferencePictureLists::Inverse(ref_list);
  if (cu->HasMv(other_list) && cu->GetRefPoc(other_list) == ref_poc) {
    *mv_out = cu->GetMv(other_list);
    return true;
  }
  return false;
}

bool InterPrediction::GetScaledMvpCand(const CodingUnit *cu,
                                       RefPicList cu_ref_list, int ref_idx,
                                       MotionVector *out) {
  if (!cu || !cu->IsInter()) {
    return false;
  }
  for (int i = 0; i < static_cast<int>(RefPicList::kTotalNumber); i++) {
    RefPicList ref_list = (i == 0) ?
      cu_ref_list : ReferencePictureLists::Inverse(cu_ref_list);
    int cu_ref_idx = cu->GetRefIdx(ref_list);
    if (!cu->HasMv(ref_list)) {
      continue;
    }
    if ((i == 0 && cu_ref_idx == ref_idx) ||
        Restrictions::Get().disable_inter_scaling_mvp) {
      *out = cu->GetMv(ref_list);
      return true;
    }
    auto *ref_pic_list = cu->GetRefPicLists();
    PicNum poc_current = cu->GetPoc();
    PicNum poc_ref_1 = ref_pic_list->GetRefPoc(cu_ref_list, ref_idx);
    PicNum poc_ref_2 = ref_pic_list->GetRefPoc(ref_list, cu_ref_idx);
    *out = cu->GetMv(ref_list);
    ScaleMv(poc_current, poc_ref_1, poc_current, poc_ref_2, out);
    return true;
  }
  return false;
}

bool
InterPrediction::GetTemporalMvPredictor(const CodingUnit &cu,
                                        RefPicList ref_list, int ref_idx,
                                        MotionVector *mv_out) {
  const YuvComponent comp = YuvComponent::kY;
  const PicNum cu_poc = cu.GetPoc();
  const PicNum cu_ref_poc = cu.GetRefPicLists()->GetRefPoc(ref_list, ref_idx);
  auto ref_pic_list = cu.GetRefPicLists();
  int tmvp_cu_ref_idx = cu.GetPicData()->GetTmvpRefIdx();
  RefPicList tmvp_cu_ref_list = cu.GetPicData()->GetTmvpRefList();
  RefPicList tmvp_mv_ref_list = ref_pic_list->HasOnlyBackReferences() ?
    ref_list : ReferencePictureLists::Inverse(tmvp_cu_ref_list);

  auto get_temporal_mv =
    [this, &cu_poc, &cu_ref_poc](const CodingUnit *col_cu,
                                 RefPicList col_ref_list,
                                 MotionVector *col_mv) {
    if (!col_cu->IsInter()) {
      return false;
    }
    if (!col_cu->HasMv(col_ref_list)) {
      col_ref_list = ReferencePictureLists::Inverse(col_ref_list);
    }
    int col_ref_idx = col_cu->GetRefIdx(col_ref_list);
    PicNum col_poc = col_cu->GetPoc();
    PicNum col_ref_poc =
      col_cu->GetRefPicLists()->GetRefPoc(col_ref_list, col_ref_idx);
    *col_mv = col_cu->GetMv(col_ref_list);
    ScaleMv(cu_poc, cu_ref_poc, col_poc, col_ref_poc, col_mv);
    return true;
  };

  // Bottom right CU
  int col_x = cu.GetPosX(comp) + cu.GetWidth(comp);
  int col_y = cu.GetPosY(comp) + cu.GetHeight(comp);
  if ((cu.GetPosY(comp) / constants::kMaxBlockSize) ==
    (col_y / constants::kMaxBlockSize)) {
    if (Restrictions::Get().disable_ext_tmvp_full_resolution) {
      col_x = ((col_x >> 4) << 4);
      col_y = ((col_y >> 4) << 4);
    }
    // Including picture out of bounds check
    const CodingUnit *col_cu =
      ref_pic_list->GetCodingUnitAt(tmvp_cu_ref_list, tmvp_cu_ref_idx,
                                    cu.GetCuTree(), col_x, col_y);
    if (col_cu && get_temporal_mv(col_cu, tmvp_mv_ref_list, mv_out)) {
      return true;
    }
  }

  // Center CU
  col_x = cu.GetPosX(comp) + cu.GetWidth(comp) / 2;
  col_y = cu.GetPosY(comp) + cu.GetHeight(comp) / 2;
  if (Restrictions::Get().disable_ext_tmvp_full_resolution) {
    col_x = ((col_x >> 4) << 4);
    col_y = ((col_y >> 4) << 4);
  }
  const CodingUnit *col_cu =
    ref_pic_list->GetCodingUnitAt(tmvp_cu_ref_list, tmvp_cu_ref_idx,
                                  cu.GetCuTree(), col_x, col_y);
  if (get_temporal_mv(col_cu, tmvp_mv_ref_list, mv_out)) {
    return true;
  }
  return false;
}

void
InterPrediction::MotionCompensationBi(const CodingUnit &cu, YuvComponent comp,
                                      const YuvPicture &ref_pic,
                                      const MotionVector &mv,
                                      int16_t *pred, ptrdiff_t pred_stride) {
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  int frac_x, frac_y;
  auto ref_buffer =
    GetFullpelRef(cu, comp, ref_pic, mv.x, mv.y, &frac_x, &frac_y);
  if (frac_x == 0 && frac_y == 0) {
    const int shift = kInternalPrecision - bitdepth_;
    const int offset = kInternalOffset;
    const int i = width > 2;
    simd_.filter_copy_bipred[i](width, height, offset, shift,
                                ref_buffer.GetDataPtr(),
                                ref_buffer.GetStride(), pred, pred_stride);
    return;
  }
  if (util::IsLuma(comp)) {
    FilterLumaBipred(width, height, frac_x, frac_y,
                     ref_buffer.GetDataPtr(), ref_buffer.GetStride(),
                     pred, pred_stride);
  } else {
    FilterChromaBipred(width, height, frac_x, frac_y,
                       ref_buffer.GetDataPtr(), ref_buffer.GetStride(),
                       pred, pred_stride);
  }
}

DataBuffer<const Sample>
InterPrediction::GetFullpelRef(const CodingUnit &cu, YuvComponent comp,
                               const YuvPicture &ref_pic, int mv_x, int mv_y,
                               int *frac_x, int *frac_y) {
  const int shift_x = ref_pic.GetSizeShiftX(comp);
  const int shift_y = ref_pic.GetSizeShiftY(comp);
  int pel_x = mv_x >> (constants::kMvPrecisionShift + shift_x);
  int pel_y = mv_y >> (constants::kMvPrecisionShift + shift_y);
  if (util::IsLuma(comp)) {
    *frac_x = mv_x & ((1 << constants::kMvPrecisionShift) - 1);
    *frac_y = mv_y & ((1 << constants::kMvPrecisionShift) - 1);
  } else  if (Restrictions::Get().disable_inter_chroma_subpel) {
    int s_x = constants::kMvPrecisionShift + shift_x;
    int s_y = constants::kMvPrecisionShift + shift_y;
    pel_x = (mv_x + (1 << (s_x - 1))) >> s_x;
    pel_y = (mv_y + (1 << (s_y - 1))) >> s_y;
    *frac_x = 0;
    *frac_y = 0;
  } else {
    *frac_x = mv_x & ((1 << (constants::kMvPrecisionShift + shift_x)) - 1);
    *frac_y = mv_y & ((1 << (constants::kMvPrecisionShift + shift_y)) - 1);
    *frac_x <<= (1 - shift_x);
    *frac_y <<= (1 - shift_y);
  }
  int posx = cu.GetPosX(comp);
  int posy = cu.GetPosY(comp);
  const Sample *sample_ptr =
    ref_pic.GetSamplePtr(comp, posx + pel_x, posy + pel_y);;
  return DataBuffer<const Sample>(sample_ptr, ref_pic.GetStride(comp));
}

template<int N>
static void FilterHorSampleSample(int width, int height, int bitdepth,
                                  const int16_t *filter,
                                  const Sample *src, ptrdiff_t src_stride,
                                  Sample *dst, ptrdiff_t dst_stride) {
  const int shift = InterPrediction::GetFilterShift<Sample, true>(bitdepth);
  const int offset = InterPrediction::GetFilterOffset<Sample, true>(shift);
  const Sample sample_max = (1 << bitdepth) - 1;
  src -= N / 2 - 1;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int sum = 0;
      sum += src[x + 0] * filter[0];
      sum += src[x + 1] * filter[1];
      sum += src[x + 2] * filter[2];
      sum += src[x + 3] * filter[3];
      if (N == 8) {
        sum += src[x + 4] * filter[4];
        sum += src[x + 5] * filter[5];
        sum += src[x + 6] * filter[6];
        sum += src[x + 7] * filter[7];
      }
      int val = (sum + offset) >> shift;
      dst[x] = util::ClipBD(val, sample_max);
    }
    src += src_stride;
    dst += dst_stride;
  }
}

template<int N>
static void FilterHorSampleShort(int width, int height, int bitdepth,
                                 const int16_t *filter,
                                 const Sample *src, ptrdiff_t src_stride,
                                 int16_t *dst, ptrdiff_t dst_stride) {
  const int shift = InterPrediction::GetFilterShift<Sample, false>(bitdepth);
  const int offset = InterPrediction::GetFilterOffset<Sample, false>(shift);
  src -= N / 2 - 1;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int sum = 0;
      sum += src[x + 0] * filter[0];
      sum += src[x + 1] * filter[1];
      sum += src[x + 2] * filter[2];
      sum += src[x + 3] * filter[3];
      if (N == 8) {
        sum += src[x + 4] * filter[4];
        sum += src[x + 5] * filter[5];
        sum += src[x + 6] * filter[6];
        sum += src[x + 7] * filter[7];
      }
      dst[x] = static_cast<int16_t>((sum + offset) >> shift);
    }
    src += src_stride;
    dst += dst_stride;
  }
}

template<int N>
static void FilterVerSampleSample(int width, int height, int bitdepth,
                                  const int16_t *filter,
                                  const Sample *src, ptrdiff_t src_stride,
                                  Sample *dst, ptrdiff_t dst_stride) {
  const int shift = InterPrediction::GetFilterShift<Sample, true>(bitdepth);
  const int offset = InterPrediction::GetFilterOffset<Sample, true>(shift);
  const Sample sample_max = (1 << bitdepth) - 1;
  src -= (N / 2 - 1) * src_stride;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int sum = 0;
      sum += src[x + 0 * src_stride] * filter[0];
      sum += src[x + 1 * src_stride] * filter[1];
      sum += src[x + 2 * src_stride] * filter[2];
      sum += src[x + 3 * src_stride] * filter[3];
      if (N == 8) {
        sum += src[x + 4 * src_stride] * filter[4];
        sum += src[x + 5 * src_stride] * filter[5];
        sum += src[x + 6 * src_stride] * filter[6];
        sum += src[x + 7 * src_stride] * filter[7];
      }
      int16_t val = static_cast<int16_t>((sum + offset) >> shift);
      dst[x] = util::ClipBD(val, sample_max);
    }
    src += src_stride;
    dst += dst_stride;
  }
}

template<int N>
static void FilterVerSampleShort(int width, int height, int bitdepth,
                                 const int16_t *filter,
                                 const Sample *src, ptrdiff_t src_stride,
                                 int16_t *dst, ptrdiff_t dst_stride) {
  const int shift = InterPrediction::GetFilterShift<Sample, false>(bitdepth);
  const int offset = InterPrediction::GetFilterOffset<Sample, false>(shift);
  src -= (N / 2 - 1) * src_stride;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int sum = 0;
      sum += src[x + 0 * src_stride] * filter[0];
      sum += src[x + 1 * src_stride] * filter[1];
      sum += src[x + 2 * src_stride] * filter[2];
      sum += src[x + 3 * src_stride] * filter[3];
      if (N == 8) {
        sum += src[x + 4 * src_stride] * filter[4];
        sum += src[x + 5 * src_stride] * filter[5];
        sum += src[x + 6 * src_stride] * filter[6];
        sum += src[x + 7 * src_stride] * filter[7];
      }
      dst[x] = static_cast<int16_t>((sum + offset) >> shift);
    }
    src += src_stride;
    dst += dst_stride;
  }
}

template<int N>
static void FilterVerShortSample(int width, int height, int bitdepth,
                                 const int16_t *filter,
                                 const int16_t *src, ptrdiff_t src_stride,
                                 Sample *dst, ptrdiff_t dst_stride) {
  const int shift = InterPrediction::GetFilterShift<int16_t, true>(bitdepth);
  const int offset = InterPrediction::GetFilterOffset<int16_t, true>(shift);
  const Sample sample_max = (1 << bitdepth) - 1;
  src -= (N / 2 - 1) * src_stride;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int sum = 0;
      sum += src[x + 0 * src_stride] * filter[0];
      sum += src[x + 1 * src_stride] * filter[1];
      sum += src[x + 2 * src_stride] * filter[2];
      sum += src[x + 3 * src_stride] * filter[3];
      if (N == 8) {
        sum += src[x + 4 * src_stride] * filter[4];
        sum += src[x + 5 * src_stride] * filter[5];
        sum += src[x + 6 * src_stride] * filter[6];
        sum += src[x + 7 * src_stride] * filter[7];
      }
      int16_t val = static_cast<int16_t>((sum + offset) >> shift);
      dst[x] = util::ClipBD(val, sample_max);
    }
    src += src_stride;
    dst += dst_stride;
  }
}

template<int N>
static void FilterVerShortShort(int width, int height, int bitdepth,
                                const int16_t *filter,
                                const int16_t *src, ptrdiff_t src_stride,
                                int16_t *dst, ptrdiff_t dst_stride) {
  const int shift = InterPrediction::GetFilterShift<int16_t, false>(bitdepth);
  const int offset = InterPrediction::GetFilterOffset<int16_t, false>(bitdepth);
  src -= (N / 2 - 1) * src_stride;

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int sum = 0;
      sum += src[x + 0 * src_stride] * filter[0];
      sum += src[x + 1 * src_stride] * filter[1];
      sum += src[x + 2 * src_stride] * filter[2];
      sum += src[x + 3 * src_stride] * filter[3];
      if (N == 8) {
        sum += src[x + 4 * src_stride] * filter[4];
        sum += src[x + 5 * src_stride] * filter[5];
        sum += src[x + 6 * src_stride] * filter[6];
        sum += src[x + 7 * src_stride] * filter[7];
      }
      dst[x] = static_cast<int16_t>((sum + offset) >> shift);
    }
    src += src_stride;
    dst += dst_stride;
  }
}

void InterPrediction::FilterLuma(int width, int height, int frac_x, int frac_y,
                                 const Sample *ref, ptrdiff_t ref_stride,
                                 Sample *pred, ptrdiff_t pred_stride) {
  const int N = kNumTapsLuma;
  const int16_t *filter_hor = &kLumaFilter[frac_x][0];
  const int16_t *filter_ver = &kLumaFilter[frac_y][0];
  if (frac_y == 0) {
    simd_.filter_h_sample_sample[0](width, height, bitdepth_, filter_hor,
                                    ref, ref_stride, pred, pred_stride);
  } else if (frac_x == 0) {
    simd_.filter_v_sample_sample[0](width, height, bitdepth_, filter_ver,
                                    ref, ref_stride, pred, pred_stride);
  } else {
    ptrdiff_t hor_offset = (N / 2 - 1) * ref_stride;
    simd_.filter_h_sample_short[0](width, height + N - 1, bitdepth_, filter_hor,
                                   ref - hor_offset, ref_stride,
                                   &filter_buffer_[0], width);
    int ver_offset = (N / 2 - 1) * width;
    simd_.filter_v_short_sample[0](width, height, bitdepth_, filter_ver,
                                   &filter_buffer_[ver_offset], width,
                                   pred, pred_stride);
  }
}

void InterPrediction::FilterChroma(int width, int height,
                                   int frac_x, int frac_y,
                                   const Sample *ref, ptrdiff_t ref_stride,
                                   Sample *pred, ptrdiff_t pred_stride) {
  const int N = kNumTapsChroma;
  const int16_t *filter_hor = &kChromaFilter[frac_x][0];
  const int16_t *filter_ver = &kChromaFilter[frac_y][0];
  if (frac_y == 0) {
    simd_.filter_h_sample_sample[1](width, height, bitdepth_, filter_hor,
                                    ref, ref_stride, pred, pred_stride);
  } else if (frac_x == 0) {
    simd_.filter_v_sample_sample[1](width, height, bitdepth_, filter_ver,
                                    ref, ref_stride, pred, pred_stride);
  } else {
    ptrdiff_t hor_offset = (N / 2 - 1) * ref_stride;
    simd_.filter_h_sample_short[1](width, height + N - 1, bitdepth_, filter_hor,
                                   ref - hor_offset, ref_stride,
                                   &filter_buffer_[0], width);
    int ver_offset = (N / 2 - 1) * width;
    simd_.filter_v_short_sample[1](width, height, bitdepth_, filter_ver,
                                   &filter_buffer_[ver_offset], width,
                                   pred, pred_stride);
  }
}

static void FilterCopyBipred(int width, int height, int16_t offset, int shift,
                             const Sample *ref, ptrdiff_t ref_stride,
                             int16_t *pred, ptrdiff_t pred_stride) {
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int16_t val = ref[x] << shift;
      pred[x] = val - offset;
    }
    ref += ref_stride;
    pred += pred_stride;
  }
}

void
InterPrediction::FilterLumaBipred(int width, int height, int frac_x, int frac_y,
                                  const Sample *ref, ptrdiff_t ref_stride,
                                  int16_t *pred, ptrdiff_t pred_stride) {
  const int N = kNumTapsLuma;
  const int16_t *filter_hor = &kLumaFilter[frac_x][0];
  const int16_t *filter_ver = &kLumaFilter[frac_y][0];
  if (frac_y == 0) {
    simd_.filter_h_sample_short[0](width, height, bitdepth_, filter_hor,
                                   ref, ref_stride, pred, pred_stride);
  } else if (frac_x == 0) {
    simd_.filter_v_sample_short[0](width, height, bitdepth_, filter_ver,
                                   ref, ref_stride, pred, pred_stride);
  } else {
    ptrdiff_t hor_offset = (N / 2 - 1) * ref_stride;
    simd_.filter_h_sample_short[0](width, height + N - 1, bitdepth_, filter_hor,
                                   ref - hor_offset, ref_stride,
                                   &filter_buffer_[0], width);
    int ver_offset = (N / 2 - 1) * width;
    simd_.filter_v_short_short[0](width, height, bitdepth_, filter_ver,
                                  &filter_buffer_[ver_offset], width,
                                  pred, pred_stride);
  }
}

void
InterPrediction::FilterChromaBipred(int width, int height,
                                    int frac_x, int frac_y,
                                    const Sample *ref, ptrdiff_t ref_stride,
                                    int16_t *pred, ptrdiff_t pred_stride) {
  const int N = kNumTapsChroma;
  const int16_t *filter_hor = &kChromaFilter[frac_x][0];
  const int16_t *filter_ver = &kChromaFilter[frac_y][0];
  if (frac_y == 0) {
    simd_.filter_h_sample_short[1](width, height, bitdepth_, filter_hor,
                                   ref, ref_stride, pred, pred_stride);
  } else if (frac_x == 0) {
    simd_.filter_v_sample_short[1](width, height, bitdepth_, filter_ver,
                                   ref, ref_stride, pred, pred_stride);
  } else {
    ptrdiff_t hor_offset = (N / 2 - 1) * ref_stride;
    simd_.filter_h_sample_short[1](width, height + N - 1, bitdepth_, filter_hor,
                                   ref - hor_offset, ref_stride,
                                   &filter_buffer_[0], width);
    int ver_offset = (N / 2 - 1) * width;
    simd_.filter_v_short_short[1](width, height, bitdepth_, filter_ver,
                                  &filter_buffer_[ver_offset], width,
                                  pred, pred_stride);
  }
}

void InterPrediction::AddAvgBi(const CodingUnit &cu, YuvComponent comp,
                               const int16_t *src_l0, ptrdiff_t src_l0_stride,
                               const int16_t *src_l1, ptrdiff_t src_l1_stride,
                               Sample *pred, ptrdiff_t pred_stride) {
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const int shift = std::max(2, kInternalPrecision - bitdepth_) + 1;
  const int offset = (1 << (shift - 1)) + 2 * kInternalOffset;
  const int i = width > 2;
  simd_.add_avg[i](width, height, offset, shift, bitdepth_,
                   src_l0, src_l0_stride, src_l1, src_l1_stride,
                   pred, pred_stride);
}

static void AddAvg(int width, int height, int offset,
                   int shift, int bitdepth,
                   const int16_t *src1, intptr_t stride1,
                   const int16_t *src2, intptr_t stride2,
                   Sample *dst, intptr_t dst_stride) {
  const Sample min_val = 0;
  const Sample max_val = (1 << bitdepth) - 1;
  DataBuffer<const int16_t> src1_buf(src1, stride1);
  DataBuffer<const int16_t> src2_buf(src2, stride2);
  SampleBuffer pred_buffer(dst, dst_stride);
  pred_buffer.AddAvg(width, height, src1_buf, src2_buf, offset, shift,
                     min_val, max_val);
}

MergeCandidate InterPrediction::GetMergeCandidateFromCu(const CodingUnit &cu) {
  const int kL0 = static_cast<int>(RefPicList::kL0);
  const int kL1 = static_cast<int>(RefPicList::kL1);
  MergeCandidate cand;
  cand.inter_dir = cu.GetInterDir();
  cand.mv[kL0] = cu.GetMv(RefPicList::kL0);
  cand.mv[kL1] = cu.GetMv(RefPicList::kL1);
  cand.ref_idx[kL0] = cu.GetRefIdx(RefPicList::kL0);
  cand.ref_idx[kL1] = cu.GetRefIdx(RefPicList::kL1);
  return cand;
}

InterPrediction::SimdFunc::SimdFunc() {
  add_avg[0] = &AddAvg;
  add_avg[1] = &AddAvg;
  filter_copy_bipred[0] = &FilterCopyBipred;
  filter_copy_bipred[1] = &FilterCopyBipred;
  filter_h_sample_sample[0] = &FilterHorSampleSample<kNumTapsLuma>;
  filter_h_sample_sample[1] = &FilterHorSampleSample<kNumTapsChroma>;
  filter_h_sample_short[0] = &FilterHorSampleShort<kNumTapsLuma>;
  filter_h_sample_short[1] = &FilterHorSampleShort<kNumTapsChroma>;
  filter_v_sample_sample[0] = &FilterVerSampleSample<kNumTapsLuma>;
  filter_v_sample_sample[1] = &FilterVerSampleSample<kNumTapsChroma>;
  filter_v_sample_short[0] = &FilterVerSampleShort<kNumTapsLuma>;
  filter_v_sample_short[1] = &FilterVerSampleShort<kNumTapsChroma>;
  filter_v_short_sample[0] = &FilterVerShortSample<kNumTapsLuma>;
  filter_v_short_sample[1] = &FilterVerShortSample<kNumTapsChroma>;
  filter_v_short_short[0] = &FilterVerShortShort<kNumTapsLuma>;
  filter_v_short_short[1] = &FilterVerShortShort<kNumTapsChroma>;
}

}   // namespace xvc
