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
#include <limits>
#include <type_traits>

#include "xvc_common_lib/utils.h"
#include "xvc_common_lib/reference_picture_lists.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/simd_cpu.h"

namespace xvc {

static const int kL0 = static_cast<int>(RefPicList::kL0);
static const int kL1 = static_cast<int>(RefPicList::kL1);

struct InterPrediction::LicParams {
  int scale;
  int offset;
  static const int shift = 5;
};

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
InterPrediction::GetMvpList(const CodingUnit &cu, RefPicList ref_list,
                            int ref_idx) {
  auto round_mv = [](MotionVector *mv) {
    constexpr int scale_shift = constants::kMvPrecisionShift;
    mv->x =
      ((mv->x + (1 << (scale_shift - 1))) >> scale_shift) * (1 << scale_shift);
    mv->y =
      ((mv->y + (1 << (scale_shift - 1))) >> scale_shift) * (1 << scale_shift);
  };
  if (Restrictions::Get().disable_inter_mvp) {
    MotionVector mvp(0, 0);
    MvCorner corner;
    const CodingUnit *tmp;
    if ((tmp = cu.GetCodingUnit(NeighborDir::kLeft, &corner)) != nullptr &&
        tmp->IsInter() && tmp->HasMv(ref_list)) {
      mvp = tmp->GetMv(ref_list, corner);
    } else if ((tmp = cu.GetCodingUnit(NeighborDir::kAbove, &corner)) != nullptr
               && tmp->IsInter() && tmp->HasMv(ref_list)) {
      mvp = tmp->GetMv(ref_list, corner);
    }
    if (cu.GetFullpelMv()) {
      round_mv(&mvp);
    }
    InterPredictorList list;
    for (int i = 0; i < static_cast<int>(list.size()); i++) {
      list[i] = mvp;
    }
    return list;
  }
  const PicNum ref_poc = cu.GetRefPicLists()->GetRefPoc(ref_list, ref_idx);
  InterPredictorList list;
  int i = 0;

  const CodingUnit *tmp = cu.GetCodingUnitLeftBelow();
  if (!tmp || !tmp->IsInter()) {
    tmp = cu.GetCodingUnitLeftCorner();
  }
  bool smvp_added = tmp && tmp->IsInter();

  // Left
  if (GetMvpCand(&cu, NeighborDir::kLeftBelow,
                 ref_list, ref_idx, ref_poc, &list[i], 0)) {
    i++;
  } else if (GetMvpCand(&cu, NeighborDir::kLeftCorner,
                        ref_list, ref_idx, ref_poc, &list[i], 0)) {
    i++;
  } else if (GetScaledMvpCand(&cu, NeighborDir::kLeftBelow,
                              ref_list, ref_idx, &list[i], 0)) {
    i++;
  } else if (GetScaledMvpCand(&cu, NeighborDir::kLeftCorner,
                              ref_list, ref_idx, &list[i], 0)) {
    i++;
  }

  // Above
  if (GetMvpCand(&cu, NeighborDir::kAboveRight,
                 ref_list, ref_idx, ref_poc, &list[i], 0)) {
    i++;
  } else if (GetMvpCand(&cu, NeighborDir::kAboveCorner,
                        ref_list, ref_idx, ref_poc, &list[i], 0)) {
    i++;
  } else if (GetMvpCand(&cu, NeighborDir::kAboveLeft,
                        ref_list, ref_idx, ref_poc, &list[i], 0)) {
    i++;
  }
  if (!smvp_added) {
    if (GetScaledMvpCand(&cu, NeighborDir::kAboveRight,
                         ref_list, ref_idx, &list[i], 0)) {
      i++;
    } else if (GetScaledMvpCand(&cu, NeighborDir::kAboveCorner,
                                ref_list, ref_idx, &list[i], 0)) {
      i++;
    } else if (GetScaledMvpCand(&cu, NeighborDir::kAboveLeft,
                                ref_list, ref_idx, &list[i], 0)) {
      i++;
    }
  }

  if (cu.GetFullpelMv()) {
    for (int j = 0; j < i; j++) {
      round_mv(&list[j]);
    }
  }

  if (i == 2 && list[0] == list[1]) {
    i = 1;
  }

  if (constants::kTemporalMvPrediction && cu.GetPicData()->GetTmvpValid() &&
      !Restrictions::Get().disable_inter_tmvp_mvp && i < 2) {
    if (GetTemporalMvPredictor(cu, ref_list, ref_idx, &list[i], nullptr)) {
      if (cu.GetFullpelMv()) {
        round_mv(&list[i]);
      }
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

AffinePredictorList
InterPrediction::GetMvpListAffine(const CodingUnit &cu, RefPicList ref_list,
                                  int ref_idx, int max_num_mvp) {
  const int width = cu.GetWidth(YuvComponent::kY);
  const int height = cu.GetHeight(YuvComponent::kY);
  const YuvPicture *ref_pic = cu.GetRefPicLists()->GetRefPic(ref_list, ref_idx);
  const PicNum ref_poc = cu.GetRefPicLists()->GetRefPoc(ref_list, ref_idx);
  std::array<MotionVector, 3> list0;
  std::array<MotionVector, 2> list1;
  std::array<MotionVector, 2> list2;
  AffinePredictorList list;
  if (Restrictions::Get().disable_ext2_inter_affine_mvp) {
    MotionVector3 mvp = {};
    const CodingUnit *tmp;
    if ((tmp = cu.GetCodingUnitLeft()) != nullptr &&
        tmp->GetUseAffine() && tmp->HasMv(ref_list)) {
      mvp = tmp->GetMvAffine(ref_list);
    } else if ((tmp = cu.GetCodingUnitAbove()) != nullptr &&
               tmp->GetUseAffine() && tmp->HasMv(ref_list)) {
      mvp = tmp->GetMvAffine(ref_list);
    }
    for (int i = 0; i < static_cast<int>(list.size()); i++) {
      list[i] = mvp;
    }
    return list;
  }

  int i0 = 0;
  i0 += GetMvpCand(&cu, NeighborDir::kAboveLeft,
                   ref_list, ref_idx, ref_poc, &list0[0], i0);
  i0 += GetMvpCand(&cu, NeighborDir::kAbove,
                   ref_list, ref_idx, ref_poc, &list0[0], i0);
  i0 += GetMvpCand(&cu, NeighborDir::kLeft,
                   ref_list, ref_idx, ref_poc, &list0[0], i0);
  if (i0 < static_cast<int>(list0.size())) {
    i0 += GetScaledMvpCand(&cu, NeighborDir::kAboveLeft,
                           ref_list, ref_idx, &list0[0], i0);
  }
  if (i0 < static_cast<int>(list0.size())) {
    i0 += GetScaledMvpCand(&cu, NeighborDir::kAbove,
                           ref_list, ref_idx, &list0[0], i0);
  }
  if (i0 < static_cast<int>(list0.size())) {
    i0 += GetScaledMvpCand(&cu, NeighborDir::kLeft,
                           ref_list, ref_idx, &list0[0], i0);
  }

  int i1 = 0;
  i1 += GetMvpCand(&cu, NeighborDir::kAboveCorner,
                   ref_list, ref_idx, ref_poc, &list1[0], i1);
  i1 += GetMvpCand(&cu, NeighborDir::kAboveRight,
                   ref_list, ref_idx, ref_poc, &list1[0], i1);
  if (i1 < static_cast<int>(list1.size())) {
    i1 += GetScaledMvpCand(&cu, NeighborDir::kAboveCorner,
                           ref_list, ref_idx, &list1[0], i1);
  }
  if (i1 < static_cast<int>(list1.size())) {
    i1 += GetScaledMvpCand(&cu, NeighborDir::kAboveRight,
                           ref_list, ref_idx, &list1[0], i1);
  }

  int i2 = 0;
  i2 += GetMvpCand(&cu, NeighborDir::kLeftCorner,
                   ref_list, ref_idx, ref_poc, &list2[0], i2);
  i2 += GetMvpCand(&cu, NeighborDir::kLeftBelow,
                   ref_list, ref_idx, ref_poc, &list2[0], i2);
  if (i2 < static_cast<int>(list2.size())) {
    i2 += GetScaledMvpCand(&cu, NeighborDir::kLeftCorner,
                           ref_list, ref_idx, &list2[0], i2);
  }
  if (i2 < static_cast<int>(list2.size())) {
    i2 += GetScaledMvpCand(&cu, NeighborDir::kLeftBelow,
                           ref_list, ref_idx, &list2[0], i2);
  }

  auto get_length = [width, height](const MotionVector &mv0,
                                    const MotionVector &mv1,
                                    const MotionVector &mv2) {
    const int max_x = width >> 1;
    const int max_y = height >> 1;
    MotionVector mv_hor(mv1.x - mv0.x, mv1.y - mv0.y);
    MotionVector mv_ver(mv2.x - mv0.x, mv2.y - mv0.y);
    if (mv_hor.x == 0 && mv_hor.y == 0) {
      return -1;
    }
    if (std::abs(mv_hor.x) > max_x || std::abs(mv_hor.y) > max_y ||
        std::abs(mv_ver.x) > max_x || std::abs(mv_ver.y) > max_y) {
      return -1;
    }
    return std::abs(mv_hor.x * height - mv_ver.y * width) +
      std::abs(mv_hor.y * height + mv_ver.x * width);
  };

  constexpr size_t max_size = list0.size() * list1.size() * list2.size();
  std::array<std::array<int, 3>, max_size> comb_list;
  std::array<int, max_size> comb_cost;
  int num_comb = 0;
  for (int j0 = 0; j0 < i0; j0++) {
    for (int j1 = 0; j1 < i1; j1++) {
      for (int j2 = 0; j2 < i2; j2++) {
        int len = get_length(list0[j0], list1[j1], list2[j2]);
        if (len < 0) {
          continue;
        }
        comb_cost[num_comb] = len;
        comb_list[num_comb] = { j0, j1, j2 };
        num_comb++;
      }
    }
  }

  const int num_list = std::min(num_comb, static_cast<int>(list.size()));
  for (int out = 0; out < num_list; out++) {
    int best_i = 0;
    for (int i = 1; i < num_comb; i++) {
      best_i = comb_cost[i] < comb_cost[best_i] ? i : best_i;
    }
    comb_cost[best_i] = std::numeric_limits<int>::max();
    list[out] = DeriveMvAffine(cu, *ref_pic, list0[comb_list[best_i][0]],
                               list1[comb_list[best_i][1]]);
    if (out >= max_num_mvp) {
      return list;
    }
  }

  if (num_list < static_cast<int>(list.size())) {
    InterPredictorList normal_mvp = GetMvpList(cu, ref_list, ref_idx);
    for (int out = num_list; out < static_cast<int>(list.size()); out++) {
      MotionVector &mvp = normal_mvp[out - num_list];
      list[out] = DeriveMvAffine(cu, *ref_pic, mvp, mvp);
    }
  }

  return list;
}

static bool HasDifferentMotion(const CodingUnit &cu1, MvCorner corner1,
                               const CodingUnit &cu2, MvCorner corner2) {
  if (cu1.GetInterDir() != cu2.GetInterDir()) {
    return true;
  }
  if (cu1.GetUseLic() != cu2.GetUseLic()) {
    return true;
  }
  for (int i = 0; i < static_cast<int>(RefPicList::kTotalNumber); i++) {
    RefPicList ref_list = RefPicList(i);
    if (!cu1.HasMv(ref_list)) {
      continue;
    }
    if (cu1.GetRefIdx(ref_list) != cu2.GetRefIdx(ref_list) ||
        cu1.GetMv(ref_list, corner1) != cu2.GetMv(ref_list, corner2)) {
      return true;
    }
  }
  return false;
}

InterMergeCandidateList
InterPrediction::GetMergeCandidates(const CodingUnit &cu, int merge_cand_idx) {
  const bool can_lic = cu.GetPicData()->GetUseLocalIlluminationCompensation();
  const bool pic_bipred = cu.GetPicType() == PicturePredictionType::kBi;
  InterMergeCandidateList list;
  int num = 0;

  MvCorner left_corner_mv;
  const CodingUnit *left_corner =
    cu.GetCodingUnit(NeighborDir::kLeftCorner, &left_corner_mv);
  bool has_a1 = left_corner && left_corner->IsInter();
  if (has_a1) {
    list[num] = GetMergeCandidateFromCu(*left_corner, left_corner_mv);
    if (num++ == merge_cand_idx) {
      return list;
    }
  }

  MvCorner above_corner_mv;
  const CodingUnit *above_corner =
    cu.GetCodingUnit(NeighborDir::kAboveCorner, &above_corner_mv);
  bool has_b1 = above_corner && above_corner->IsInter();
  if (has_b1 &&
    (!has_a1 || HasDifferentMotion(*left_corner, left_corner_mv,
                                   *above_corner, above_corner_mv))) {
    list[num] = GetMergeCandidateFromCu(*above_corner, above_corner_mv);
    if (num++ == merge_cand_idx) {
      return list;
    }
  }

  MvCorner above_right_mv;
  const CodingUnit *above_right =
    cu.GetCodingUnit(NeighborDir::kAboveRight, &above_right_mv);
  bool has_b0 = above_right && above_right->IsInter();
  if (has_b0 &&
    (!has_b1 || HasDifferentMotion(*above_corner, above_corner_mv,
                                   *above_right, above_right_mv))) {
    list[num] = GetMergeCandidateFromCu(*above_right, above_right_mv);
    if (num++ == merge_cand_idx) {
      return list;
    }
  }

  MvCorner left_below_mv;
  const CodingUnit *left_below =
    cu.GetCodingUnit(NeighborDir::kLeftBelow, &left_below_mv);
  bool has_a0 = left_below && left_below->IsInter();
  if (has_a0 && (!has_a1 || HasDifferentMotion(*left_corner, left_corner_mv,
                                               *left_below, left_below_mv))) {
    list[num] = GetMergeCandidateFromCu(*left_below, left_below_mv);
    if (num++ == merge_cand_idx) {
      return list;
    }
  }

  MvCorner above_left_mv;
  const CodingUnit *above_left =
    cu.GetCodingUnit(NeighborDir::kAboveLeft, &above_left_mv);
  bool has_b2 = above_left && above_left->IsInter();
  if (has_b2 && num < 4
      && (!has_a1 || HasDifferentMotion(*left_corner, left_corner_mv,
                                        *above_left, above_left_mv))
      && (!has_b1 || HasDifferentMotion(*above_corner, above_corner_mv,
                                        *above_left, above_left_mv))) {
    list[num] = GetMergeCandidateFromCu(*above_left, above_left_mv);
    if (num++ == merge_cand_idx) {
      return list;
    }
  }

  if (constants::kTemporalMvPrediction && num < static_cast<int>(list.size()) &&
      !Restrictions::Get().disable_inter_tmvp_merge &&
      cu.GetPicData()->GetTmvpValid()) {
    bool use_lic = false;
    bool found_any = GetTemporalMvPredictor(cu, RefPicList::kL0, 0,
                                            &list[num].mv[0], &use_lic);
    list[num].ref_idx[0] = 0;
    list[num].inter_dir = InterDir::kL0;
    if (pic_bipred && GetTemporalMvPredictor(cu, RefPicList::kL1, 0,
                                             &list[num].mv[1], &use_lic)) {
      list[num].ref_idx[1] = 0;
      list[num].inter_dir = found_any ? InterDir::kBi : InterDir::kL1;
      found_any = true;
    }
    list[num].use_lic = can_lic && use_lic;
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
        list[num].use_lic =
          list[cand_l0_idx].use_lic || list[cand_l1_idx].use_lic;
        if (num++ == merge_cand_idx) {
          return list;
        }
      }
    }
  }

  const ReferencePictureLists *ref_pic_list = cu.GetRefPicLists();
  const int max_num_refs = !pic_bipred ?
    ref_pic_list->GetNumRefPics(RefPicList::kL0) :
    std::min(ref_pic_list->GetNumRefPics(RefPicList::kL0),
             ref_pic_list->GetNumRefPics(RefPicList::kL1));
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

AffineMergeCandidate
InterPrediction::GetAffineMergeCand(const CodingUnit &cu) {
  const YuvComponent comp = YuvComponent::kY;
  const CodingUnit *neigh = cu.GetCodingUnitLeftCorner();
  if (!neigh || !neigh->GetUseAffine()) {
    neigh = cu.GetCodingUnitAboveCorner();
  }
  if (!neigh || !neigh->GetUseAffine()) {
    neigh = cu.GetCodingUnitAboveRight();
  }
  if (!neigh || !neigh->GetUseAffine()) {
    neigh = cu.GetCodingUnitLeftBelow();
  }
  if (!neigh || !neigh->GetUseAffine()) {
    neigh = cu.GetCodingUnitAboveLeft();
  }
  assert(neigh && neigh->GetUseAffine());
  const float scale_x =
    1.0f * (cu.GetPosX(comp) - neigh->GetPosX(comp)) / neigh->GetWidth(comp);
  const float scale_y =
    1.0f * (cu.GetPosY(comp) - neigh->GetPosY(comp)) / neigh->GetHeight(comp);
  const float scale_len_x = 1.0f * cu.GetWidth(comp) / neigh->GetWidth(comp);
  const float scale_len_y = 1.0f * cu.GetHeight(comp) / neigh->GetHeight(comp);
  auto scale_mv = [&](const MotionVector3 &ref) {
    const int mv_x = static_cast<int>(ref[0].x +
      (ref[2].x - ref[0].x) * scale_y + (ref[1].x - ref[0].x) * scale_x);
    const int mv_y = static_cast<int>(ref[0].y +
      (ref[2].y - ref[0].y) * scale_y + (ref[1].y - ref[0].y) * scale_x);
    return MotionVector3{
      MotionVector(mv_x, mv_y),
      MotionVector(static_cast<int>(mv_x + (ref[1].x - ref[0].x) * scale_len_x),
      static_cast<int>(mv_y + (ref[1].y - ref[0].y) * scale_len_x)),
      MotionVector(static_cast<int>(mv_x + (ref[2].x - ref[0].x) * scale_len_y),
      static_cast<int>(mv_y + (ref[2].y - ref[0].y) * scale_len_y))
    };
  };
  AffineMergeCandidate merge_cand;
  merge_cand.inter_dir = neigh->GetInterDir();
  if (neigh->HasMv(RefPicList::kL0)) {
    merge_cand.mv[kL0] = scale_mv(neigh->GetMvAffine(RefPicList::kL0));
    merge_cand.ref_idx[kL0] = neigh->GetRefIdx(RefPicList::kL0);
  }
  if (neigh->HasMv(RefPicList::kL1)) {
    merge_cand.mv[kL1] = scale_mv(neigh->GetMvAffine(RefPicList::kL1));
    merge_cand.ref_idx[kL1] = neigh->GetRefIdx(RefPicList::kL1);
  }
  // TODO(PH) Consider removing these, only needed to mimic a true mv field
  if (cu.GetWidth(comp) <= constants::kMinBlockSize) {
    merge_cand.mv[kL0][1] = merge_cand.mv[kL0][0];
    merge_cand.mv[kL1][1] = merge_cand.mv[kL1][0];
  }
  if (cu.GetHeight(comp) <= constants::kMinBlockSize) {
    merge_cand.mv[kL0][2] = merge_cand.mv[kL0][0];
    merge_cand.mv[kL1][2] = merge_cand.mv[kL1][0];
  }
  return merge_cand;
}

MotionVector3
InterPrediction::DeriveMvAffine(const CodingUnit &cu, const YuvPicture &ref_pic,
                                const MotionVector &mv1,
                                const MotionVector &mv2) {
  const int width = cu.GetWidth(YuvComponent::kY);
  const int height = cu.GetHeight(YuvComponent::kY);
  MotionVector3 out;
  out[0] = mv1;
  out[1] = mv2;
  ClipMV(cu, ref_pic, &out[0].x, &out[0].y);
  ClipMV(cu, ref_pic, &out[1].x, &out[1].y);
  out[2].x = out[0].x - (out[1].y - out[0].y) * height / width;
  out[2].y = out[0].y + (out[1].x - out[0].x) * height / width;
  ClipMV(cu, ref_pic, &out[2].x, &out[2].y);
  return out;
}

void InterPrediction::CalculateMV(CodingUnit *cu) {
  if (cu->GetMergeFlag()) {
    int merge_idx = cu->GetMergeIdx();
    if (cu->GetUseAffine()) {
      assert(merge_idx == 0);
      AffineMergeCandidate merge_cand = GetAffineMergeCand(*cu);
      ApplyMergeCand(cu, merge_cand);
    } else {
      InterMergeCandidateList merge_list = GetMergeCandidates(*cu, merge_idx);
      ApplyMergeCand(cu, merge_list[merge_idx]);
    }
  } else if (cu->GetUseAffine()) {
    for (int i = 0; i < static_cast<int>(RefPicList::kTotalNumber); i++) {
      RefPicList ref_list = static_cast<RefPicList>(i);
      if (cu->HasMv(ref_list)) {
        const int ref_idx = cu->GetRefIdx(ref_list);
        const int mvp_idx = cu->GetMvpIdx(ref_list);
        const YuvPicture *ref_pic =
          cu->GetRefPicLists()->GetRefPic(ref_list, ref_idx);
        const MotionVector &mvd0 = cu->GetMvdAffine(0, ref_list);
        const MotionVector &mvd1 = cu->GetMvdAffine(1, ref_list);
        const AffinePredictorList mvp_list =
          GetMvpListAffine(*cu, ref_list, ref_idx, mvp_idx);
        MotionVector3 mv = mvp_list[mvp_idx];
        mv[0].x += mvd0.x;
        mv[0].y += mvd0.y;
        mv[1].x += mvd1.x;
        mv[1].y += mvd1.y;
        mv = DeriveMvAffine(*cu, *ref_pic, mv[0], mv[1]);
        cu->SetMv(mv, ref_list);
      } else {
        // Zero out for deblocking
        cu->SetMv(MotionVector(), ref_list);
        cu->SetRefIdx(-1, ref_list);
      }
    }
  } else {
    for (int i = 0; i < static_cast<int>(RefPicList::kTotalNumber); i++) {
      RefPicList ref_list = static_cast<RefPicList>(i);
      if (cu->HasMv(ref_list)) {
        const int ref_idx = cu->GetRefIdx(ref_list);
        const int mvp_idx = cu->GetMvpIdx(ref_list);
        MotionVector mvd = cu->GetMvDelta(ref_list);
        if (cu->GetFullpelMv()) {
          mvd.x = mvd.x * (1 << constants::kMvPrecisionShift);
          mvd.y = mvd.y * (1 << constants::kMvPrecisionShift);
        }
        InterPredictorList mvp_list = GetMvpList(*cu, ref_list, ref_idx);
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

void InterPrediction::ApplyMergeCand(CodingUnit *cu,
                                     const MergeCandidate &merge_cand) {
  cu->SetInterDir(merge_cand.inter_dir);
  cu->SetUseLic(merge_cand.use_lic);
  for (int i = 0; i < static_cast<int>(RefPicList::kTotalNumber); i++) {
    RefPicList ref_list = static_cast<RefPicList>(i);
    cu->SetMv(merge_cand.mv[i], ref_list);
    cu->SetRefIdx(merge_cand.ref_idx[i], ref_list);
  }
}

void InterPrediction::ApplyMergeCand(CodingUnit *cu,
                                     const AffineMergeCandidate &merge_cand) {
  cu->SetInterDir(merge_cand.inter_dir);
  for (int i = 0; i < static_cast<int>(RefPicList::kTotalNumber); i++) {
    RefPicList ref_list = static_cast<RefPicList>(i);
    cu->SetMv(merge_cand.mv[i], ref_list);
    cu->SetRefIdx(merge_cand.ref_idx[i], ref_list);
  }
}

void InterPrediction::MotionCompensation(const CodingUnit &cu,
                                         YuvComponent comp,
                                         SampleBuffer *pred_buffer) {
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  if (cu.GetInterDir() != InterDir::kBi) {
    // Uni-prediction
    RefPicList ref_list =
      cu.GetInterDir() == InterDir::kL0 ? RefPicList::kL0 : RefPicList::kL1;
    MotionCompRefList(cu, comp, ref_list, true, pred_buffer);
  } else {
    DataBuffer<int16_t>
      pred_l0_buffer(&bipred_temp_[0][0], constants::kMaxBlockSize);
    DataBuffer<int16_t>
      pred_l1_buffer(&bipred_temp_[1][0], constants::kMaxBlockSize);
    if (cu.GetUseLic()) {
      // Bi-prediction with intermediate rounding
      MotionCompRefList(cu, comp, RefPicList::kL0, true, pred_buffer);
      FilterCopyBipred(width, height, *pred_buffer, &pred_l0_buffer);
      MotionCompRefList(cu, comp, RefPicList::kL1, true, pred_buffer);
      FilterCopyBipred(width, height, *pred_buffer, &pred_l1_buffer);
    } else {
      // Normal bi-prediction
      MotionCompRefList(cu, comp, RefPicList::kL0, false, &pred_l0_buffer);
      MotionCompRefList(cu, comp, RefPicList::kL1, false, &pred_l1_buffer);
    }
    AddAvgBi(cu, comp, pred_l0_buffer, pred_l1_buffer, pred_buffer);
  }
}

void
InterPrediction::MotionCompensationMv(const CodingUnit &cu, YuvComponent comp,
                                      const YuvPicture & ref_pic,
                                      int mv_x, int mv_y, bool post_filter,
                                      SampleBuffer *pred_buffer) {
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  ClipMV(cu, ref_pic, &mv_x, &mv_y);
  int frac_x, frac_y;
  SampleBufferConst ref_buffer =
    GetFullpelRef(cu, comp, ref_pic, mv_x, mv_y, &frac_x, &frac_y);
  MotionCompUniPred(width, height, comp, ref_buffer, frac_x, frac_y,
                    pred_buffer);
  if (post_filter && cu.GetUseLic()) {
    LocalIlluminationComp(cu, comp, mv_x, mv_y, ref_pic, pred_buffer);
  }
}

void
InterPrediction::MotionCompensationMv(const CodingUnit & cu, YuvComponent comp,
                                      const YuvPicture &ref_pic,
                                      const MotionVector3 &mv,
                                      SampleBuffer *pred_buffer) {
  MotionCompAffine(cu, comp, ref_pic, mv, pred_buffer);
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

bool
InterPrediction::GetMvpCand(const CodingUnit *cu_this, NeighborDir dir,
                            RefPicList ref_list, int ref_idx, PicNum ref_poc,
                            MotionVector *mv_list, int index) {
  MvCorner mv_corner;
  const CodingUnit *cu = cu_this->GetCodingUnit(dir, &mv_corner);
  if (!cu || !cu->IsInter()) {
    return false;
  }
  if (cu->HasMv(ref_list) && cu->GetRefIdx(ref_list) == ref_idx) {
    const MotionVector &mv = cu->GetMv(ref_list, mv_corner);
    bool unique = true;
    for (int i = 0; i < index; i++) {
      if (mv_list[i] == mv) {
        unique = false;
      }
    }
    if (unique) {
      mv_list[index] = mv;
      return true;
    }
  }
  RefPicList other_list = ReferencePictureLists::Inverse(ref_list);
  if (cu->HasMv(other_list) && cu->GetRefPoc(other_list) == ref_poc) {
    const MotionVector &mv = cu->GetMv(other_list, mv_corner);
    bool unique = true;
    for (int i = 0; i < index; i++) {
      if (mv_list[i] == mv) {
        unique = false;
      }
    }
    if (unique) {
      mv_list[index] = mv;
      return true;
    }
  }
  return false;
}

bool
InterPrediction::GetScaledMvpCand(const CodingUnit *cu_this, NeighborDir dir,
                                  RefPicList cu_ref_list, int ref_idx,
                                  MotionVector *mv_list, int index) {
  MvCorner mv_corner;
  const CodingUnit *cu = cu_this->GetCodingUnit(dir, &mv_corner);
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
      const MotionVector &mv = cu->GetMv(ref_list, mv_corner);
      bool unique = true;
      for (int j = 0; j < index; j++) {
        if (mv_list[j] == mv) {
          unique = false;
        }
      }
      if (unique) {
        mv_list[index] = mv;
        return true;
      }
    }
    auto *ref_pic_list = cu->GetRefPicLists();
    PicNum poc_current = cu->GetPoc();
    PicNum poc_ref_1 = ref_pic_list->GetRefPoc(cu_ref_list, ref_idx);
    PicNum poc_ref_2 = ref_pic_list->GetRefPoc(ref_list, cu_ref_idx);
    MotionVector mv = cu->GetMv(ref_list, mv_corner);
    ScaleMv(poc_current, poc_ref_1, poc_current, poc_ref_2, &mv);
    bool unique = true;
    for (int j = 0; j < index; j++) {
      if (mv_list[j] == mv) {
        unique = false;
      }
    }
    if (unique) {
      mv_list[index] = mv;
      return true;
    }
  }
  return false;
}

bool
InterPrediction::GetTemporalMvPredictor(const CodingUnit &cu,
                                        RefPicList ref_list, int ref_idx,
                                        MotionVector *mv_out, bool *use_lic) {
  const YuvComponent comp = YuvComponent::kY;
  const PicNum cu_poc = cu.GetPoc();
  const PicNum cu_ref_poc = cu.GetRefPicLists()->GetRefPoc(ref_list, ref_idx);
  const ReferencePictureLists *ref_pic_list = cu.GetRefPicLists();
  int tmvp_cu_ref_idx = cu.GetPicData()->GetTmvpRefIdx();
  RefPicList tmvp_cu_ref_list = cu.GetPicData()->GetTmvpRefList();
  RefPicList tmvp_mv_ref_list = ref_pic_list->HasOnlyBackReferences() ?
    ref_list : ReferencePictureLists::Inverse(tmvp_cu_ref_list);

  auto get_temporal_mv = [&cu_poc, &cu_ref_poc](const CodingUnit *col_cu,
                                                RefPicList col_ref_list,
                                                int x, int y,
                                                MotionVector *col_mv) {
    if (!col_cu->IsInter()) {
      return false;
    }
    if (!col_cu->HasMv(col_ref_list)) {
      col_ref_list = ReferencePictureLists::Inverse(col_ref_list);
    }
    MvCorner mv_corner = col_cu->GetMvCorner(x, y);
    int col_ref_idx = col_cu->GetRefIdx(col_ref_list);
    PicNum col_poc = col_cu->GetPoc();
    PicNum col_ref_poc =
      col_cu->GetRefPicLists()->GetRefPoc(col_ref_list, col_ref_idx);
    *col_mv = col_cu->GetMv(col_ref_list, mv_corner);
    ScaleMv(cu_poc, cu_ref_poc, col_poc, col_ref_poc, col_mv);
    return true;
  };

  // Bottom right CU
  int col_x = cu.GetPosX(comp) + cu.GetWidth(comp);
  int col_y = cu.GetPosY(comp) + cu.GetHeight(comp);
  if ((cu.GetPosY(comp) / constants::kMaxBlockSize) ==
    (col_y / constants::kMaxBlockSize)) {
    bool valid = true;
    if (Restrictions::Get().disable_ext_tmvp_full_resolution) {
      valid = col_x < cu.GetPicData()->GetPictureWidth(comp) &&
        col_y < cu.GetPicData()->GetPictureHeight(comp);
      col_x = ((col_x >> 4) << 4);
      col_y = ((col_y >> 4) << 4);
    }
    // Including picture out of bounds check
    const CodingUnit *col_cu =
      ref_pic_list->GetCodingUnitAt(tmvp_cu_ref_list, tmvp_cu_ref_idx,
                                    cu.GetCuTree(), col_x, col_y);
    if (valid && col_cu && get_temporal_mv(col_cu, tmvp_mv_ref_list,
                                           col_x, col_y, mv_out)) {
      if (use_lic) {
        *use_lic |= col_cu->GetUseLic();
      }
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
  if (get_temporal_mv(col_cu, tmvp_mv_ref_list, col_x, col_y, mv_out)) {
    if (use_lic) {
      *use_lic |= col_cu->GetUseLic();
    }
    return true;
  }
  return false;
}

template<typename PredBuffer>
void
InterPrediction::MotionCompRefList(const CodingUnit &cu, YuvComponent comp,
                                   RefPicList ref_list, bool post_filter,
                                   PredBuffer *pred_buffer) {
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const int ref_idx = cu.GetRefIdx(ref_list);
  const ReferencePictureLists *ref_pic_lists = cu.GetRefPicLists();
  const YuvPicture *ref_pic = ref_pic_lists->GetRefPic(ref_list, ref_idx);
  if (cu.GetUseAffine()) {
    const MotionVector3 &mv_affine = cu.GetMvAffine(ref_list);
    MotionCompAffine(cu, comp, *ref_pic, mv_affine, pred_buffer);
  } else {
    MotionVector mv = cu.GetMv(ref_list, MvCorner::kDefault);
    ClipMV(cu, *ref_pic, &mv.x, &mv.y);
    int frac_x, frac_y;
    SampleBufferConst ref_buffer =
      GetFullpelRef(cu, comp, *ref_pic, mv.x, mv.y, &frac_x, &frac_y);
    MotionCompUniPred(width, height, comp, ref_buffer, frac_x, frac_y,
                      pred_buffer);
    if (post_filter && cu.GetUseLic()) {
      // Performing lic is only supported on Sample types (not int16_t samples)
      // TODO(PH) Consider adding more templates to remove this cast
      if (std::is_same<PredBuffer, SampleBuffer>::value) {
        SampleBuffer *sample_buffer =
          reinterpret_cast<SampleBuffer*>(pred_buffer);
        LocalIlluminationComp(cu, comp, mv.x, mv.y, *ref_pic, sample_buffer);
      }
    }
  }
}

template<typename PredBuffer>
void InterPrediction::MotionCompAffine(const CodingUnit &cu, YuvComponent comp,
                                       const YuvPicture &ref_pic,
                                       const MotionVector3 &mv_affine,
                                       PredBuffer *pred_buffer) {
  static const int kAffinePrec = 8;
  const YuvComponent luma = YuvComponent::kY;
  const int pos_x = cu.GetPosX(comp);
  const int pos_y = cu.GetPosY(comp);
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const int mv_shift_x =
    constants::kMvPrecisionShift + ref_pic.GetSizeShiftX(comp);
  const int mv_shift_y =
    constants::kMvPrecisionShift + ref_pic.GetSizeShiftY(comp);
  const int mv_scale = 1 << constants::kMvPrecisionShift;

  MotionVector3 mv = mv_affine;
  ClipMV(cu, ref_pic, &mv[0].x, &mv[0].y);
  ClipMV(cu, ref_pic, &mv[1].x, &mv[1].y);
  ClipMV(cu, ref_pic, &mv[2].x, &mv[2].y);
  if (mv[0] == mv[1]) {
    int frac_x, frac_y;
    SampleBufferConst ref_buffer =
      GetFullpelRef(cu, comp, ref_pic, mv[0].x, mv[0].y, &frac_x, &frac_y);
    return MotionCompUniPred(width, height, comp, ref_buffer, frac_x, frac_y,
                             pred_buffer);
  }

  auto get_subblock_size = [](const MotionVector& ref, MotionVector &mv_uni,
                              int size, int scale) {
    static const int kMinSubblockSize = 4;
    static const int kSizeShift = 6 - constants::kMvPrecisionShift;
    const int max_len =
      std::max(std::abs(mv_uni.x - ref.x), std::abs(mv_uni.y - ref.y));
    if (!max_len) {
      return size;
    }
    int subblock_size = std::max(1, (size >> kSizeShift) / max_len);
    while (size % subblock_size) {
      subblock_size--;
    }
    return std::max(kMinSubblockSize, subblock_size) >> scale;
  };
  const int subblock_width =
    get_subblock_size(mv[0], mv[1], width, ref_pic.GetSizeShiftX(comp));
  const int subblock_height =
    get_subblock_size(mv[0], mv[2], height, ref_pic.GetSizeShiftY(comp));
  // TODO(PH) ClipMv inlined for performance, consider using lambda instead?
  const int mv_max_x =
    (rec_pic_.GetWidth(luma) - cu.GetPosX(luma) + 8 - 1) * mv_scale;
  const int mv_min_x =
    (-constants::kMaxBlockSize - cu.GetPosX(luma) - 8 + 1) * mv_scale;
  const int mv_max_y =
    (rec_pic_.GetHeight(luma) - cu.GetPosY(luma) + 8 - 1) * mv_scale;
  const int mv_min_y =
    (-constants::kMaxBlockSize - cu.GetPosY(luma) - 8 + 1) * mv_scale;
  const int delta_mv_hor_x = ((mv[1].x - mv[0].x) * (1 << kAffinePrec)) / width;
  const int delta_mv_hor_y = ((mv[1].y - mv[0].y) * (1 << kAffinePrec)) / width;
  const int delta_mv_ver_x = -delta_mv_hor_y;
  const int delta_mv_ver_y = delta_mv_hor_x;
  MotionVector mv_hor(mv[0].x * (1 << kAffinePrec),
                      mv[0].y * (1 << kAffinePrec));
  MotionVector mv_ver = mv_hor;

  for (int subblock_y = 0; subblock_y < height; subblock_y += subblock_height) {
    for (int subblock_x = 0; subblock_x < width; subblock_x += subblock_width) {
      const int mv_x =
        util::Clip3((mv_hor.x + delta_mv_hor_x * (subblock_width >> 1) +
                     delta_mv_ver_x * (subblock_height >> 1)) >> kAffinePrec,
                    mv_min_x, mv_max_x);
      const int mv_y =
        util::Clip3((mv_hor.y + delta_mv_hor_y * (subblock_width >> 1) +
                     delta_mv_ver_y * (subblock_height >> 1)) >> kAffinePrec,
                    mv_min_y, mv_max_y);
      const int mv_full_x = mv_x >> mv_shift_x;
      const int mv_full_y = mv_y >> mv_shift_y;
      const int frac_x = mv_x & ((1 << mv_shift_x) - 1);
      const int frac_y = mv_y & ((1 << mv_shift_y) - 1);
      SampleBufferConst ref_buffer =
        ref_pic.GetSampleBuffer(comp, pos_x + subblock_x + mv_full_x,
                                pos_y + subblock_y + mv_full_y);
      PredBuffer block_buffer = pred_buffer->Offset(subblock_x, subblock_y);
      MotionCompUniPred(subblock_width, subblock_height, comp, ref_buffer,
                        frac_x, frac_y, &block_buffer);
      mv_hor.x += delta_mv_hor_x * subblock_width;
      mv_hor.y += delta_mv_hor_y * subblock_width;
    }

    mv_ver.x += delta_mv_ver_x * subblock_height;
    mv_ver.y += delta_mv_ver_y * subblock_height;
    mv_hor = mv_ver;
  }
}

void
InterPrediction::MotionCompUniPred(int width, int height, YuvComponent comp,
                                   const SampleBufferConst &ref_buffer,
                                   int frac_x, int frac_y,
                                   SampleBuffer *pred_buffer) {
  if (frac_x == 0 && frac_y == 0) {
    pred_buffer->CopyFrom(width, height, ref_buffer);
  } else if (util::IsLuma(comp)) {
    FilterLuma(width, height, frac_x, frac_y,
               ref_buffer.GetDataPtr(), ref_buffer.GetStride(),
               pred_buffer->GetDataPtr(), pred_buffer->GetStride());
  } else {
    FilterChroma(width, height, frac_x, frac_y,
                 ref_buffer.GetDataPtr(), ref_buffer.GetStride(),
                 pred_buffer->GetDataPtr(), pred_buffer->GetStride());
  }
}

void
InterPrediction::MotionCompUniPred(int width, int height, YuvComponent comp,
                                   const SampleBufferConst &ref_buffer,
                                   int frac_x, int frac_y,
                                   DataBuffer<int16_t> *pred_buffer) {
  if (frac_x == 0 && frac_y == 0) {
    FilterCopyBipred(width, height, ref_buffer, pred_buffer);
  } else if (util::IsLuma(comp)) {
    FilterLumaBipred(width, height, frac_x, frac_y,
                     ref_buffer.GetDataPtr(), ref_buffer.GetStride(),
                     pred_buffer->GetDataPtr(), pred_buffer->GetStride());
  } else {
    FilterChromaBipred(width, height, frac_x, frac_y,
                       ref_buffer.GetDataPtr(), ref_buffer.GetStride(),
                       pred_buffer->GetDataPtr(), pred_buffer->GetStride());
  }
}

SampleBufferConst
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
  return SampleBufferConst(sample_ptr, ref_pic.GetStride(comp));
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

void InterPrediction::FilterCopyBipred(int width, int height,
                                       const SampleBufferConst &ref_buffer,
                                       DataBuffer<int16_t>* pred_buffer) {
  const int shift = kInternalPrecision - bitdepth_;
  const int offset = kInternalOffset;
  const int i = width > 2;
  simd_.filter_copy_bipred[i](width, height, offset, shift,
                              ref_buffer.GetDataPtr(), ref_buffer.GetStride(),
                              pred_buffer->GetDataPtr(),
                              pred_buffer->GetStride());
}

static void FilterCopyBipred_c(int width, int height, int16_t offset, int shift,
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
                               const DataBuffer<const int16_t> &src_l0_buffer,
                               const DataBuffer<const int16_t> &src_l1_buffer,
                               SampleBuffer *pred_buffer) {
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const int shift = std::max(2, kInternalPrecision - bitdepth_) + 1;
  const int offset = (1 << (shift - 1)) + 2 * kInternalOffset;
  const int i = width > 2;
  simd_.add_avg[i](width, height, offset, shift, bitdepth_,
                   src_l0_buffer.GetDataPtr(), src_l0_buffer.GetStride(),
                   src_l1_buffer.GetDataPtr(), src_l1_buffer.GetStride(),
                   pred_buffer->GetDataPtr(), pred_buffer->GetStride());
}

void
InterPrediction::LocalIlluminationComp(const CodingUnit &cu, YuvComponent comp,
                                       int mv_x, int mv_y,
                                       const YuvPicture &ref_pic,
                                       SampleBuffer *pred_buffer) {
  assert(!Restrictions::Get().disable_ext2_inter_local_illumination_comp);
  const int shift_x =
    constants::kMvPrecisionShift + ref_pic.GetSizeShiftX(comp);
  const int shift_y =
    constants::kMvPrecisionShift + ref_pic.GetSizeShiftY(comp);
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const Sample max_val = (1 << rec_pic_.GetBitdepth()) - 1;
  const MotionVector mv_fullpel((mv_x + (1 << (shift_x - 1))) >> shift_x,
    (mv_y + (1 << (shift_y - 1))) >> shift_y);
  SampleBufferConst rec_buffer =
    rec_pic_.GetSampleBuffer(comp, cu.GetPosX(comp), cu.GetPosY(comp));

  LicParams params = DeriveLicParams(cu, comp, mv_fullpel, ref_pic, rec_buffer);
  pred_buffer->AddLinearModel(width, height, *pred_buffer, params.scale,
                              params.shift, params.offset, 0, max_val);
}

InterPrediction::LicParams
InterPrediction::DeriveLicParams(const CodingUnit &cu, YuvComponent comp,
                                 const MotionVector &mv_full,
                                 const YuvPicture &ref_pic,
                                 const SampleBufferConst &src_buffer) {
  static const int kModelQuantShift = 15;
  static const int kDefaultScaleShift = 5;
  static const int kModelMinResolutionShift = 6;
  static const int kModelPrecisionShift = 7;
  const auto get_msb = [](unsigned int x) {
    int msb = 0;
    while (x) {
      msb++;
      x >>= 1;
    }
    return msb;
  };
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const CodingUnit *cu_above = cu.GetCodingUnitAbove();
  const CodingUnit *cu_left = cu.GetCodingUnitLeft();
  const int step_size = std::min(width, height) > 8 ? 2 : 1;
  const SampleBufferConst ref_buffer =
    ref_pic.GetSampleBuffer(comp, cu.GetPosX(comp), cu.GetPosY(comp));
  int sum_x = 0;
  int sum_y = 0;
  int sum_xx = 0;
  int sum_xy = 0;
  int nbr = 0;

  if (!cu_above && !cu_left) {
    return LicParams{ 1 << kDefaultScaleShift, 0 };
  }
  if (cu_above) {
    MotionVector mv_clip = mv_full;
    ClipMV(*cu_above, ref_pic, &mv_clip.x, &mv_clip.y);
    const Sample *ref = ref_buffer.GetDataPtr() + mv_clip.x +
      (mv_clip.y * ref_buffer.GetStride()) - ref_buffer.GetStride();
    const Sample *src = src_buffer.GetDataPtr() - src_buffer.GetStride();
    const int dx = step_size * std::max(1, width / height);
    for (int x = 0; x < width; x += dx) {
      sum_x += ref[x];
      sum_y += src[x];
      sum_xx += ref[x] * ref[x];
      sum_xy += ref[x] * src[x];
      nbr++;
    }
  }
  if (cu_left) {
    MotionVector mv_clip = mv_full;
    ClipMV(*cu_left, ref_pic, &mv_clip.x, &mv_clip.y);
    const Sample *ref = ref_buffer.GetDataPtr() + mv_clip.x +
      (mv_clip.y * ref_buffer.GetStride()) - 1;
    const Sample *src = src_buffer.GetDataPtr() - 1;
    const int dy = step_size * std::max(1, height / width);
    for (int y = 0; y < height; y += dy) {
      const Sample a = ref[y * ref_buffer.GetStride()];
      const Sample b = src[y * src_buffer.GetStride()];
      sum_x += a;
      sum_y += b;
      sum_xx += a * a;
      sum_xy += a * b;
      nbr++;
    }
  }
  const int size_shift = util::SizeToLog2(nbr);
  const int base_shift = std::max(0, bitdepth_ + size_shift - kModelQuantShift);
  const int avg_x = sum_x >> base_shift;
  const int avg_y = sum_y >> base_shift;
  const int xx_offset = sum_xx >> kModelPrecisionShift;
  const int avg_xy = (((sum_xy + xx_offset) >> (2 * base_shift)) << size_shift);
  const int avg_xx = (((sum_xx + xx_offset) >> (2 * base_shift)) << size_shift);
  const int stddev_xy = avg_xy - avg_x * avg_y;
  const int stddev_xx = avg_xx - avg_x * avg_x;
  const int shift_xx_quant = std::max(0, get_msb(std::abs(stddev_xx)) -
                                      kModelMinResolutionShift);
  const int shift_xy = std::max(0, shift_xx_quant - 12);
  const int total_shift = kModelQuantShift - kDefaultScaleShift +
    shift_xx_quant - shift_xy;
  const int stddev_xy_shifted = stddev_xy >> shift_xy;
  const int stddev_xx_shifted = util::Clip3(stddev_xx >> shift_xx_quant, 0,
    (1 << kModelMinResolutionShift) - 1);
  if (stddev_xx_shifted == 0) {
    return LicParams{ 1 << kDefaultScaleShift, 0 };
  }
  const int stddev_xx_scaled =
    ((1 << kModelQuantShift) + (stddev_xx_shifted / 2)) / stddev_xx_shifted;
  const int scale = (stddev_xy_shifted * stddev_xx_scaled) >> total_shift;
  LicParams params;
  assert(params.shift == kDefaultScaleShift);
  params.scale = util::Clip3(scale, 0, 1 << (kDefaultScaleShift + 2));
  const int offset = (sum_y - ((params.scale * sum_x) >> params.shift) +
    (1 << (size_shift - 1))) >> size_shift;
  params.offset =
    util::Clip3(offset, -(1 << (bitdepth_ - 1)), (1 << (bitdepth_ - 1)) - 1);
  return params;
}

static void AddAvg_c(int width, int height, int offset,
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

MergeCandidate InterPrediction::GetMergeCandidateFromCu(const CodingUnit &cu,
                                                        MvCorner corner) {
  MergeCandidate cand;
  cand.inter_dir = cu.GetInterDir();
  cand.mv[kL0] = cu.GetMv(RefPicList::kL0, corner);
  cand.mv[kL1] = cu.GetMv(RefPicList::kL1, corner);
  cand.ref_idx[kL0] = cu.GetRefIdx(RefPicList::kL0);
  cand.ref_idx[kL1] = cu.GetRefIdx(RefPicList::kL1);
  cand.use_lic = cu.GetUseLic();
  return cand;
}

InterPrediction::SimdFunc::SimdFunc() {
  add_avg[0] = &AddAvg_c;
  add_avg[1] = &AddAvg_c;
  filter_copy_bipred[0] = &FilterCopyBipred_c;
  filter_copy_bipred[1] = &FilterCopyBipred_c;
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
