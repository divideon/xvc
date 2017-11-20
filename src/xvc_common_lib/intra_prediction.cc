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

#include "xvc_common_lib/intra_prediction.h"

#include <algorithm>
#include <cassert>
#include <utility>

#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

static const std::array<int8_t, 17> kAngleTable = {
  -32, -26, -21, -17, -13, -9, -5, -2, 0, 2, 5, 9, 13, 17, 21, 26, 32
};

static const std::array<int8_t, 33> kAngleTableExt = {
  -32, -29, -26, -23, -21, -19, -17, -15, -13, -11, -9, -7, -5, -3, -2, -1,
  0, 1, 2, 3, 5, 7, 9, 11, 13, 15, 17, 19, 21, 23, 26, 29, 32
};

static const std::array<int16_t, 8> kInvAngleTable = {
  4096, 1638, 910, 630, 482, 390, 315, 256
};

static const std::array<int16_t, 16> kInvAngleTableExt = {
  8192, 4096, 2731, 1638, 1170, 910, 745, 630, 546, 482, 431, 390, 356, 315,
  282, 256
};

struct IntraPrediction::NeighborState {
  bool has_any() const {
    return has_above_left || has_above || has_left ||
      has_above_right > 0 || has_below_left > 0;
  }
  bool has_all(int below_left, int above_right) const {
    return has_above_left && has_above && has_left &&
      has_below_left == below_left && has_above_right == above_right;
  }
  bool has_above_left = false;
  bool has_above = false;
  int has_above_right = 0;
  bool has_left = false;
  int has_below_left = 0;
};

struct IntraPrediction::LmParams {
  int scale;
  int offset;
  int shift;
};

IntraPrediction::IntraPrediction(int bitdepth)
  : bitdepth_(bitdepth),
  temp_pred_buffer_(constants::kMaxBlockSize + kDownscaleLumaPadding,
                    constants::kMaxBlockSize + kDownscaleLumaPadding) {
}

void IntraPrediction::Predict(IntraMode intra_mode, const CodingUnit &cu,
                              YuvComponent comp, const RefState &ref_state,
                              const YuvPicture &rec_pic,
                              SampleBuffer *output_buffer) {
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  Sample *out_ptr = output_buffer->GetDataPtr();
  const ptrdiff_t out_stride = output_buffer->GetStride();
  const Sample *ref_samples = &ref_state.ref_samples[0];
  if (Restrictions::Get().disable_intra_planar &&
      intra_mode == IntraMode::kPlanar) {
    intra_mode = IntraMode::kDc;
  }
  if (util::IsLuma(comp)) {
    bool filtered_ref = UseFilteredRefSamples(cu, intra_mode);
    ref_samples =
      filtered_ref ? &ref_state.ref_filtered[0] : &ref_state.ref_samples[0];
  }
  const bool post_filter = util::IsLuma(comp) && width <= 16 && height <= 16;
  switch (intra_mode) {
    case IntraMode::kPlanar:
      PlanarPred(width, height, ref_samples, kRefSampleStride_,
                 out_ptr, out_stride);
      break;

    case IntraMode::kDc:
      PredIntraDC(width, height, post_filter,
                  &ref_state.ref_samples[0], kRefSampleStride_,
                  out_ptr, out_stride);
      break;

    case IntraMode::kInvalid:
      assert(0);
      break;

    case IntraMode::kLmChroma:
      PredLmChroma(cu, comp, rec_pic, output_buffer);
      break;

    default:
      AngularPred(width, height, intra_mode, post_filter,
                  ref_samples, kRefSampleStride_, out_ptr, out_stride);
      break;
  }
}

void
IntraPrediction::FillReferenceState(const CodingUnit &cu, YuvComponent comp,
                                    const YuvPicture &rec_pic,
                                    RefState *ref_state) {
  SampleBufferConst src_buffer =
    rec_pic.GetSampleBuffer(comp, cu.GetPosX(comp), cu.GetPosY(comp));
  const Sample *src_ptr = src_buffer.GetDataPtr();
  const ptrdiff_t src_stride = src_buffer.GetStride();
  NeighborState neighbors = DetermineNeighbors(cu, comp);
  ComputeRefSamples(cu.GetWidth(comp), cu.GetHeight(comp), neighbors,
                    src_ptr, src_stride, &ref_state->ref_samples[0],
                    kRefSampleStride_);

  // TODO(Dev) optimize decoder by skipping filtering depending on intra mode
  if (util::IsLuma(comp)) {
    FilterRefSamples(cu.GetWidth(comp), cu.GetHeight(comp),
                     &ref_state->ref_samples[0], &ref_state->ref_filtered[0],
                     kRefSampleStride_);
  }
}

IntraPredictorLuma
IntraPrediction::GetPredictorLuma(const CodingUnit &cu) const {
  const int max_modes = !Restrictions::Get().disable_ext2_intra_67_modes ?
    kNbrIntraModesExt : kNbrIntraModes - 1;
  const int offset = !Restrictions::Get().disable_ext2_intra_67_modes ?
    kNbrIntraModesExt - 5 : kNbrIntraModes - 6;
  IntraPredictorLuma mpm;
  if (Restrictions::Get().disable_intra_mpm_prediction) {
    mpm.num_neighbor_modes = 1;
    mpm[0] = IntraMode::kPlanar;
    mpm[1] = IntraMode::kDc;
    mpm[2] = Convert(IntraAngle::kVertical);
    if (!Restrictions::Get().disable_ext2_intra_6_predictors) {
      mpm[3] = Convert(IntraAngle::kHorizontal);
      mpm[4] = Convert(IntraAngle::kDiagonal);
      mpm[5] = static_cast<IntraMode>(2);
    }
    return mpm;
  }
  if (Restrictions::Get().disable_ext2_intra_6_predictors) {
    FillPredictorLumaDefault(cu, &mpm);
    return mpm;
  }
  std::array<bool, kNbrIntraModesExt> added_modes = { false };
  auto add_predictor_from_cu =
    [&added_modes, &mpm](int mpm_index, const CodingUnit *tmp) {
    if (tmp && tmp->IsIntra()) {
      IntraMode mode = tmp->GetIntraMode(YuvComponent::kY);
      if (!added_modes[mode]) {
        added_modes[mode] = true;
        mpm[mpm_index++] = mode;
        return 1;
      }
    }
    return 0;
  };
  auto add_predictor_if_new =
    [&added_modes, &mpm](int mpm_index, IntraMode mode) {
    if (!added_modes[mode]) {
      added_modes[mode] = true;
      mpm[mpm_index++] = mode;
      return 1;
    }
    return 0;
  };
  int index = 0;
  if (index < constants::kNumIntraMpmExt) {
    index += add_predictor_from_cu(index, cu.GetCodingUnitLeftCorner());
  }
  if (index < constants::kNumIntraMpmExt) {
    index += add_predictor_from_cu(index, cu.GetCodingUnitAboveCorner());
  }
  mpm.num_neighbor_modes = index > 1 ? 3 : 2;
  if (index < constants::kNumIntraMpmExt) {
    index += add_predictor_if_new(index, IntraMode::kPlanar);
  }
  if (index < constants::kNumIntraMpmExt) {
    index += add_predictor_if_new(index, IntraMode::kDc);
  }
  if (index < constants::kNumIntraMpmExt) {
    index += add_predictor_from_cu(index, cu.GetCodingUnitLeftBelow());
  }
  if (index < constants::kNumIntraMpmExt) {
    index += add_predictor_from_cu(index, cu.GetCodingUnitAboveRight());
  }
  if (index < constants::kNumIntraMpmExt) {
    index += add_predictor_from_cu(index, cu.GetCodingUnitAboveLeft());
  }
  int current_added = index;
  for (int i = 0; i < current_added; i++) {
    if (index == constants::kNumIntraMpmExt) {
      break;
    }
    IntraMode mode = mpm[i];
    if (mode <= IntraMode::kDc) {
      continue;
    }
    IntraMode predictor =
      static_cast<IntraMode>(((mode + offset) % (max_modes - 2)) + 2);
    index += add_predictor_if_new(index, predictor);
    if (index == constants::kNumIntraMpmExt) {
      break;
    }
    predictor = static_cast<IntraMode>(((mode - 1) % (max_modes - 2)) + 2);
    index += add_predictor_if_new(index, predictor);
  }
  static const std::array<IntraAngle, 4> default_angles = {
    IntraAngle::kVertical, IntraAngle::kHorizontal,
    IntraAngle::kFirst, IntraAngle::kDiagonal
  };
  for (IntraAngle pred_angle : default_angles) {
    if (index == constants::kNumIntraMpmExt) {
      break;
    }
    IntraMode predictor = Convert(pred_angle);
    index += add_predictor_if_new(index, predictor);
  }
  assert(index == constants::kNumIntraMpmExt);
  return mpm;
}

void IntraPrediction::FillPredictorLumaDefault(const CodingUnit &cu,
                                               IntraPredictorLuma *mpm) const {
  constexpr YuvComponent comp = YuvComponent::kY;
  const int max_modes = !Restrictions::Get().disable_ext2_intra_67_modes ?
    kNbrIntraModesExt : kNbrIntraModes - 1;
  const int offset = !Restrictions::Get().disable_ext2_intra_67_modes ?
    kNbrIntraModesExt - 5 : kNbrIntraModes - 6;
  const CodingUnit *cu_left = cu.GetCodingUnitLeft();
  IntraMode left = IntraMode::kDc;
  if (cu_left && cu_left->IsIntra()) {
    left = cu_left->GetIntraMode(comp);
  }
  const CodingUnit *cu_above;
  if (Restrictions::Get().disable_ext_intra_unrestricted_predictor) {
    cu_above = cu.GetCodingUnitAboveIfSameCtu();
  } else {
    cu_above = cu.GetCodingUnitAbove();
  }
  IntraMode above = IntraMode::kDc;
  if (cu_above && cu_above->IsIntra()) {
    above = cu_above->GetIntraMode(comp);
  }
  if (left == above) {
    mpm->num_neighbor_modes = 1;
    if (left > IntraMode::kDc) {
      (*mpm)[0] = left;
      (*mpm)[1] =
        static_cast<IntraMode>(((left + offset) % (max_modes - 2)) + 2);
      (*mpm)[2] = static_cast<IntraMode>(((left - 1) % (max_modes - 2)) + 2);
    } else {
      (*mpm)[0] = IntraMode::kPlanar;
      (*mpm)[1] = IntraMode::kDc;
      (*mpm)[2] = Convert(IntraAngle::kVertical);
    }
  } else {
    mpm->num_neighbor_modes = 2;
    (*mpm)[0] = left;
    (*mpm)[1] = above;
    if (left > IntraMode::kPlanar && above > IntraMode::kPlanar) {
      (*mpm)[2] = IntraMode::kPlanar;
    } else {
      (*mpm)[2] = (left + above) < 2 ?
        Convert(IntraAngle::kVertical) : IntraMode::kDc;
    }
  }
}

IntraPredictorChroma
IntraPrediction::GetPredictorsChroma(IntraMode luma_mode) const {
  IntraPredictorChroma chroma_preds;
  chroma_preds[0] = IntraChromaMode::kPlanar;
  chroma_preds[1] = Convert(IntraChromaAngle::kVertical);
  chroma_preds[2] = Convert(IntraChromaAngle::kHorizontal);
  chroma_preds[3] = IntraChromaMode::kDc;
  if (!Restrictions::Get().disable_ext2_intra_chroma_from_luma) {
    chroma_preds[4] = IntraChromaMode::kLmChroma;
    chroma_preds[5] = Convert(IntraChromaAngle::kDmChroma);
  } else {
    chroma_preds[4] = Convert(IntraChromaAngle::kDmChroma);
    chroma_preds[5] = IntraChromaMode::kInvalid;
  }
  static_assert(kMaxNumIntraChromaModes == 6,
                "Only 6 chroma predictors supported");
  for (int i = 0; i < 4; i++) {
    if (static_cast<int>(chroma_preds[i]) == static_cast<int>(luma_mode)) {
      chroma_preds[i] = Convert(IntraChromaAngle::kVerticalPlus8);
      break;
    }
  }
  return chroma_preds;
}

IntraMode IntraPrediction::Convert(IntraAngle intra_dir) {
  static const std::array<uint8_t, kNbrIntraDirs> kIntraAngleToModeExtMap = {
    0, 1, 2, 4, 6, 8, 10, 12, 14, 16,   // [0, 10)
    18, 20, 22, 24, 26, 28, 30, 32,     // [10, 18)
    34, 36, 38, 40, 42, 44, 46, 48,     // [18, 26)
    50, 52, 54, 56, 58, 60, 62, 64, 66  // [26, 35)
  };
  if (Restrictions::Get().disable_ext2_intra_67_modes) {
    return static_cast<IntraMode>(intra_dir);
  }
  int intra_mode_ext = kIntraAngleToModeExtMap[static_cast<int>(intra_dir)];
  return static_cast<IntraMode>(intra_mode_ext);
}

IntraChromaMode IntraPrediction::Convert(IntraChromaAngle angle) {
  if (angle == IntraChromaAngle::kDmChroma) {
    return IntraChromaMode::kDmChroma;
  }
  return static_cast<IntraChromaMode>(Convert(static_cast<IntraAngle>(angle)));
}

bool IntraPrediction::UseFilteredRefSamples(const CodingUnit & cu,
                                            IntraMode intra_mode) {
  if (Restrictions::Get().disable_intra_ref_sample_filter) {
    return false;
  }
  static const std::array<int8_t, 8> kFilterRefThreshold = {
    0, 20, 10, 7, 1, 0, 10, 0   // 1, 2, 4, 8, 16, 32, 64, 128
  };
  static const std::array<int8_t, 8> kFilterRefThresholdExt = {
    0, 20, 20, 14, 2, 0, 20, 0   // 1, 2, 4, 8, 16, 32, 64, 128
  };
  static_assert(constants::kMaxBlockSize <= 128, "Only defined up to 128");
  int size = (util::SizeToLog2(cu.GetWidth(YuvComponent::kY)) +
              util::SizeToLog2(cu.GetHeight(YuvComponent::kY))) >> 1;
  int mode_diff = std::min(
    std::abs(intra_mode - Convert(IntraAngle::kHorizontal)),
    std::abs(intra_mode - Convert(IntraAngle::kVertical)));
  if (Restrictions::Get().disable_ext2_intra_67_modes) {
    return mode_diff > kFilterRefThreshold[size];
  }
  return mode_diff > kFilterRefThresholdExt[size];
}

void
IntraPrediction::PredIntraDC(int width, int height, bool dc_filter,
                             const Sample *ref_samples, ptrdiff_t ref_stride,
                             Sample *output_buffer, ptrdiff_t output_stride) {
  int sum = 0;
  for (int x = 0; x < width; x++) {
    sum += ref_samples[1 + x];
  }
  for (int y = 0; y < height; y++) {
    sum += ref_samples[ref_stride + y];
  }
  int total_size = width + height;
  Sample dc_val = static_cast<Sample>((sum + (total_size >> 1)) / total_size);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      output_buffer[x] = dc_val;
    }
    output_buffer += output_stride;
  }

  if (dc_filter && !Restrictions::Get().disable_intra_dc_post_filter) {
    for (int y = height - 1; y > 0; y--) {
      output_buffer -= output_stride;
      output_buffer[0] =
        (ref_samples[ref_stride + y] + 3 * output_buffer[0] + 2) >> 2;
    }
    output_buffer -= output_stride;
    for (int x = 1; x < width; x++) {
      output_buffer[x] = (ref_samples[1 + x] + 3 * output_buffer[x] + 2) >> 2;
    }
    // corner
    output_buffer[0] = (ref_samples[1] + ref_samples[ref_stride] +
                        2 * output_buffer[0] + 2) >> 2;
  }
}

void
IntraPrediction::PlanarPred(int width, int height,
                            const Sample *ref_samples, ptrdiff_t ref_stride,
                            Sample *output_buffer, ptrdiff_t output_stride) {
  const int width_log2 = util::SizeToLog2(width);
  const int height_log2 = util::SizeToLog2(height);
  const Sample *above = ref_samples + 1;
  const Sample *left = ref_samples + ref_stride;
  Sample topRight = ref_samples[1 + width];
  Sample bottomLeft = left[height];
  int shift = width_log2 + height_log2 + 1;
  int offset = 1 << (shift - 1);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int hor = (height - 1 - y) * above[x] + (y + 1) * bottomLeft;
      int ver = (width - 1 - x) * left[y] + (x + 1) * topRight;
      int pred = ((hor << width_log2) + (ver << height_log2) + offset) >> shift;
      output_buffer[x] = static_cast<Sample>(pred);
    }
    output_buffer += output_stride;
  }
}

void
IntraPrediction::AngularPred(int width, int height, IntraMode dir_mode,
                             bool filter,
                             const Sample *ref_samples, ptrdiff_t ref_stride,
                             Sample *output_buffer, ptrdiff_t output_stride) {
  Sample ref_flip_buffer[kRefSampleStride_ * 2];
  const bool is_horizontal = dir_mode < Convert(IntraAngle::kDiagonal);
  const Sample *ref_ptr = ref_samples;

  // Compute flipped reference samples
  if (is_horizontal) {
    const int top_size = width + height;
    assert(top_size > 0);
    ref_flip_buffer[0] = ref_samples[0];
    for (int i = 0; i < top_size; i++) {
      ref_flip_buffer[1 + i] = ref_samples[ref_stride + i];
      ref_flip_buffer[ref_stride + i] = ref_samples[1 + i];
    }
    ref_ptr = ref_flip_buffer;
    // For non-square blocks the output stride is not changed meaning that
    // flipped prediction will write outside of height*output_stride area
    // this is ok if we are only writing to a temporary buffer
    assert(height <= output_stride);
    std::swap(width, height);
  }

  // Get the prediction angle.
  int angle_offset = is_horizontal ?
    Convert(IntraAngle::kHorizontal) - dir_mode :
    dir_mode - Convert(IntraAngle::kVertical);
  int angle = !Restrictions::Get().disable_ext2_intra_67_modes ?
    kAngleTableExt[16 + angle_offset] : kAngleTable[8 + angle_offset];

  if (!angle) {
    // Speed-up for pure horizontal and vertical
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        output_buffer[y * output_stride + x] = ref_ptr[1 + x];
      }
    }
    if (filter && !Restrictions::Get().disable_intra_ver_hor_post_filter) {
      const Sample above_left = ref_ptr[0];
      const Sample above = ref_ptr[1];
      const Sample max_val = (1 << bitdepth_) - 1;
      for (int y = 0; y < height; y++) {
        int16_t val = above + ((ref_ptr[ref_stride + y] - above_left) >> 1);
        output_buffer[y * output_stride] = util::ClipBD(val, max_val);
      }
    }
  } else {
    Sample ref_line_buffer[kRefSampleStride_];
    const Sample *ref_line = ref_ptr + 1;
    auto *inv_angle_ptr = !Restrictions::Get().disable_ext2_intra_67_modes ?
      &kInvAngleTableExt[0] : &kInvAngleTable[0];

    // Project the side direction to the prediction line
    if (angle < 0) {
      int num_projected = -((height * angle) >> 5) - 1;
      Sample *ref_line_base_ptr = ref_line_buffer + num_projected + 1;
      // Most above samples are directly copied
      for (int i = 0; i < width + 1; i++) {
        ref_line_base_ptr[i - 1] = ref_ptr[i];
      }
      // Rest is projected from the other side edge to the prediction line
      int inv_angle = inv_angle_ptr[-angle_offset - 1];
      int inv_angle_sum = 128;
      for (int i = 0; i < num_projected; i++) {
        inv_angle_sum += inv_angle;
        ref_line_base_ptr[-2 - i] = ref_ptr[ref_stride +
          (inv_angle_sum >> 8) - 1];
      }
      ref_line = ref_line_base_ptr;
    }

    // Finally generate the prediction
    int angle_sum = 0;
    for (int y = 0; y < height; y++) {
      angle_sum += angle;
      int offset = angle_sum >> 5;
      int interpolate_weight = angle_sum & 31;
      if (interpolate_weight) {
        for (int x = 0; x < width; x++) {
          output_buffer[y * output_stride + x] = static_cast<Sample>(
            ((32 - interpolate_weight) * ref_line[offset + x] +
             interpolate_weight * ref_line[offset + x + 1] + 16) >> 5);
        }
      } else {
        for (int x = 0; x < width; x++) {
          output_buffer[y * output_stride + x] = ref_line[offset + x];
        }
      }
    }
    if (filter && std::abs(angle) <= 1 &&
        !Restrictions::Get().disable_ext2_intra_67_modes &&
        !Restrictions::Get().disable_intra_ver_hor_post_filter) {
      const Sample max_val = (1 << bitdepth_) - 1;
      for (int y = 0; y < height; y++) {
        int16_t val = output_buffer[y * output_stride] +
          ((ref_ptr[ref_stride + y] - ref_ptr[0]) >> 2);
        output_buffer[y * output_stride] = util::ClipBD(val, max_val);
      }
    }
  }

  // Flip back prediction for horizontal modes
  if (is_horizontal) {
    int short_side = std::min(width, height);
    for (int y = 0; y < short_side - 1; y++) {
      for (int x = y + 1; x < short_side; x++) {
        Sample tmp = output_buffer[y * output_stride + x];
        output_buffer[y * output_stride + x] =
          output_buffer[x * output_stride + y];
        output_buffer[x * output_stride + y] = tmp;
      }
    }
    if (width < height) {
      for (int y = 0; y < height - short_side; y++) {
        for (int x = 0; x < short_side; x++) {
          Sample tmp =
            output_buffer[(short_side + y) * output_stride + x];
          output_buffer[x*output_stride + short_side + y] = tmp;
        }
      }
    } else if (height < width) {
      for (int y = 0; y < short_side; y++) {
        for (int x = 0; x < width - short_side; x++) {
          Sample tmp =
            output_buffer[y*output_stride + short_side + x];
          output_buffer[(short_side + x)*output_stride + y] = tmp;
        }
      }
    }
  }
}

void IntraPrediction::PredLmChroma(const CodingUnit &cu, YuvComponent comp,
                                   const YuvPicture &rec_pic,
                                   SampleBuffer *output_buffer) {
  assert(!Restrictions::Get().disable_ext2_intra_chroma_from_luma);
  assert(!util::IsLuma(comp));
  const YuvComponent luma = YuvComponent::kY;
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const Sample max_val = (1 << rec_pic.GetBitdepth()) - 1;
  SampleBufferConst chroma_buffer =
    rec_pic.GetSampleBuffer(comp, cu.GetPosX(comp), cu.GetPosY(comp));
  SampleBufferConst luma_src_buffer =
    rec_pic.GetSampleBuffer(luma, cu.GetPosX(luma), cu.GetPosY(luma));
  SampleBuffer luma_sub_buffer = temp_pred_buffer_.Offset(1, 1);

  if (util::IsFirstChroma(comp)) {
    const int luma_width = cu.GetWidth(luma);
    const int luma_height = cu.GetHeight(luma);
    // Reuse downscaled luma samples for the second chroma component
    RescaleLuma(cu, luma_width, luma_height, luma_src_buffer, width, height,
                &luma_sub_buffer);
  }
  LmParams params = DeriveLmParams(cu, comp, chroma_buffer, luma_sub_buffer);
  output_buffer->AddLinearModel(width, height, luma_sub_buffer, params.scale,
                                params.shift, params.offset, 0, max_val);
}

IntraPrediction::LmParams
IntraPrediction::DeriveLmParams(const CodingUnit &cu, YuvComponent comp,
                                const SampleBufferConst &src_buffer,
                                const SampleBufferConst &ref_buffer) {
  static const int kModelQuantShift = 15;
  static const int kModelUpScaleShift = 13;
  static const int kModelMinResolutionShift = 5;
  static const int kModelPrecisionShift = 7;
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const bool has_above = cu.GetPosY(YuvComponent::kY) > 0;
  const bool has_left = cu.GetPosX(YuvComponent::kY) > 0;
  if (!has_above && !has_left) {
    return LmParams{ 0, 1 << (bitdepth_ - 1), 0 };
  }
  int sum_x = 0;
  int sum_y = 0;
  int sum_xx = 0;
  int sum_xy = 0;
  int nbr = 0;

  if (has_above) {
    const Sample *src = src_buffer.GetDataPtr() - src_buffer.GetStride();
    const Sample *ref = ref_buffer.GetDataPtr() - ref_buffer.GetStride();
    const int dx = has_left ? std::max(1, width / height) : 1;
    for (int x = 0; x < width; x += dx) {
      sum_x += ref[x];
      sum_y += src[x];
      sum_xx += ref[x] * ref[x];
      sum_xy += ref[x] * src[x];
      nbr++;
    }
  }
  if (has_left) {
    const Sample *src = src_buffer.GetDataPtr() - 1;
    const Sample *ref = ref_buffer.GetDataPtr() - 1;
    const int dy = has_above ? std::max(1, height / width) : 1;
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
  int size_shift = util::SizeToLog2(nbr);
  if (size_shift > kModelQuantShift - bitdepth_) {
    const int shift = size_shift + bitdepth_ - kModelQuantShift;
    sum_x = (sum_x + (1 << (shift - 1))) >> shift;
    sum_y = (sum_y + (1 << (shift - 1))) >> shift;
    sum_xx = (sum_xx + (1 << (shift - 1))) >> shift;
    sum_xy = (sum_xy + (1 << (shift - 1))) >> shift;
    size_shift -= shift;
  }
  assert(size_shift >= 0);
  const int avg_x = sum_x >> size_shift;
  const int avg_y = sum_y >> size_shift;
  const int x_frac = sum_x & ((1 << size_shift) - 1);
  const int y_frac = sum_y & ((1 << size_shift) - 1);
  // When using floating point, the scale parameter is derived from:
  // scale = (nbr * sum_xy - sum_x - sum_y) / (nbr * sum_xx - (sum_x * sum_x))
  // when compensating for integer truncation this becomes equivalent to:
  // scale ~= stddev_xy / stddev_xx
  const int stddev_xy = sum_xy - ((avg_x * avg_y) << size_shift)
    - (avg_x * y_frac) - (avg_y * x_frac);
  const int stddev_xx = sum_xx - ((avg_x * avg_x) << size_shift)
    - 2 * avg_x * x_frac;

  // Calculate model scale parameter through division,
  // numerator and denominator are scaled to avoid overflow and precision loss
  const int shift_xy = stddev_xy == 0 ?
    0 : std::max(0, util::Log2Floor(std::abs(stddev_xy)) - bitdepth_ + 2);
  const int shift_xx = stddev_xx == 0 ?
    0 : std::max(0, util::Log2Floor(std::abs(stddev_xx)) -
                 kModelMinResolutionShift);
  const int stddev_xy_shifted = stddev_xy >> shift_xy;
  const int shift_xx_shifted = stddev_xx >> shift_xx;
  const int total_shift = bitdepth_ + shift_xx + 4 + kModelPrecisionShift -
    kModelUpScaleShift - shift_xy;
  if (shift_xx_shifted < (1 << kModelMinResolutionShift)) {
    // prevent divide by zero
    return LmParams{ 0, avg_y, 0 };
  }
  int scale = stddev_xy_shifted * static_cast<uint32_t>(
    ((1 << (bitdepth_ + 4)) + (shift_xx_shifted / 2)) / shift_xx_shifted);
  scale = shift_xy >= 0 ? scale >> total_shift : scale << -total_shift;
  LmParams params;
  params.scale = (1 << kModelPrecisionShift) *
    util::Clip3(scale, -(1 << (kModelQuantShift - kModelPrecisionShift)),
    (1 << (kModelQuantShift - kModelPrecisionShift)) - 1);
  const int base_shift =
    util::Log2Floor(std::abs(params.scale) + (params.scale < 0 ? -1 : 0)) -
    (params.scale ? kModelMinResolutionShift : 0);
  params.shift = kModelUpScaleShift - base_shift;
  params.scale >>= base_shift;
  params.offset = avg_y - ((params.scale * avg_x) >> params.shift);
  return params;
}

IntraPrediction::NeighborState
IntraPrediction::DetermineNeighbors(const CodingUnit &cu, YuvComponent comp) {
  NeighborState neighbors;
  int x = cu.GetPosX(comp);
  int y = cu.GetPosY(comp);
  if (x > 0) {
    neighbors.has_left = true;
    neighbors.has_below_left = cu.GetCuSizeBelowLeft(comp);
  }
  if (y > 0) {
    neighbors.has_above = true;
    neighbors.has_above_right = cu.GetCuSizeAboveRight(comp);
  }
  if (x > 0 && y > 0) {
    neighbors.has_above_left = true;
  }
  return neighbors;
}

void IntraPrediction::ComputeRefSamples(int width, int height,
                                        const IntraPrediction::NeighborState
                                        &neighbors, const Sample *input,
                                        ptrdiff_t input_stride, Sample *output,
                                        ptrdiff_t output_stride) {
  const Sample dc_val = 1 << (bitdepth_ - 1);
  const int top_left_size = width;
  const int left_size = width + height;
  const int top_size = width + height;

  if (!neighbors.has_any()) {
    for (int x = 0; x < width + height + 1; x++) {
      output[x] = dc_val;
    }
    for (int y = 0; y < height + width; y++) {
      output[output_stride + y] = dc_val;
    }
    return;
  }

  if (neighbors.has_all(width, height)) {
    input -= input_stride + 1;
    for (int x = 0; x < width + height + 1; x++) {
      output[x] = input[x];
    }
    input += input_stride;
    for (int y = 0; y < height + width; y++) {
      output[output_stride + y] = input[y * input_stride];
    }
    return;
  }

  // Case when partial neighbors
  std::array<Sample, 5 * constants::kMaxBlockSize> line_buffer;

  // 1. Initialize with default value
  const int total_refs = left_size + top_size + top_left_size;
  for (int i = 0; i < total_refs; i++) {
    line_buffer[i] = dc_val;
  }

  // 2. Fill when available
  // Fill top-left sample
  const Sample *src_temp = input - input_stride - 1;
  Sample *line_temp = &line_buffer[0] + left_size;
  if (neighbors.has_above_left) {
    for (int i = 0; i < top_left_size; i++) {
      line_temp[i] = src_temp[0];
    }
  }

  // Fill left & below-left samples
  src_temp += input_stride;
  line_temp--;
  if (neighbors.has_left) {
    for (int i = 0; i < height; i++) {
      line_temp[-i] = src_temp[i*input_stride];
    }
    src_temp += height*input_stride;
    line_temp -= height;
    if (neighbors.has_below_left) {
      for (int i = 0; i < neighbors.has_below_left; i++) {
        line_temp[-i] = src_temp[i*input_stride];
      }
      // Out of picture bounds padding
      for (int i = neighbors.has_below_left; i < width; i++) {
        line_temp[-i] = line_temp[-neighbors.has_below_left + 1];
      }
    }
  }

  // Fill above & above-right samples
  src_temp = input - input_stride;
  line_temp = &line_buffer[left_size + top_left_size];
  if (neighbors.has_above) {
    for (int i = 0; i < width; i++) {
      line_temp[i] = src_temp[i];
    }
    if (neighbors.has_above_right) {
      for (int i = 0; i < neighbors.has_above_right; i++) {
        line_temp[width + i] = src_temp[width + i];
      }
      // Out of picture bounds padding
      for (int i = neighbors.has_above_right; i < height; i++) {
        line_temp[width + i] = line_temp[width + neighbors.has_above_right - 1];
      }
    }
  }
  if (!Restrictions::Get().disable_intra_ref_padding) {
    // 3a. Pad missing bottom left
    if (!neighbors.has_below_left) {
      Sample ref;
      if (neighbors.has_left) {
        ref = line_buffer[width];
      } else if (neighbors.has_above_left) {
        ref = line_buffer[left_size];
      } else if (neighbors.has_above) {
        ref = line_buffer[left_size + top_left_size];
      } else {
        ref = line_buffer[left_size + top_left_size + width];
      }
      for (int i = 0; i < width; i++) {
        line_buffer[i] = ref;
      }
    }

    // 3b. Pad any other missing
    if (!neighbors.has_left) {
      for (int i = 0; i < height; i++) {
        line_buffer[width + i] = line_buffer[width - 1];
      }
    }
    if (!neighbors.has_above_left) {
      for (int i = 0; i < top_left_size; i++) {
        line_buffer[left_size + i] = line_buffer[left_size - 1];
      }
    }
    if (!neighbors.has_above) {
      for (int i = 0; i < width; i++) {
        line_buffer[left_size + top_left_size + i] =
          line_buffer[left_size + top_left_size - 1];
      }
    }
    if (!neighbors.has_above_right) {
      for (int i = 0; i < height; i++) {
        line_buffer[left_size + top_left_size + width + i] =
          line_buffer[left_size + top_left_size + width - 1];
      }
    }
  }

  // 4. Copy processed samples
  // TODO(Dev) can be done with memcpy
  line_temp = &line_buffer[left_size + top_left_size - 1];
  for (int x = 0; x < top_size + 1; x++) {
    output[x] = line_temp[x];
  }
  line_temp = &line_buffer[left_size - 1];
  for (int y = 0; y < left_size; y++) {
    output[output_stride + y] = line_temp[-y];
  }
}

void IntraPrediction::FilterRefSamples(int width, int height,
                                       const Sample *src_ref, Sample *dst_ref,
                                       ptrdiff_t stride) {
  Sample above_left = src_ref[0];
  dst_ref[0] = ((above_left << 1) + src_ref[1] + src_ref[stride] + 2) >> 2;

  // above
  for (int x = 1; x < width + height; x++) {
    dst_ref[x] =
      ((src_ref[x] << 1) + src_ref[x - 1] + src_ref[x + 1] + 2) >> 2;
  }
  dst_ref[width + height] = src_ref[width + height];

  // left
  dst_ref[stride] =
    ((src_ref[stride] << 1) + above_left + src_ref[stride + 1] + 2) >> 2;
  for (int y = 1; y < height + width; y++) {
    dst_ref[stride + y] = ((src_ref[stride + y] << 1) + src_ref[stride + y - 1]
                           + src_ref[stride + y + 1] + 2) >> 2;
  }
  dst_ref[stride + height + width - 1] = src_ref[stride + height + width - 1];
}

void IntraPrediction::RescaleLuma(const CodingUnit &cu,
                                  int src_width, int src_height,
                                  const SampleBufferConst &src_buffer,
                                  int out_width, int out_height,
                                  SampleBuffer *out_buffer) {
  const bool has_above = cu.GetPosY(YuvComponent::kY) > 0;
  const bool has_left = cu.GetPosX(YuvComponent::kY) > 0;
  const ptrdiff_t src_stride = src_buffer.GetStride();
  const ptrdiff_t out_stride = out_buffer->GetStride();
  const int start_x = has_left ? 0 : 1;
  const int start_y = has_above ? -1 : 0;
  Sample *out = out_buffer->GetDataPtr();

  if ((src_width >> 1) == out_width && (src_height >> 1) == out_height) {
    assert(cu.GetPicData()->GetChromaFormat() == ChromaFormat::k420);
    const Sample *src =
      src_buffer.GetDataPtr() + (has_above ? -2 * src_stride : 0);
    if (has_left) {
      for (int y = start_y; y < out_height; y++) {
        int sum = src[-3] + 2 * src[-2] + src[-1] + src[-3 + src_stride] +
          2 * src[-2 + src_stride] + src[-1 + src_stride];
        out[y * out_stride - 1] = static_cast<Sample>((sum + 4) >> 3);
        src += 2 * src_stride;
      }
    } else {
      for (int y = start_y; y < out_height; y++) {
        int sum = src[0] + src[0 + src_stride];
        out[y * out_stride] = static_cast<Sample>((sum + 1) >> 1);
        src += 2 * src_stride;
      }
    }
    src = src_buffer.GetDataPtr() + (has_above ? -2 * src_stride : 0);
    for (int y = start_y; y < out_height; y++) {
      for (int x = start_x; x < out_width; x++) {
        int sum = src[2 * x - 1] + 2 * src[2 * x + 0] + src[2 * x + 1] +
          src[2 * x - 1 + src_stride] + 2 * src[2 * x + 0 + src_stride] +
          src[2 * x + 1 + src_stride];
        out[y * out_stride + x] = static_cast<Sample>((sum + 4) >> 3);
      }
      src += 2 * src_stride;
    }
  } else if (src_width == out_width && src_height == out_height) {
    assert(cu.GetPicData()->GetChromaFormat() == ChromaFormat::k444);
    const Sample *src = src_buffer.GetDataPtr();
    if (has_above) {
      for (int x = 0; x < out_width; x++) {
        out[-out_stride + x] = src[-src_stride + x];
      }
    }
    if (has_left) {
      for (int y = 0; y < out_height; y++) {
        out[y * out_stride - 1] = src[y * src_stride - 1];
      }
    }
    out_buffer->CopyFrom(out_width, out_height, src_buffer);
  } else {
    assert(cu.GetPicData()->GetChromaFormat() == ChromaFormat::k422);
    assert((src_width >> 1) == out_width);
    const Sample *src = src_buffer.GetDataPtr() - (has_above ? src_stride : 0);
    if (has_left) {
      for (int y = start_y; y < out_height; y++) {
        int sum = src[-3] + 2 * src[-2] + src[-1];
        out[y * out_stride - 1] = static_cast<Sample>((sum + 2) >> 2);
        src += src_stride;
      }
    } else {
      for (int y = start_y; y < out_height; y++) {
        int sum = src[0] + src[1];
        out[y * out_stride] = static_cast<Sample>((sum + 1) >> 1);
        src += src_stride;
      }
    }
    src = src_buffer.GetDataPtr() - (has_above ? src_stride : 0);
    for (int y = start_y; y < out_height; y++) {
      for (int x = start_x; x < out_width; x++) {
        int sum = src[2 * x - 1] + 2 * src[2 * x + 0] + src[2 * x + 1];
        out[y * out_stride + x] = static_cast<Sample>((sum + 2) >> 2);
      }
      src += src_stride;
    }
  }
}

}   // namespace xvc
