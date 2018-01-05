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
#include <cstdlib>
#include <utility>

#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

const int8_t IntraPrediction::kAngleTable_[17] = {
  -32, -26, -21, -17, -13, -9, -5, -2, 0, 2, 5, 9, 13, 17, 21, 26, 32
};

const int16_t IntraPrediction::kInvAngleTable_[8] = {
  4096, 1638, 910, 630, 482, 390, 315, 256
};

void IntraPrediction::Predict(IntraMode intra_mode, const CodingUnit &cu,
                              YuvComponent comp,
                              const Sample *input_pic, ptrdiff_t input_stride,
                              Sample *output_buffer, ptrdiff_t output_stride) {
  IntraPrediction::State ref_state =
    ComputeReferenceState(cu, comp, input_pic, input_stride);
  Predict(intra_mode, cu, comp, ref_state, output_buffer, output_stride);
}

void IntraPrediction::Predict(IntraMode intra_mode, const CodingUnit &cu,
                              YuvComponent comp, const State &ref_state,
                              Sample *output_buffer, ptrdiff_t output_stride) {
  const Sample *ref_samples = &ref_state.ref_samples[0];
  if (util::IsLuma(comp)) {
    bool filtered_ref = UseFilteredRefSamples(cu, intra_mode);
    ref_samples =
      filtered_ref ? &ref_state.ref_filtered[0] : &ref_state.ref_samples[0];
  }
  bool post_filter = comp == kY && cu.GetWidth(comp) <= 16 &&
    cu.GetHeight(comp) <= 16;
  IntraMode mode = (Restrictions::Get().disable_intra_planar &&
                    intra_mode == kPlanar) ? kDc : intra_mode;
  switch (mode) {
    case kPlanar:
      PlanarPred(cu.GetWidth(comp), cu.GetHeight(comp), ref_samples,
                 kRefSampleStride_, output_buffer, output_stride);
      break;

    case kDc:
      PredIntraDC(cu.GetWidth(comp), cu.GetHeight(comp), post_filter,
                  &ref_state.ref_samples[0], kRefSampleStride_, output_buffer,
                  output_stride);
      break;

    default:
      AngularPred(cu.GetWidth(comp), cu.GetHeight(comp), intra_mode,
                  post_filter, ref_samples, kRefSampleStride_, output_buffer,
                  output_stride);
      break;
  }
}

IntraPrediction::State
IntraPrediction::ComputeReferenceState(const CodingUnit &cu, YuvComponent comp,
                                       const Sample *input_pic,
                                       ptrdiff_t input_stride) {
  IntraPrediction::State ref_state;
  NeighborState neighbors = DetermineNeighbors(cu, comp);
  ComputeRefSamples(cu.GetWidth(comp), cu.GetHeight(comp), neighbors,
                    input_pic, input_stride, &ref_state.ref_samples[0],
                    kRefSampleStride_);

  // TODO(Dev) optimize decoder by skipping filtering depending on intra mode
  if (util::IsLuma(comp)) {
    FilterRefSamples(cu.GetWidth(comp), cu.GetHeight(comp),
                     &ref_state.ref_samples[0], &ref_state.ref_filtered[0],
                     kRefSampleStride_);
  }
  return ref_state;
}

IntraPredictorLuma
IntraPrediction::GetPredictorLuma(const CodingUnit &cu) const {
  const CodingUnit *cu_left = cu.GetCodingUnitLeft();
  IntraMode left = IntraMode::kDc;
  if (cu_left && cu_left->IsIntra()) {
    left = cu_left->GetIntraMode(YuvComponent::kY);
  }
  const CodingUnit *cu_above;
  if (Restrictions::Get().disable_ext_intra_unrestricted_predictor) {
    cu_above = cu.GetCodingUnitAboveIfSameCtu();
  } else {
    cu_above = cu.GetCodingUnitAbove();
  }
  IntraMode above = IntraMode::kDc;
  if (cu_above && cu_above->IsIntra()) {
    above = cu_above->GetIntraMode(YuvComponent::kY);
  }
  IntraPredictorLuma mpm;
  if (Restrictions::Get().disable_intra_mpm_prediction) {
    mpm.num_neighbor_modes = 1;
    mpm[0] = IntraMode::kPlanar;
    mpm[1] = IntraMode::kDc;
    mpm[2] = IntraMode::kVertical;
    return mpm;
  }
  if (left == above) {
    mpm.num_neighbor_modes = 1;
    if (left > IntraMode::kDc) {
      mpm[0] = left;
      mpm[1] = static_cast<IntraMode>(((left + 29) % 32) + 2);
      mpm[2] = static_cast<IntraMode>(((left - 1) % 32) + 2);
    } else {
      mpm[0] = IntraMode::kPlanar;
      mpm[1] = IntraMode::kDc;
      mpm[2] = IntraMode::kVertical;
    }
  } else {
    mpm.num_neighbor_modes = 2;
    mpm[0] = left;
    mpm[1] = above;
    if (left > IntraMode::kPlanar && above > IntraMode::kPlanar) {
      mpm[2] = IntraMode::kPlanar;
    } else {
      mpm[2] = (left + above) < 2 ? IntraMode::kVertical : IntraMode::kDc;
    }
  }
  return mpm;
}

IntraPredictorChroma
IntraPrediction::GetPredictorsChroma(IntraMode luma_mode) const {
  IntraPredictorChroma chroma_preds;
  chroma_preds[0] = IntraChromaMode::kPlanar;
  chroma_preds[1] = IntraChromaMode::kVertical;
  chroma_preds[2] = IntraChromaMode::kHorizontal;
  chroma_preds[3] = IntraChromaMode::kDc;
  chroma_preds[4] = IntraChromaMode::kDmChroma;
  for (int i = 0; i < static_cast<int>(chroma_preds.size()) - 1; i++) {
    if (static_cast<int>(chroma_preds[i]) == luma_mode) {
      chroma_preds[i] = IntraChromaMode::kVerticalPlus8;
      break;
    }
  }
  return chroma_preds;
}

bool IntraPrediction::UseFilteredRefSamples(const CodingUnit & cu,
                                            IntraMode intra_mode) {
  if (Restrictions::Get().disable_intra_ref_sample_filter) {
    return false;
  }
  static const std::array<int8_t, 8> kFilterRefThreshold = {
      /*1x1:*/0, /*2x2:*/20, /*4x4:*/10, /*8x8:*/7, /*16x16:*/1,
      /*32x32:*/ 0, /*64x64:*/ 10, /*128x128:*/ 0
  };
  int size = (util::SizeToLog2(cu.GetWidth(YuvComponent::kY)) +
              util::SizeToLog2(cu.GetHeight(YuvComponent::kY))) >> 1;
  int mode_diff = std::min(abs(intra_mode - IntraMode::kHorizontal),
                           abs(intra_mode - IntraMode::kVertical));
  return mode_diff > kFilterRefThreshold[size];
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
  const bool is_horizontal = dir_mode < 18;
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
    IntraMode::kHorizontal - dir_mode : dir_mode - IntraMode::kVertical;
  int angle = kAngleTable_[8 + angle_offset];

  if (!angle) {
    // Speed-up for pure horizontal and vertical
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        output_buffer[y * output_stride + x] = ref_ptr[1 + x];
      }
    }
    if (filter && !Restrictions::Get().disable_intra_ver_hor_post_filter) {
      Sample above_left = ref_ptr[0];
      Sample above = ref_ptr[1];
      Sample max_val = (1 << bitdepth_) - 1;
      for (int y = 0; y < height; y++) {
        int16_t val = above + ((ref_ptr[ref_stride + y] - above_left) >> 1);
        output_buffer[y * output_stride] = util::ClipBD(val, max_val);
      }
    }
  } else {
    Sample ref_line_buffer[kRefSampleStride_];
    const Sample *ref_line = ref_ptr + 1;

    // Project the side direction to the prediction line
    if (angle < 0) {
      int num_projected = -((height * angle) >> 5) - 1;
      Sample *ref_line_base_ptr = ref_line_buffer + num_projected + 1;
      // Most above samples are directly copied
      for (int i = 0; i < width + 1; i++) {
        ref_line_base_ptr[i - 1] = ref_ptr[i];
      }
      // Rest is projected from the other side edge to the prediction line
      int inv_angle = kInvAngleTable_[-angle_offset - 1];
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

}   // namespace xvc
