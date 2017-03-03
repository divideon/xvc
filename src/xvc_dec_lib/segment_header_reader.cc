/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_dec_lib/segment_header_reader.h"

#include "xvc_common_lib/restrictions.h"

namespace xvc {

Decoder::State SegmentHeaderReader::Read(SegmentHeader* segment_header,
                                         BitReader *bitreader,
                                         SegmentNum segment_counter) {
  segment_header->major_version = bitreader->ReadBits(16);
  if (segment_header->major_version > constants::kXvcMajorVersion) {
    return Decoder::State::kDecoderVersionTooLow;
  }
  segment_header->minor_version = bitreader->ReadBits(16);
  segment_header->pic_width = bitreader->ReadBits(16);
  segment_header->pic_height = bitreader->ReadBits(16);
  segment_header->chroma_format = ChromaFormat(bitreader->ReadBits(4));
  segment_header->internal_bitdepth = bitreader->ReadBits(4) + 8;
  if (segment_header->internal_bitdepth >
      static_cast<int>(8 * sizeof(Sample))) {
    return Decoder::State::kBitstreamBitdepthTooHigh;
  }
  segment_header->bitstream_ticks = bitreader->ReadBits(24);
  segment_header->base_qp = bitreader->ReadBits(7) - 64;
  segment_header->max_sub_gop_length = bitreader->ReadBits(8);

  bitreader->ReadBit();  // shorter_sub_gops_allowed
  segment_header->open_gop = bitreader->ReadBit() == 1;
  segment_header->deblock = bitreader->ReadBits(2);  // deblock
  if (segment_header->deblock == 3) {
    int d = constants::kDeblockOffsetBits;
    segment_header->beta_offset = bitreader->ReadBits(d) - (1 << (d - 1));
    segment_header->tc_offset = bitreader->ReadBits(d) - (1 << (d - 1));
  }
  bitreader->ReadBits(4);  // num_ref_pic_r0
  bitreader->ReadBits(4);  // num_ref_pic_r1

  // This is the only place where Restrictions::GetRW is allowed to be called.
  auto &restr = Restrictions::GetRW();

  int intra_restrictions = bitreader->ReadBit();
  if (intra_restrictions) {
    // Override the value of the restriction flags only if the flag is
    // set to true in the bitstream.
    if (bitreader->ReadBit()) {
      restr.disable_intra_ref_padding = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_intra_ref_sample_filter = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_intra_dc_post_filter = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_intra_ver_hor_post_filter = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_intra_planar = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_intra_mpm_prediction = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_intra_chroma_predictor = true;
    }
  }

  int inter_restrictions = bitreader->ReadBit();
  if (inter_restrictions) {
    // Override the value of the restriction flags only if the flag is
    // set to true in the bitstream.
    if (bitreader->ReadBit()) {
      restr.disable_inter_mvp = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_inter_scaling_mvp = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_inter_merge_candidates = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_inter_merge_mode = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_inter_skip_mode = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_inter_chroma_subpel = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_inter_mvd_greater_than_flags = true;
    }
  }

  int transform_restrictions = bitreader->ReadBit();
  if (transform_restrictions) {
    // Override the value of the restriction flags only if the flag is
    // set to true in the bitstream.
    if (bitreader->ReadBit()) {
      restr.disable_transform_adaptive_scan_order = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_transform_residual_greater_than_flags = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_transform_last_position = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_transform_cbf = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_transform_subblock_csbf = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_transform_adaptive_exp_golomb = true;
    }
  }

  int cabac_restrictions = bitreader->ReadBit();
  if (cabac_restrictions) {
    // Override the value of the restriction flags only if the flag is
    // set to true in the bitstream.
    if (bitreader->ReadBit()) {
      restr.disable_cabac_ctx_update = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_cabac_split_flag_ctx = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_cabac_skip_flag_ctx = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_cabac_subblock_csbf_ctx = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_cabac_coeff_sig_ctx = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_cabac_coeff_greater1_ctx = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_cabac_coeff_greater2_ctx = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_cabac_coeff_last_pos_ctx = true;
    }
  }

  int deblock_restrictions = bitreader->ReadBit();
  if (deblock_restrictions) {
    // Override the value of the restriction flags only if the flag is
    // set to true in the bitstream.
    if (bitreader->ReadBit()) {
      restr.disable_deblock_strong_filter = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_deblock_weak_filter = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_deblock_chroma_filter = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_deblock_boundary_strength_zero = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_deblock_boundary_strength_one = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_deblock_initial_sample_decision = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_deblock_two_samples_weak_filter = true;
    }
    if (bitreader->ReadBit()) {
      restr.disable_deblock_depending_on_qp = true;
    }
  }

  segment_header->soc = segment_counter;
  return Decoder::State::kSegmentHeaderDecoded;
}

}   // namespace xvc
