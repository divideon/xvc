/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_enc_lib/segment_header_writer.h"

#include <cassert>

#include "xvc_common_lib/restrictions.h"

namespace xvc {

void SegmentHeaderWriter::Write(SegmentHeader* segment_header,
                                BitWriter *bit_writer,
                                double framerate,
                                int open_gop) {
  bit_writer->Clear();
  bit_writer->WriteBits(33, 8);  // Nal Unit header with nal_unit_type == 16
  bit_writer->WriteBits(segment_header->codec_identifier, 24);
  bit_writer->WriteBits(segment_header->major_version, 16);
  bit_writer->WriteBits(segment_header->minor_version, 16);
  bit_writer->WriteBits(segment_header->pic_width, 16);
  bit_writer->WriteBits(segment_header->pic_height, 16);
  bit_writer->WriteBits(static_cast<uint8_t>(segment_header->chroma_format), 4);
  bit_writer->WriteBits(segment_header->internal_bitdepth - 8, 4);
  bit_writer->WriteBits(static_cast<uint32_t>(constants::kTimeScale
                                              / framerate), 24);
  bit_writer->WriteBits(
    static_cast<uint32_t>(segment_header->max_sub_gop_length), 8);
  bit_writer->WriteBit(1);  // shorter_sub_gops_allowed
  // The open gop flag is set to 0 if the tail pictures
  // do not predict from the Intra picture in the next segment.
  bit_writer->WriteBit(open_gop);
  bit_writer->WriteBits(segment_header->num_ref_pics, 4);
  assert(segment_header->deblock >= 0);
  assert(segment_header->deblock <= 3);
  bit_writer->WriteBits(segment_header->deblock, 2);
  if (segment_header->deblock == 3) {
    int d = constants::kDeblockOffsetBits;
    bit_writer->WriteBits(segment_header->beta_offset + (1 << (d - 1)), d);
    bit_writer->WriteBits(segment_header->tc_offset + (1 << (d - 1)), d);
  }

  auto &restr = Restrictions::Get();
  if (Restrictions::GetIntraRestrictions()) {
    bit_writer->WriteBit(1);  // intra_restrictions
    bit_writer->WriteBit(restr.disable_intra_ref_padding);
    bit_writer->WriteBit(restr.disable_intra_ref_sample_filter);
    bit_writer->WriteBit(restr.disable_intra_dc_post_filter);
    bit_writer->WriteBit(restr.disable_intra_ver_hor_post_filter);
    bit_writer->WriteBit(restr.disable_intra_planar);
    bit_writer->WriteBit(restr.disable_intra_mpm_prediction);
    bit_writer->WriteBit(restr.disable_intra_chroma_predictor);
  } else {
    bit_writer->WriteBit(0);  // intra_restrictions
  }
  if (Restrictions::GetInterRestrictions()) {
    bit_writer->WriteBit(1);  // inter_restrictions
    bit_writer->WriteBit(restr.disable_inter_mvp);
    bit_writer->WriteBit(restr.disable_inter_scaling_mvp);
    bit_writer->WriteBit(restr.disable_inter_tmvp_mvp);
    bit_writer->WriteBit(restr.disable_inter_tmvp_merge);
    bit_writer->WriteBit(restr.disable_inter_tmvp_ref_list_derivation);
    bit_writer->WriteBit(restr.disable_inter_merge_candidates);
    bit_writer->WriteBit(restr.disable_inter_merge_mode);
    bit_writer->WriteBit(restr.disable_inter_merge_bipred);
    bit_writer->WriteBit(restr.disable_inter_skip_mode);
    bit_writer->WriteBit(restr.disable_inter_chroma_subpel);
    bit_writer->WriteBit(restr.disable_inter_mvd_greater_than_flags);
    bit_writer->WriteBit(restr.disable_inter_bipred);
  } else {
    bit_writer->WriteBit(0);  // inter_restrictions
  }
  if (Restrictions::GetTransformRestrictions()) {
    bit_writer->WriteBit(1);  // transform_restrictions
    bit_writer->WriteBit(restr.disable_transform_adaptive_scan_order);
    bit_writer->WriteBit(restr.disable_transform_residual_greater_than_flags);
    bit_writer->WriteBit(restr.disable_transform_residual_greater2);
    bit_writer->WriteBit(restr.disable_transform_last_position);
    bit_writer->WriteBit(restr.disable_transform_root_cbf);
    bit_writer->WriteBit(restr.disable_transform_cbf);
    bit_writer->WriteBit(restr.disable_transform_subblock_csbf);
    bit_writer->WriteBit(restr.disable_transform_sign_hiding);
    bit_writer->WriteBit(restr.disable_transform_adaptive_exp_golomb);
  } else {
    bit_writer->WriteBit(0);  // transform_restrictions
  }
  if (Restrictions::GetCabacRestrictions()) {
    bit_writer->WriteBit(1);  // cabac_restrictions
    bit_writer->WriteBit(restr.disable_cabac_ctx_update);
    bit_writer->WriteBit(restr.disable_cabac_split_flag_ctx);
    bit_writer->WriteBit(restr.disable_cabac_skip_flag_ctx);
    bit_writer->WriteBit(restr.disable_cabac_inter_dir_ctx);
    bit_writer->WriteBit(restr.disable_cabac_subblock_csbf_ctx);
    bit_writer->WriteBit(restr.disable_cabac_coeff_sig_ctx);
    bit_writer->WriteBit(restr.disable_cabac_coeff_greater1_ctx);
    bit_writer->WriteBit(restr.disable_cabac_coeff_greater2_ctx);
    bit_writer->WriteBit(restr.disable_cabac_coeff_last_pos_ctx);
    bit_writer->WriteBit(restr.disable_cabac_init_per_pic_type);
    bit_writer->WriteBit(restr.disable_cabac_init_per_qp);
  } else {
    bit_writer->WriteBit(0);  // cabac_restrictions
  }
  if (Restrictions::GetDeblockRestrictions()) {
    bit_writer->WriteBit(1);  // deblock_restrictions
    bit_writer->WriteBit(restr.disable_deblock_strong_filter);
    bit_writer->WriteBit(restr.disable_deblock_weak_filter);
    bit_writer->WriteBit(restr.disable_deblock_chroma_filter);
    bit_writer->WriteBit(restr.disable_deblock_boundary_strength_zero);
    bit_writer->WriteBit(restr.disable_deblock_boundary_strength_one);
    bit_writer->WriteBit(restr.disable_deblock_initial_sample_decision);
    bit_writer->WriteBit(restr.disable_deblock_weak_sample_decision);
    bit_writer->WriteBit(restr.disable_deblock_two_samples_weak_filter);
    bit_writer->WriteBit(restr.disable_deblock_depending_on_qp);
  } else {
    bit_writer->WriteBit(0);  // deblock_restrictions
  }
  if (Restrictions::GetExtRestrictions()) {
    bit_writer->WriteBit(1);
    bit_writer->WriteBit(restr.disable_ext);
    bit_writer->WriteBit(restr.disable_ext_tmvp_full_resolution);
    bit_writer->WriteBit(restr.disable_ext_tmvp_exclude_intra_from_ref_list);
    bit_writer->WriteBit(restr.disable_ext_ref_list_l0_trim);
    bit_writer->WriteBit(restr.disable_ext_implicit_partition_type);
    bit_writer->WriteBit(restr.disable_ext_cabac_alt_split_flag_ctx);
    bit_writer->WriteBit(restr.disable_ext_cabac_alt_inter_dir_ctx);
    bit_writer->WriteBit(restr.disable_ext_cabac_alt_last_pos_ctx);
    bit_writer->WriteBit(restr.disable_ext_two_cu_trees);
    bit_writer->WriteBit(restr.disable_ext_transform_size_64);
  } else {
    bit_writer->WriteBit(0);  // ext_restrictions
  }
  bit_writer->PadZeroBits();
}

}   // namespace xvc
