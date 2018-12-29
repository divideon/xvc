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

#include "xvc_enc_lib/segment_header_writer.h"

#include <cassert>

#include "xvc_common_lib/picture_types.h"
#include "xvc_common_lib/restrictions.h"

namespace xvc {

void SegmentHeaderWriter::Write(const SegmentHeader &segment_header,
                                BitWriter *bit_writer, double framerate) {
  bit_writer->WriteBits(1, 1);  // xvc_bit_one
  bit_writer->WriteBits(0, 1);  // nal_rfe
  bit_writer->WriteBits(static_cast<uint8_t>(NalUnitType::kSegmentHeader), 5);
  bit_writer->WriteBits(1, 1);  // nal_rfl
  bit_writer->WriteBits(segment_header.codec_identifier, 24);
  bit_writer->WriteBits(segment_header.major_version, 16);
  bit_writer->WriteBits(segment_header.minor_version, 16);
  bit_writer->WriteBits(segment_header.GetOutputWidth(),
                        constants::kPicSizeBits);
  bit_writer->WriteBits(segment_header.GetOutputHeight(),
                        constants::kPicSizeBits);
  bit_writer->WriteBits(static_cast<uint8_t>(segment_header.chroma_format), 4);
  bit_writer->WriteBits(segment_header.internal_bitdepth - 8, 4);
  bit_writer->WriteBits(static_cast<uint32_t>(constants::kTimeScale
                                              / framerate), 24);
  bit_writer->WriteBits(
    static_cast<uint32_t>(segment_header.max_sub_gop_length), 8);

  bit_writer->WriteBits(static_cast<uint8_t>(segment_header.color_matrix), 3);
  // The open gop flag is set to 0 if the tail pictures
  // do not predict from the Intra picture in the next segment.
  bit_writer->WriteBit(segment_header.open_gop ? 1 : 0);
  bit_writer->WriteBits(segment_header.num_ref_pics, 4);
  static_assert(constants::kMaxBinarySplitDepth < (1 << 2),
                "max binary split depth signaling");
  bit_writer->WriteBits(segment_header.max_binary_split_depth, 2);
  bit_writer->WriteBits(static_cast<uint32_t>(segment_header.checksum_mode),
                        1);

  assert(segment_header.adaptive_qp >= 0 && segment_header.adaptive_qp <= 3);
  bit_writer->WriteBits(segment_header.adaptive_qp, 2);
  bit_writer->WriteBits(segment_header.chroma_qp_offset_table, 2);
  int chroma_qp_offsets = (segment_header.chroma_qp_offset_u != 0 ||
                           segment_header.chroma_qp_offset_v != 0) ? 1 : 0;
  bit_writer->WriteBits(chroma_qp_offsets, 1);
  if (chroma_qp_offsets) {
    int d = constants::kChromaOffsetBits;
    bit_writer->WriteBits(segment_header.chroma_qp_offset_u + (1 << (d - 1)),
                          d);
    bit_writer->WriteBits(segment_header.chroma_qp_offset_v + (1 << (d - 1)),
                          d);
  }

  int deblocking_mode = static_cast<int>(segment_header.deblocking_mode);
  assert(deblocking_mode >= 0 && deblocking_mode <= 3);
  bit_writer->WriteBits(deblocking_mode, 2);
  if (segment_header.deblocking_mode == DeblockingMode::kCustom) {
    int d = constants::kDeblockOffsetBits;
    bit_writer->WriteBits(segment_header.beta_offset + (1 << (d - 1)), d);
    bit_writer->WriteBits(segment_header.tc_offset + (1 << (d - 1)), d);
  }
  // Checking major version on encoder side is only needed for unit tests
  if (segment_header.major_version > 1) {
    bit_writer->WriteBit(segment_header.low_delay ? 1 : 0);
    bit_writer->WriteBit(segment_header.leading_pictures > 0 ? 1 : 0);
    bit_writer->WriteBit(segment_header.source_padding ? 1 : 0);
  }

  WriteRestrictions(segment_header.restrictions, bit_writer);
  bit_writer->PadZeroBits();
}

void SegmentHeaderWriter::WriteRestrictions(const Restrictions &restr,
                                            BitWriter *bit_writer) {
  if (restr.GetIntraRestrictions()) {
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
  if (restr.GetInterRestrictions()) {
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
  if (restr.GetTransformRestrictions()) {
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
  if (restr.GetCabacRestrictions()) {
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
  if (restr.GetDeblockRestrictions()) {
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
  if (restr.GetHighLevelRestrictions()) {
    bit_writer->WriteBit(1);  // high_level_restrictions
    bit_writer->WriteBit(restr.disable_high_level_default_checksum_method);
  } else {
    bit_writer->WriteBit(0);  // high_level_restrictions
  }
  if (restr.GetExtRestrictions()) {
    bit_writer->WriteBit(1);  // ext_restrictions
    bit_writer->WriteBit(restr.disable_ext_sink);
    bit_writer->WriteBit(restr.disable_ext_implicit_last_ctu);
    bit_writer->WriteBit(restr.disable_ext_tmvp_full_resolution);
    bit_writer->WriteBit(restr.disable_ext_tmvp_exclude_intra_from_ref_list);
    bit_writer->WriteBit(restr.disable_ext_ref_list_l0_trim);
    bit_writer->WriteBit(restr.disable_ext_implicit_partition_type);
    bit_writer->WriteBit(restr.disable_ext_cabac_alt_split_flag_ctx);
    bit_writer->WriteBit(restr.disable_ext_cabac_alt_inter_dir_ctx);
    bit_writer->WriteBit(restr.disable_ext_cabac_alt_last_pos_ctx);
    bit_writer->WriteBit(restr.disable_ext_two_cu_trees);
    bit_writer->WriteBit(restr.disable_ext_transform_size_64);
    bit_writer->WriteBit(restr.disable_ext_intra_unrestricted_predictor);
    bit_writer->WriteBit(restr.disable_ext_deblock_subblock_size_4);
  } else {
    bit_writer->WriteBit(0);  // ext_restrictions
  }
  if (restr.GetExt2Restrictions()) {
    bit_writer->WriteBit(1);  // ext2_restrictions
    bit_writer->WriteBit(restr.disable_ext2_intra_67_modes);
    bit_writer->WriteBit(restr.disable_ext2_intra_6_predictors);
    bit_writer->WriteBit(restr.disable_ext2_intra_chroma_from_luma);
    bit_writer->WriteBit(restr.disable_ext2_inter_adaptive_fullpel_mv);
    bit_writer->WriteBit(restr.disable_ext2_inter_affine);
    bit_writer->WriteBit(restr.disable_ext2_inter_affine_merge);
    bit_writer->WriteBit(restr.disable_ext2_inter_affine_mvp);
    bit_writer->WriteBit(restr.disable_ext2_inter_bipred_l1_mvd_zero);
    bit_writer->WriteBit(restr.disable_ext2_inter_high_precision_mv);
    bit_writer->WriteBit(restr.disable_ext2_inter_local_illumination_comp);
    bit_writer->WriteBit(restr.disable_ext2_transform_skip);
    bit_writer->WriteBit(restr.disable_ext2_transform_high_precision);
    bit_writer->WriteBit(restr.disable_ext2_transform_select);
    bit_writer->WriteBit(restr.disable_ext2_transform_dst);
    bit_writer->WriteBit(restr.disable_ext2_cabac_alt_residual_ctx);
  } else {
    bit_writer->WriteBit(0);  // ext2_restrictions
  }
}

}   // namespace xvc
