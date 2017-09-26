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

#include "xvc_dec_lib/segment_header_reader.h"

#include "xvc_common_lib/restrictions.h"

namespace xvc {

Decoder::State SegmentHeaderReader::Read(SegmentHeader* segment_header,
                                         BitReader *bit_reader,
                                         SegmentNum segment_counter) {
  segment_header->codec_identifier = bit_reader->ReadBits(24);
  if (segment_header->codec_identifier != constants::kXvcCodecIdentifier) {
    return Decoder::State::kNoSegmentHeader;
  }
  segment_header->major_version = bit_reader->ReadBits(16);
  if (segment_header->major_version > constants::kXvcMajorVersion) {
    return Decoder::State::kDecoderVersionTooLow;
  }
  segment_header->minor_version = bit_reader->ReadBits(16);
  if (!SupportedBitstreamVersion(segment_header->major_version,
                                 segment_header->minor_version)) {
    return Decoder::State::kBitstreamVersionTooLow;
  }
  segment_header->SetWidth(bit_reader->ReadBits(16));
  segment_header->SetHeight(bit_reader->ReadBits(16));
  segment_header->chroma_format = ChromaFormat(bit_reader->ReadBits(4));
  segment_header->internal_bitdepth = bit_reader->ReadBits(4) + 8;
  if (segment_header->internal_bitdepth >
      static_cast<int>(8 * sizeof(Sample))) {
    return Decoder::State::kBitstreamBitdepthTooHigh;
  }
  segment_header->bitstream_ticks = bit_reader->ReadBits(24);
  segment_header->max_sub_gop_length = bit_reader->ReadBits(8);

  segment_header->color_matrix = ColorMatrix(bit_reader->ReadBits(3));
  segment_header->open_gop = bit_reader->ReadBit() != 0;
  segment_header->num_ref_pics = bit_reader->ReadBits(4);
  static_assert(constants::kMaxBinarySplitDepth < (1 << 2),
                "max binary split depth signaling");
  segment_header->max_binary_split_depth = bit_reader->ReadBits(2);
  segment_header->checksum_mode = Checksum::Mode(bit_reader->ReadBits(1));
  segment_header->adaptive_qp = bit_reader->ReadBits(2);
  segment_header->chroma_qp_offset_table = bit_reader->ReadBits(2);
  int chroma_qp_offsets = bit_reader->ReadBit();
  if (chroma_qp_offsets) {
    int d = constants::kChromaOffsetBits;
    segment_header->chroma_qp_offset_u =
      bit_reader->ReadBits(d) - (1 << (d - 1));
    segment_header->chroma_qp_offset_v =
      bit_reader->ReadBits(d) - (1 << (d - 1));
  }
  segment_header->deblock = bit_reader->ReadBits(2);
  if (segment_header->deblock == 3) {
    int d = constants::kDeblockOffsetBits;
    segment_header->beta_offset = bit_reader->ReadBits(d) - (1 << (d - 1));
    segment_header->tc_offset = bit_reader->ReadBits(d) - (1 << (d - 1));
  }

  auto &restr = segment_header->restrictions;
  restr = Restrictions();

  // Note! Override the value of the restriction flags only if the flag is
  // set to true in the bitstream.

  int intra_restrictions = bit_reader->ReadBit();
  if (intra_restrictions) {
    restr.disable_intra_ref_padding |= !!bit_reader->ReadBit();
    restr.disable_intra_ref_sample_filter |= !!bit_reader->ReadBit();
    restr.disable_intra_dc_post_filter |= !!bit_reader->ReadBit();
    restr.disable_intra_ver_hor_post_filter |= !!bit_reader->ReadBit();
    restr.disable_intra_planar |= !!bit_reader->ReadBit();
    restr.disable_intra_mpm_prediction |= !!bit_reader->ReadBit();
    restr.disable_intra_chroma_predictor |= !!bit_reader->ReadBit();
  }

  int inter_restrictions = bit_reader->ReadBit();
  if (inter_restrictions) {
    restr.disable_inter_mvp |= !!bit_reader->ReadBit();
    restr.disable_inter_scaling_mvp |= !!bit_reader->ReadBit();
    restr.disable_inter_tmvp_mvp |= !!bit_reader->ReadBit();
    restr.disable_inter_tmvp_merge |= !!bit_reader->ReadBit();
    restr.disable_inter_tmvp_ref_list_derivation |= !!bit_reader->ReadBit();
    restr.disable_inter_merge_candidates |= !!bit_reader->ReadBit();
    restr.disable_inter_merge_mode |= !!bit_reader->ReadBit();
    restr.disable_inter_merge_bipred |= !!bit_reader->ReadBit();
    restr.disable_inter_skip_mode |= !!bit_reader->ReadBit();
    restr.disable_inter_chroma_subpel |= !!bit_reader->ReadBit();
    restr.disable_inter_mvd_greater_than_flags |= !!bit_reader->ReadBit();
    restr.disable_inter_bipred |= !!bit_reader->ReadBit();
  }

  int transform_restrictions = bit_reader->ReadBit();
  if (transform_restrictions) {
    restr.disable_transform_adaptive_scan_order |= !!bit_reader->ReadBit();
    restr.disable_transform_residual_greater_than_flags |=
      !!bit_reader->ReadBit();
    restr.disable_transform_residual_greater2 |= !!bit_reader->ReadBit();
    restr.disable_transform_last_position |= !!bit_reader->ReadBit();
    restr.disable_transform_root_cbf |= !!bit_reader->ReadBit();
    restr.disable_transform_cbf |= !!bit_reader->ReadBit();
    restr.disable_transform_subblock_csbf |= !!bit_reader->ReadBit();
    if (segment_header->major_version > 1) {
      restr.disable_transform_skip |= !!bit_reader->ReadBit();
    }
    restr.disable_transform_sign_hiding |= !!bit_reader->ReadBit();
    restr.disable_transform_adaptive_exp_golomb |= !!bit_reader->ReadBit();
  }

  int cabac_restrictions = bit_reader->ReadBit();
  if (cabac_restrictions) {
    restr.disable_cabac_ctx_update |= !!bit_reader->ReadBit();
    restr.disable_cabac_split_flag_ctx |= !!bit_reader->ReadBit();
    restr.disable_cabac_skip_flag_ctx |= !!bit_reader->ReadBit();
    restr.disable_cabac_inter_dir_ctx |= !!bit_reader->ReadBit();
    restr.disable_cabac_subblock_csbf_ctx |= !!bit_reader->ReadBit();
    restr.disable_cabac_coeff_sig_ctx |= !!bit_reader->ReadBit();
    restr.disable_cabac_coeff_greater1_ctx |= !!bit_reader->ReadBit();
    restr.disable_cabac_coeff_greater2_ctx |= !!bit_reader->ReadBit();
    restr.disable_cabac_coeff_last_pos_ctx |= !!bit_reader->ReadBit();
    restr.disable_cabac_init_per_pic_type |= !!bit_reader->ReadBit();
    restr.disable_cabac_init_per_qp |= !!bit_reader->ReadBit();
  }

  int deblock_restrictions = bit_reader->ReadBit();
  if (deblock_restrictions) {
    restr.disable_deblock_strong_filter |= !!bit_reader->ReadBit();
    restr.disable_deblock_weak_filter |= !!bit_reader->ReadBit();
    restr.disable_deblock_chroma_filter |= !!bit_reader->ReadBit();
    restr.disable_deblock_boundary_strength_zero |= !!bit_reader->ReadBit();
    restr.disable_deblock_boundary_strength_one |= !!bit_reader->ReadBit();
    restr.disable_deblock_initial_sample_decision |= !!bit_reader->ReadBit();
    restr.disable_deblock_weak_sample_decision |= !!bit_reader->ReadBit();
    restr.disable_deblock_two_samples_weak_filter |= !!bit_reader->ReadBit();
    restr.disable_deblock_depending_on_qp |= !!bit_reader->ReadBit();
  }

  int high_level_restrictions = bit_reader->ReadBit();
  if (high_level_restrictions) {
    restr.disable_high_level_default_checksum_method |=
      !!bit_reader->ReadBit();
  }

  int ext_restrictions = bit_reader->ReadBit();
  if (ext_restrictions) {
    restr.disable_ext_sink |= !!bit_reader->ReadBit();
    restr.disable_ext_implicit_last_ctu |= !!bit_reader->ReadBit();
    restr.disable_ext_tmvp_full_resolution |= !!bit_reader->ReadBit();
    restr.disable_ext_tmvp_exclude_intra_from_ref_list |=
      !!bit_reader->ReadBit();
    restr.disable_ext_ref_list_l0_trim |= !!bit_reader->ReadBit();
    restr.disable_ext_implicit_partition_type |= !!bit_reader->ReadBit();
    restr.disable_ext_cabac_alt_split_flag_ctx |= !!bit_reader->ReadBit();
    restr.disable_ext_cabac_alt_inter_dir_ctx |= !!bit_reader->ReadBit();
    restr.disable_ext_cabac_alt_last_pos_ctx |= !!bit_reader->ReadBit();
    restr.disable_ext_two_cu_trees |= !!bit_reader->ReadBit();
    restr.disable_ext_transform_size_64 |= !!bit_reader->ReadBit();
    restr.disable_ext_intra_unrestricted_predictor |= !!bit_reader->ReadBit();
    restr.disable_ext_deblock_subblock_size_4 |= !!bit_reader->ReadBit();
    if (segment_header->major_version > 1) {
      restr.disable_ext_transform_high_precision |= !!bit_reader->ReadBit();
    }
  }

  if (segment_header->major_version <= 1) {
    restr.disable_transform_skip = true;
    restr.disable_ext_transform_high_precision = true;
  }

  Restrictions::GetRW() = restr;

  segment_header->soc = segment_counter;
  return Decoder::State::kSegmentHeaderDecoded;
}

bool SegmentHeaderReader::SupportedBitstreamVersion(uint32_t major_version,
                                                    uint32_t minor_version) {
  if (major_version == constants::kXvcMajorVersion &&
      static_cast<int>(minor_version) >=
      static_cast<int>(constants::kXvcMinorVersion)) {
    return true;
  }
  int length = sizeof(constants::kSupportedOldBitstreamVersions) /
    sizeof(constants::kSupportedOldBitstreamVersions[0]);
  for (int i = 0; i < length; i++) {
    if (constants::kSupportedOldBitstreamVersions[i][0] == major_version &&
        constants::kSupportedOldBitstreamVersions[i][1] <= minor_version)
      return true;
  }
  return false;
}

}   // namespace xvc
