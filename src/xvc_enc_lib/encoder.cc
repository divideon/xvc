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

#include "xvc_enc_lib/encoder.h"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <utility>

#include "xvc_common_lib/reference_list_sorter.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/segment_header.h"
#include "xvc_enc_lib/segment_header_writer.h"
#include "xvc_enc_lib/thread_encoder.h"

namespace xvc {

Encoder::Encoder(int internal_bitdepth, int num_threads)
  : segment_header_(new SegmentHeader()),
  simd_(SimdCpu::GetRuntimeCapabilities(), internal_bitdepth),
  encoder_settings_() {
  assert(internal_bitdepth >= 8);
#if XVC_HIGH_BITDEPTH
  assert(internal_bitdepth <= 16);
#else
  assert(internal_bitdepth == 8);
#endif
  segment_header_->codec_identifier = constants::kXvcCodecIdentifier;
  segment_header_->major_version = constants::kXvcMajorVersion;
  segment_header_->minor_version = constants::kXvcMinorVersion;
  segment_header_->internal_bitdepth = internal_bitdepth;
  segment_header_->soc = 0;
  if (num_threads != 0) {
    thread_encoder_ = std::unique_ptr<ThreadEncoder>(
      new ThreadEncoder(num_threads, encoder_settings_));
  }
}

Encoder::~Encoder() {
}

bool Encoder::Encode(const uint8_t *pic_bytes,
                     xvc_enc_pic_buffer *out_rec_pic, int64_t user_data) {
  return Encode(pic_bytes, nullptr, out_rec_pic, user_data);
}

bool Encoder::Encode(const PicPlanes &planes, xvc_enc_pic_buffer *out_rec_pic,
                     int64_t user_data) {
  return Encode(nullptr, &planes, out_rec_pic, user_data);
}

bool Encoder::Encode(const uint8_t *pic_bytes, const PicPlanes *pic_planes,
                     xvc_enc_pic_buffer *out_rec_pic, int64_t user_data) {
  if (!initialized_) {
    initialized_ = true;
    Initialize();
  }
  api_output_nals_.clear();

  PicNum doc =
    SegmentHeader::CalcDocFromPoc(poc_, segment_header_->max_sub_gop_length,
                                  sub_gop_start_poc_);
  int tid =
    SegmentHeader::CalcTidFromDoc(doc, segment_header_->max_sub_gop_length,
                                  sub_gop_start_poc_);
  if (segment_header_->low_delay) {
    doc = poc_;
  }

  // Check if it is time to encode a new segment header.
  bool encode_segment_header = ((poc_ % segment_length_) == 0);
  if (segment_header_->leading_pictures > 0) {
    encode_segment_header = (poc_ >= segment_header_->max_sub_gop_length &&
      ((poc_ - segment_header_->max_sub_gop_length) % segment_length_) == 0);
  }
  if (tid == 0 && poc_ > 0) {
    sub_gop_start_poc_ = doc_ + segment_header_->max_sub_gop_length;
  }

  if (encode_segment_header) {
    StartNewSegment();
  }

  // Set picture parameters and get bytes for original picture
  std::shared_ptr<PictureEncoder> pic_enc =
    PrepareNewInputPicture(*segment_header_, doc, poc_, tid,
                           encode_segment_header, pic_bytes, pic_planes,
                           user_data);

  if (encode_segment_header) {
    DetermineBufferFlags(*pic_enc);
  }
  if (tid == 0) {
    UpdateReferenceCounts(poc_);
  }

  if (encoder_settings_.leading_pictures == 0 && poc_ == 0) {
    // Directly encode intra picture when coding the segment header
    EncodeOnePicture(pic_enc);
    doc_ = 0;
  } else if (tid == 0) {
    for (PicNum i = 0; i < segment_header_->max_sub_gop_length; i++) {
      for (auto &pic : pic_encoders_) {
        if (pic->GetPicData()->GetDoc() == doc_ + 1) {
          assert(pic->GetOutputStatus() == OutputStatus::kReady);
          EncodeOnePicture(pic);
        }
      }
    }
  }

  // Increase encoder poc counter by one for each call to Encode.
  // poc_ is initialized to 0.
  poc_++;

  // If enough pictures have been encoded, the reconstructed picture
  // with lowest poc can be output.
  if (pic_encoders_.size() + segment_header_->max_sub_gop_length >=
      pic_buffering_num_) {
    ReconstructNextPicture(out_rec_pic);
  } else if (out_rec_pic) {
    out_rec_pic->pic = nullptr;
    out_rec_pic->size = 0;
  }
  PrepareOutputNals();
  return true;
}

bool Encoder::Flush(xvc_enc_pic_buffer *rec_pic) {
  api_output_nals_.clear();
  // Since poc is increased at the end of each call to Encode
  // it is reduced by one here to get the poc of the last picture.
  if (poc_ > 0) {
    poc_--;
  }

  // Check if there are pictures left to encode.
  if (doc_ < poc_) {
    // If flush is performed before a full SubGop has been input,
    // then leading_pictures is disabled and picture numbers are
    // recalculated.
    if (doc_ == 0 && segment_header_->leading_pictures) {
      std::shared_ptr<PictureEncoder> first_pic = RewriteLeadingPictures();
      assert(first_pic);
      EncodeOnePicture(first_pic);
      doc_ = 0;
    }

    PicNum pics_to_encode = poc_ - doc_;
    PicNum num_encoded = 0;
    while (num_encoded < pics_to_encode) {
      // Find next picture to encode by searching for
      // the one that has doc = this->doc_ + 1.
      bool found = false;
      for (auto &pic : pic_encoders_) {
        if (pic->GetPicData()->GetDoc() == doc_ + 1) {
          EncodeOnePicture(pic);
          found = true;
          num_encoded++;
        }
      }
      if (!found) {
        doc_++;
      }
    }
  }

  // Increase poc by one for each call to Flush.
  poc_++;

  if (rec_pic) {
    rec_pic->pic = nullptr;
    rec_pic->size = 0;
  }
  // Check if reconstruction should be performed.
  ReconstructNextPicture(rec_pic);
  PrepareOutputNals();
  return doc_ + 1 < poc_ || last_rec_poc_ + 1 < poc_ ||
    !doc_bitstream_order_.empty();
}

void Encoder::SetEncoderSettings(const EncoderSettings &settings) {
  assert(poc_ == 0);
  encoder_settings_ = settings;
  segment_header_->num_ref_pics = settings.default_num_ref_pics;
  segment_header_->leading_pictures = settings.leading_pictures;
  segment_header_->max_binary_split_depth = settings.max_binary_split_depth;
  segment_header_->source_padding = settings.source_padding != 0;
  segment_header_->chroma_qp_offset_table = settings.chroma_qp_offset_table;
  segment_header_->leading_pictures = settings.leading_pictures;
  segment_header_->chroma_qp_offset_u = settings.chroma_qp_offset_u;
  segment_header_->chroma_qp_offset_v = settings.chroma_qp_offset_v;
  segment_header_->adaptive_qp = settings.adaptive_qp;
  // Setup restriction flags
  Restrictions &restrictions = segment_header_->restrictions;
  restrictions.EnableRestrictedMode(settings.restricted_mode);
  // Apply speed settings that correspond directly to restriction flags
  if (settings.fast_transform_size_64) {
    restrictions.disable_ext_transform_size_64 = 1;
  }
  if (settings.fast_transform_select) {
    restrictions.disable_ext2_transform_select = 1;
  }
  if (settings.fast_inter_local_illumination_comp) {
    restrictions.disable_ext2_inter_local_illumination_comp = 1;
  }
  if (settings.fast_inter_adaptive_fullpel_mv) {
    restrictions.disable_ext2_inter_adaptive_fullpel_mv = 1;
  }
  Restrictions::GetRW() = restrictions;
}

void Encoder::Initialize() {
  if (encoder_settings_.leading_pictures > 0 &&
    (segment_header_->max_sub_gop_length == 1 ||
     segment_header_->low_delay)) {
    encoder_settings_.leading_pictures = 0;
    segment_header_->leading_pictures = 0;
  } else if (encoder_settings_.leading_pictures) {
    segment_header_->leading_pictures = encoder_settings_.leading_pictures;
  }
  if (encoder_settings_.leading_pictures > 0) {
    poc_ = 1;
    last_rec_poc_ = 0;
  }
  if (thread_encoder_) {
    // When running without threads there is no point in buffering extra pics
    // TODO(PH) Scales memory usage linearly with number of threads...
    extra_num_buffered_subgops_ =
      static_cast<int>(thread_encoder_->GetNumThreads() - 1);
  }
  pic_buffering_num_ = segment_header_->num_ref_pics +
    static_cast<size_t>(segment_header_->max_sub_gop_length);
  if (!extra_num_buffered_subgops_) {
    // TODO(PH) Using one more extra buffered picture than actually needed
    // due to how picture reference counting (introduced with threads) works
    pic_buffering_num_++;
  } else {
    pic_buffering_num_ += extra_num_buffered_subgops_ *
      static_cast<size_t>(segment_header_->max_sub_gop_length);
  }
}

void Encoder::StartNewSegment() {
  prev_segment_header_ = std::move(segment_header_);
  segment_header_.reset(new SegmentHeader(*prev_segment_header_));
  if (((poc_ + segment_length_) % closed_gop_interval_) == 0) {
    segment_header_->open_gop = false;
  } else {
    segment_header_->open_gop = true;
  }
  if ((!encoder_settings_.leading_pictures && poc_ != 0) ||
    (encoder_settings_.leading_pictures &&
     poc_ != segment_header_->max_sub_gop_length)) {
    segment_header_->soc++;
  }
}

void Encoder::EncodeOnePicture(std::shared_ptr<PictureEncoder> pic_enc) {
  // Check if current picture is a tail picture.
  // This means that it will be sent before the key picture of the
  // next segment and then be buffered in the decoder to be decoded after
  // the key picture.
  std::shared_ptr<SegmentHeader> segment_header =
    pic_enc->GetPicData()->GetSoc() == segment_header_->soc ?
    segment_header_ : prev_segment_header_;
  assert(pic_enc->GetOutputStatus() == OutputStatus::kReady);
  pic_enc->SetOutputStatus(OutputStatus::kProcessing);

  NalBuffer pic_nal_buffer;
  if (!avail_nal_buffers_.empty()) {
    pic_nal_buffer = std::move(avail_nal_buffers_.back());
    avail_nal_buffers_.pop_back();
  } else {
    pic_nal_buffer.reset(new std::vector<uint8_t>());
  }

  // Determine reference pictures
  ReferenceListSorter<PictureEncoder>
    ref_list_sorter(*segment_header, prev_segment_header_->open_gop);
  auto dependent_pic_enc =
    ref_list_sorter.Prepare(pic_enc->GetPoc(), pic_enc->GetPicData()->GetTid(),
                            pic_enc->GetPicData()->IsIntraPic(), pic_encoders_,
                            pic_enc->GetPicData()->GetRefPicLists(),
                            segment_header->leading_pictures);

  if (thread_encoder_) {
    thread_encoder_->EncodeAsync(segment_header, pic_enc, dependent_pic_enc,
                                 std::move(pic_nal_buffer), segment_qp_,
                                 pic_enc->GetBufferFlag());
  } else {
    // Bitstream reference valid until next picture is coded
    const std::vector<uint8_t> *pic_bytes =
      pic_enc->Encode(*segment_header, segment_qp_, pic_enc->GetBufferFlag(),
                      encoder_settings_);
    *pic_nal_buffer = *pic_bytes;
    pic_enc->SetOutputStatus(OutputStatus::kFinishedProcessing);
    OnPictureEncoded(pic_enc, dependent_pic_enc, std::move(pic_nal_buffer));
  }

  // For all pictures expect last sub-gop bitstream order is equal to doc order
  // TODO(PH) Is there a more robust mechanism to detect this case?
  if (pic_enc->GetPicData()->GetSoc() == segment_header_->soc) {
    doc_bitstream_order_.push_back(pic_enc->GetDoc());
  }
  doc_++;
}

void Encoder::OnPictureEncoded(std::shared_ptr<PictureEncoder> pic_enc,
                               const PicEncList &inter_deps,
                               NalBuffer &&pic_nal_buffer) {
  assert(pic_enc->GetOutputStatus() == OutputStatus::kFinishedProcessing);
  pic_enc->SetOutputStatus(OutputStatus::kHasNotBeenOutput);

  // When a picture has been encoded, the picture data is put into
  // the xvc_enc_nal_unit struct to be delivered through the API.
  xvc_enc_nal_unit nal;
  nal.bytes = const_cast<uint8_t*>(&(*pic_nal_buffer)[0]);
  nal.size = pic_nal_buffer->size();
  nal.buffer_flag = pic_enc->GetBufferFlag();
  nal.user_data = pic_enc ? pic_enc->GetUserData() : 0;
  SetNalStats(*pic_enc->GetPicData(), *pic_enc, &nal.stats);
  auto &nal_stats_pair =
    pending_out_nal_buffers_[pic_enc->GetPicData()->GetDoc()];
  nal_stats_pair.first = std::move(pic_nal_buffer);
  nal_stats_pair.second = nal;

  // Decrease ref count for all reference pictures (avoiding duplicate entries)
  PicNum last_poc = pic_enc->GetPoc();
  PicEncList inter_deps_sorted = inter_deps;
  std::sort(inter_deps_sorted.begin(), inter_deps_sorted.end());
  for (auto &dep_pic : inter_deps_sorted) {
    const bool is_prev_sub_gop_pic = dep_pic->GetPicData()->GetTid() == 0 &&
      dep_pic->GetPoc() < pic_enc->GetPoc();
    if (last_poc == dep_pic->GetPoc() || is_prev_sub_gop_pic) {
      continue;
    }
    dep_pic->RemoveReferenceCount(1);
    assert(dep_pic->GetReferenceCount() >= 0);
    last_poc = dep_pic->GetPoc();
  }

  // Prune all previous lowest layer pictures in previous sub-gops
  if (pic_enc->GetPicData()->GetTid() == 0) {
    // Because reference count on previous subgops is decreased already on
    // the first picture (e.g. poc 16) in the sub-gop, lowest layer pictures
    // gets an extra reference count to survive one extra sub-gop.
    for (std::shared_ptr<PictureEncoder> &prev_pic_enc : pic_encoders_) {
      if (prev_pic_enc->GetPicData()->GetTid() == 0 &&
          prev_pic_enc->GetPoc() < pic_enc->GetPoc() &&
          prev_pic_enc->IsReferenced()) {
        prev_pic_enc->RemoveReferenceCount(1);
        assert(prev_pic_enc->GetReferenceCount() >= 0);
      }
    }
  }
}

void Encoder::PrepareOutputNals() {
  if (doc_bitstream_order_.empty()) {
    return;
  }
  PicNum next_doc = doc_bitstream_order_.front();
  auto next_output_nal_it = pending_out_nal_buffers_.find(next_doc);
  if (next_output_nal_it == pending_out_nal_buffers_.end()) {
    return;
  }
  doc_bitstream_order_.pop_front();
  NalBuffer &&pic_nal_buffer = std::move(next_output_nal_it->second.first);
  xvc_enc_nal_unit &nal = next_output_nal_it->second.second;
  if (nal.stats.nal_unit_type ==
      static_cast<uint32_t>(NalUnitType::kIntraAccessPicture)) {
    xvc_enc_nal_unit segment_nal =
      WriteSegmentHeaderNal(*segment_header_, &segment_header_bit_writer_);
    api_output_nals_.emplace_back(std::move(segment_nal));
  }
  assert(nal.bytes == &(*pic_nal_buffer)[0]);
  assert(nal.size == pic_nal_buffer->size());
  api_output_nals_.emplace_back(nal);
  // The assumption is that we don't start encoding any new pictures until
  // next api call
  avail_nal_buffers_.emplace_back(std::move(pic_nal_buffer));
  pending_out_nal_buffers_.erase(next_output_nal_it);
}

void Encoder::ReconstructNextPicture(xvc_enc_pic_buffer *out_pic) {
  std::shared_ptr<PictureEncoder> pic_enc;
  for (std::shared_ptr<PictureEncoder> &pic : pic_encoders_) {
    if (pic->GetPoc() == last_rec_poc_ + 1) {
      pic_enc = pic;
      break;
    }
  }
  if (!pic_enc ||
      pic_enc->GetOutputStatus() == OutputStatus::kReady) {
    if (out_pic) {
      out_pic->pic = nullptr;
      out_pic->size = 0;
    }
    return;
  }
  assert(pic_enc->GetOutputStatus() != OutputStatus::kHasBeenOutput);
  if (thread_encoder_) {
    thread_encoder_->WaitForPicture(
      pic_enc, [this](std::shared_ptr<PictureEncoder> pic,
                      const PicEncList &deps, NalBuffer &&nal_buffer) {
      OnPictureEncoded(pic, deps, std::move(nal_buffer));
    });
  } else if (pic_enc->GetOutputStatus() != OutputStatus::kHasNotBeenOutput) {
    if (out_pic) {
      out_pic->pic = nullptr;
      out_pic->size = 0;
    }
    return;
  }
  pic_enc->SetOutputStatus(OutputStatus::kHasBeenOutput);
  if (out_pic) {
    std::shared_ptr<YuvPicture> rec_pic = pic_enc->GetRecPic();
    rec_pic->CopyToSameBitdepth(&output_pic_bytes_);
    out_pic->size = output_pic_bytes_.size();
    out_pic->pic = output_pic_bytes_.empty() ? nullptr : &output_pic_bytes_[0];
  }
  last_rec_poc_++;
}

std::shared_ptr<PictureEncoder>
Encoder::PrepareNewInputPicture(const SegmentHeader &segment, PicNum doc,
                                PicNum poc, int tid, bool is_access_picture,
                                const uint8_t *pic_bytes,
                                const PicPlanes *pic_planes,
                                int64_t user_data) {
  // Start with assuming all picture in sub-gop reference each other
  // clean-up later in UpdateReferenceCounts, also special case for first intra
  int ref_cnt = encoder_settings_.leading_pictures || poc > 0 ?
    static_cast<int>(segment.max_sub_gop_length) : 1;
  if (tid == 0 && segment.max_sub_gop_length > 1 &&
      !extra_num_buffered_subgops_) {
    // Store lowest layer pictures for one extra sub-gop than actually needed
    ref_cnt++;
  }
  if (tid == 0) {
    // Keep lowest layer pictures in pic buffer for this many sub-gops
    ref_cnt += segment.num_ref_pics + extra_num_buffered_subgops_;
  }
  std::shared_ptr<PictureEncoder> pic_enc = GetNewPictureEncoder();
  pic_enc->Init(segment, doc, poc, tid, is_access_picture);
  pic_enc->SetReferenceCount(ref_cnt);
  pic_enc->SetUserData(user_data);
  PictureFormat input_format(segment.GetOutputWidth(),
                             segment.GetOutputHeight(),
                             input_bitdepth_, segment.chroma_format,
                             segment.color_matrix, false);
  if (pic_bytes) {
    input_resampler_.ConvertFrom(input_format, pic_bytes,
                                 pic_enc->GetOrigPic().get());
  } else if (pic_planes) {
    input_resampler_.ConvertFrom(input_format, *pic_planes,
                                 pic_enc->GetOrigPic().get());
  }
  return pic_enc;
}

void Encoder::DetermineBufferFlags(const PictureEncoder &intra_pic) {
  if (segment_header_->leading_pictures &&
      intra_pic.GetPicData()->GetDoc() == 1) {
    // leading pictures are never buffered
    return;
  }
  for (std::shared_ptr<PictureEncoder> &pic_enc : pic_encoders_) {
    SegmentHeader &segment_header =
      pic_enc->GetPicData()->GetSoc() == segment_header_->soc ?
      *segment_header_ : *prev_segment_header_;
    if (pic_enc->GetOutputStatus() == OutputStatus::kReady &&
        pic_enc->GetPoc() < intra_pic.GetPoc()) {
      if (segment_header.open_gop) {
        // for closed gop buffer flag is not used but segment header is still
        // send after all pictures in previous segment
        pic_enc->SetBufferFlag(true);
      }
      // Insert last sub-gop before next intra access picture in bitstream order
      // TODO(PH) Consider also check soc to ensure inserted before next segment
      auto doc_order_insert_it = doc_bitstream_order_.end();
      for (auto doc_order_it = doc_bitstream_order_.begin();
           doc_order_it != doc_bitstream_order_.end(); ++doc_order_it) {
        if ((doc_order_insert_it == doc_bitstream_order_.end() ||
             *doc_order_it < *doc_order_insert_it) &&
            *doc_order_it > pic_enc->GetDoc()) {
          doc_order_insert_it = doc_order_it;
        }
      }
      doc_bitstream_order_.insert(doc_order_insert_it, pic_enc->GetDoc());
    }
  }
}

void Encoder::UpdateReferenceCounts(PicNum last_subgop_end_poc) {
  const PicNum last_subgop_start_poc =
    last_subgop_end_poc < segment_header_->max_sub_gop_length ?
    0 : last_subgop_end_poc - segment_header_->max_sub_gop_length + 1;
  // Identify all picture encoders in current subgop
  std::vector<PictureEncoder*> subgop_pic_encoders;
  subgop_pic_encoders.reserve(
    static_cast<int>(segment_header_->max_sub_gop_length));
  for (std::shared_ptr<PictureEncoder> &pic_enc : pic_encoders_) {
    if (pic_enc->GetPoc() >= last_subgop_start_poc) {
      subgop_pic_encoders.push_back(pic_enc.get());
    }
  }
  if (subgop_pic_encoders.empty()) {
    return;
  }
  assert(last_subgop_start_poc == 0 ||
         subgop_pic_encoders.size() == segment_header_->max_sub_gop_length);

  for (const auto *pic_enc : subgop_pic_encoders) {
    std::shared_ptr<const PictureData> pic_data = pic_enc->GetPicData();
    SegmentHeader &segment_header =
      pic_enc->GetPicData()->GetSoc() == segment_header_->soc ?
      *segment_header_ : *prev_segment_header_;
    // Find all pictures that are referenced by this picture
    ReferencePictureLists ref_pic_list;
    ReferenceListSorter<PictureEncoder>
      ref_list_sorter(segment_header, prev_segment_header_->open_gop);
    auto depdenent_pic_encs =
      ref_list_sorter.Prepare(pic_data->GetPoc(), pic_data->GetTid(),
                              pic_data->IsIntraPic(), pic_encoders_,
                              &ref_pic_list,
                              segment_header.leading_pictures);

    for (auto *pic_enc2 : subgop_pic_encoders) {
      const PicNum poc = pic_enc2->GetPoc();
      bool not_referenced =
        std::none_of(depdenent_pic_encs.cbegin(), depdenent_pic_encs.cend(),
                     [poc](const std::shared_ptr<const PictureEncoder> &pic) {
        return poc == pic->GetPoc();
      });
      if (not_referenced) {
        pic_enc2->RemoveReferenceCount(1);
        assert(pic_enc2->GetReferenceCount() >= 0);
      }
    }
  }
}

std::shared_ptr<PictureEncoder> Encoder::GetNewPictureEncoder() {
  // Allocate a new PictureEncoder if the number of buffered pictures
  // is lower than the maximum that will be used.
  if (pic_encoders_.size() < pic_buffering_num_) {
    auto pic =
      std::make_shared<PictureEncoder>(simd_,
                                       segment_header_->GetInternalPicFormat(),
                                       segment_header_->GetCropWidth(),
                                       segment_header_->GetCropHeight());
    pic_encoders_.push_back(pic);
    return pic;
  }

  std::shared_ptr<PictureEncoder> avail_pic_enc;
  while (true) {
    for (auto &pic_enc : pic_encoders_) {
      auto pic_data = pic_enc->GetPicData();
      if (pic_enc->GetOutputStatus() != OutputStatus::kHasBeenOutput ||
          pic_enc->IsReferenced()) {
        continue;
      }
      avail_pic_enc = pic_enc;
      break;
    }
    if (avail_pic_enc || !thread_encoder_) {
      break;
    }
    // No more available buffers, sleep and wait for one to become available
    thread_encoder_->WaitOne([this](std::shared_ptr<PictureEncoder> pic,
                                    const PicEncList &deps,
                                    NalBuffer &&nal_buffer) {
      OnPictureEncoded(pic, deps, std::move(nal_buffer));
    });
  }
  assert(avail_pic_enc);
  return avail_pic_enc;
}

std::shared_ptr<PictureEncoder> Encoder::RewriteLeadingPictures() {
  std::shared_ptr<PictureEncoder> pic_zero;
  assert(segment_header_->leading_pictures);
  segment_header_->leading_pictures = 0;
  poc_--;
  // Convert all pictures from a leading picture structure to normal
  for (std::shared_ptr<PictureEncoder> &pic : pic_encoders_) {
    std::shared_ptr<PictureData> pic_data = pic->GetPicData();
    PicNum poc = pic_data->GetPoc() - 1;
    pic_data->SetPoc(poc);
    PicNum doc = SegmentHeader::CalcDocFromPoc(
      poc, segment_header_->max_sub_gop_length, sub_gop_start_poc_);
    int tid = SegmentHeader::CalcTidFromDoc(
      doc, segment_header_->max_sub_gop_length, sub_gop_start_poc_);
    int max_tid =
      SegmentHeader::GetMaxTid(segment_header_->max_sub_gop_length);
    pic_data->SetDoc(doc);
    pic_data->SetTid(tid);
    pic_data->SetHighestLayer(tid == max_tid &&
                              !segment_header_->low_delay);
    if (poc == 0) {
      pic_data->SetNalType(NalUnitType::kIntraAccessPicture);
      pic_zero = pic;
    }
  }
  return pic_zero;
}

xvc_enc_nal_unit
Encoder::WriteSegmentHeaderNal(const SegmentHeader &segment_header,
                               BitWriter *bit_writer) {
  bit_writer->Clear();
  if (encoder_settings_.encapsulation_mode != 0) {
    bit_writer->WriteBits(constants::kEncapsulationCode, 8);
    bit_writer->WriteBits(1, 8);
  }
  SegmentHeaderWriter::Write(segment_header, bit_writer, framerate_);

  std::vector<uint8_t> *nal_bytes = bit_writer->GetBytes();
  xvc_enc_nal_unit nal;
  nal.bytes = &(*nal_bytes)[0];
  nal.size = nal_bytes->size();
  nal.buffer_flag = 0;
  ::memset(&nal.stats, 0, sizeof(nal.stats));
  nal.stats.nal_unit_type = static_cast<uint32_t>(NalUnitType::kSegmentHeader);
  nal.stats.soc = segment_header.soc;
  nal.stats.tid = 0;
  nal.user_data = 0;
  return nal;
}

void Encoder::SetNalStats(const PictureData &pic_data,
                          const PictureEncoder &pic_enc,
                          xvc_enc_nal_stats *nal_stats) {
  const int poc_offset = (segment_header_->leading_pictures != 0 ? -1 : 0);
  nal_stats->nal_unit_type =
    static_cast<uint32_t>(pic_data.GetNalType());

  // Expose the 32 least significant bits of poc and doc.
  nal_stats->poc = static_cast<uint32_t>(pic_data.GetPoc() + poc_offset);
  nal_stats->doc = static_cast<uint32_t>(pic_data.GetDoc() + poc_offset);
  nal_stats->soc = static_cast<uint32_t>(pic_data.GetSoc());
  nal_stats->tid = pic_data.GetTid();
  if (pic_data.GetPicQp()) {
    nal_stats->qp = pic_data.GetPicQp()->GetQpRaw(YuvComponent::kY);
  } else {
    nal_stats->qp = constants::kMaxAllowedQp + 1;
  }
  nal_stats->sse = pic_enc.GetRecPicErrSum();
  nal_stats->psnr_y = pic_enc.GetRecPicPsnr(YuvComponent::kY);
  nal_stats->psnr_u = pic_enc.GetRecPicPsnr(YuvComponent::kU);
  nal_stats->psnr_v = pic_enc.GetRecPicPsnr(YuvComponent::kV);

  // Expose the first reference pictures in L0 and L1.
  const ReferencePictureLists* rpl = pic_data.GetRefPicLists();
  int length = sizeof(nal_stats->l0) / sizeof(nal_stats->l0[0]);
  for (int i = 0; i < length; i++) {
    if (i < rpl->GetNumRefPics(RefPicList::kL0)) {
      nal_stats->l0[i] =
        static_cast<int32_t>(rpl->GetRefPoc(RefPicList::kL0, i) + poc_offset);
    } else {
      nal_stats->l0[i] = -1;
    }
    if (i < rpl->GetNumRefPics(RefPicList::kL1)) {
      nal_stats->l1[i] =
        static_cast<int32_t>(rpl->GetRefPoc(RefPicList::kL1, i) + poc_offset);
    } else {
      nal_stats->l1[i] = -1;
    }
  }
}

}   // namespace xvc
