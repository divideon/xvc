/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include "xvc_dec_lib/decoder.h"

#include <cassert>
#include <limits>

#include "xvc_common_lib/reference_list_sorter.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/segment_header.h"
#include "xvc_common_lib/utils.h"
#include "xvc_dec_lib/segment_header_reader.h"
#include "xvc_dec_lib/thread_decoder.h"

namespace xvc {

Decoder::Decoder(int num_threads)
  : curr_segment_header_(std::make_shared<SegmentHeader>()),
  prev_segment_header_(std::make_shared<SegmentHeader>()),
  simd_(SimdCpu::GetRuntimeCapabilities()) {
  if (num_threads != 0) {
    thread_decoder_ =
      std::unique_ptr<ThreadDecoder>(new ThreadDecoder(num_threads));
  }
}

Decoder::~Decoder() {
  if (thread_decoder_) {
    thread_decoder_->StopAll();
  }
}

bool Decoder::DecodeNal(const uint8_t *nal_unit, size_t nal_unit_size,
                        int64_t user_data) {
  // Nal header parsing
  BitReader bit_reader(nal_unit, nal_unit_size);
  uint8_t header = bit_reader.ReadByte();
  // Check the nal_rfe to see if the Nal Unit shall be ignored.
  int nal_rfe = ((header >> 6) & 3);
  if (nal_rfe > 0) {
    // Accept the nal unit only if it uses one of the encapsulation codes.
    if (header == constants::kEncapsulationCode1 ||
        header == constants::kEncapsulationCode2) {
      bit_reader.ReadByte();
      header = bit_reader.ReadByte();
    } else {
      return false;
    }
  }
  NalUnitType nal_unit_type = NalUnitType((header >> 1) & 31);

  // Segment header parsing
  if (nal_unit_type == NalUnitType::kSegmentHeader) {
    return DecodeSegmentHeaderNal(&bit_reader);
  }
  if (state_ == State::kNoSegmentHeader ||
      state_ == State::kDecoderVersionTooLow ||
      state_ == State::kBitstreamBitdepthTooHigh) {
    // Do not decode anything else than a segment header if
    // no segment header has been decoded or if the xvc version
    // of the decoder is identified to be too low or if the
    // bitstream bitdepth is too high.
    return false;
  }

  if (nal_unit_type >= NalUnitType::kIntraPicture &&
      nal_unit_type <= NalUnitType::kReservedPictureType10) {
    // All picture types are decoded using the same process.
    // First, the buffer flag is checked to see if the picture
    // should be decoded or buffered.
    int buffer_flag = bit_reader.ReadBit();
    int tid = bit_reader.ReadBits(3);
    int new_desired_max_tid = SegmentHeader::GetFramerateMaxTid(
      decoder_ticks_, curr_segment_header_->bitstream_ticks,
      curr_segment_header_->max_sub_gop_length);
    if (new_desired_max_tid < max_tid_ || tid == 0) {
      // Number of temporal layers can always be decreased,
      // but only increased at temporal layer 0 pictures.
      max_tid_ = new_desired_max_tid;
    }
    if (tid > max_tid_) {
      // Ignore (drop) picture if it belongs to a temporal layer that
      // should not be decoded.
      return true;
    }
    num_pics_in_buffer_++;
    bit_reader.Rewind(4);

    NalUnitPtr nal_element(new std::vector<uint8_t>(&nal_unit[0], &nal_unit[0]
                                                    + nal_unit_size));
    if (buffer_flag == 0 && num_tail_pics_ > 0) {
      nal_buffer_.push_front({ std::move(nal_element), user_data });
    } else {
      nal_buffer_.push_back({ std::move(nal_element), user_data });
    }
    if (buffer_flag) {
      num_tail_pics_++;
      return true;
    }
    while (nal_buffer_.size() > 0 &&
           num_pics_in_buffer_ - nal_buffer_.size() + 1 < pic_buffering_num_) {
      auto &&nal = nal_buffer_.front();
      DecodeOneBufferedNal(std::move(nal.first), nal.second);
      nal_buffer_.pop_front();
    }
    return true;
  }
  return false;
}

void Decoder::DecodeAllBufferedNals() {
  for (auto &&nal : nal_buffer_) {
    DecodeOneBufferedNal(std::move(nal.first), nal.second);
  }
  if (thread_decoder_) {
    thread_decoder_->WaitAll([this](std::shared_ptr<PictureDecoder> pic_dec,
                                    bool success, const PicDecList &deps) {
      OnPictureDecoded(pic_dec, success, deps);
    });
  }
  nal_buffer_.clear();
}

bool Decoder::DecodeSegmentHeaderNal(BitReader *bit_reader) {
  // If there are old nal units buffered that are not tail pictures,
  // they are discarded before decoding the new segment.
  if (nal_buffer_.size() > static_cast<size_t>(num_tail_pics_)) {
    num_pics_in_buffer_ -= static_cast<uint32_t>(nal_buffer_.size());
    nal_buffer_.clear();
    num_tail_pics_ = 0;
  }
  prev_segment_header_ = curr_segment_header_;
  curr_segment_header_ = std::make_shared<SegmentHeader>();
  soc_++;
  state_ =
    SegmentHeaderReader::Read(curr_segment_header_.get(), bit_reader, soc_);
  if (state_ != State::kSegmentHeaderDecoded) {
    return false;
  }
  sub_gop_length_ = curr_segment_header_->max_sub_gop_length;
  if (sub_gop_length_ + 1 > sliding_window_length_) {
    sliding_window_length_ = sub_gop_length_ + 1 + (thread_decoder_ ? 1 : 0);
  }
  pic_buffering_num_ =
    sliding_window_length_ + curr_segment_header_->num_ref_pics;

  if (output_width_ == 0) {
    output_width_ = curr_segment_header_->GetOutputWidth();
  }
  if (output_height_ == 0) {
    output_height_ = curr_segment_header_->GetOutputHeight();
  }
  if (output_chroma_format_ == ChromaFormat::kUndefinedChromaFormat) {
    output_chroma_format_ = curr_segment_header_->chroma_format;
  }
  if (output_color_matrix_ == ColorMatrix::kUndefinedColorMatrix) {
    output_color_matrix_ = curr_segment_header_->color_matrix;
  }
  if (output_bitdepth_ == 0) {
    output_bitdepth_ = curr_segment_header_->internal_bitdepth;
  }
  max_tid_ = SegmentHeader::GetFramerateMaxTid(
    decoder_ticks_, curr_segment_header_->bitstream_ticks, sub_gop_length_);
  return true;
}

void
Decoder::DecodeOneBufferedNal(NalUnitPtr &&nal, int64_t user_data) {
  BitReader pic_bit_reader(&(*nal)[0], nal->size());
  std::shared_ptr<SegmentHeader> segment_header = curr_segment_header_;

  int header = pic_bit_reader.ReadByte();
  int nal_rfe = ((header >> 6) & 3);
  if (nal_rfe > 0) {
    // Read two more bytes if encapsulation is used.
    pic_bit_reader.ReadBits(16);
  }

  // Special handling for tail pictures
  int buffer_flag = pic_bit_reader.ReadBits(1);
  pic_bit_reader.Rewind(9);
  if (buffer_flag) {
    segment_header = prev_segment_header_;
    num_tail_pics_--;
  }

  // Parse picture header to determine poc
  PictureDecoder::PicNalHeader pic_header =
    PictureDecoder::DecodeHeader(&pic_bit_reader, &sub_gop_end_poc_,
                                 &sub_gop_start_poc_, &sub_gop_length_,
                                 segment_header->max_sub_gop_length,
                                 prev_segment_header_->max_sub_gop_length,
                                 doc_, soc_, num_tail_pics_);
  doc_ = pic_header.doc + 1;

  // Reload restriction flags for current thread if segment has changed
  thread_local SegmentNum loaded_restrictions_soc = static_cast<SegmentNum>(-1);
  if (loaded_restrictions_soc != segment_header->soc) {
    Restrictions::GetRW() = segment_header->restrictions;
  }

  // Determine dependencies for reference picture based on poc and tid
  const bool is_intra_nal =
    pic_header.nal_unit_type == NalUnitType::kIntraPicture ||
    pic_header.nal_unit_type == NalUnitType::kIntraAccessPicture;
  ReferenceListSorter<PictureDecoder>
    ref_list_sorter(*segment_header, prev_segment_header_->open_gop);
  ReferencePictureLists ref_pic_list;
  auto inter_dependencies =
    ref_list_sorter.Prepare(pic_header.poc, pic_header.tid, is_intra_nal,
                            pic_decoders_, &ref_pic_list);

  // Bump ref count before finding an available picture decoder
  for (auto pic_dep : inter_dependencies) {
    pic_dep->AddReferenceCount(1);
  }

  // Find an available decoder to use for this nal
  std::shared_ptr<PictureDecoder> pic_dec;
  if (thread_decoder_) {
    while (!(pic_dec = GetFreePictureDecoder(*segment_header))) {
      thread_decoder_->WaitOne([this](std::shared_ptr<PictureDecoder> pic,
                                      bool success, const PicDecList &deps) {
        OnPictureDecoded(pic, success, deps);
      });
    }
  } else {
    pic_dec = GetFreePictureDecoder(*segment_header);
    assert(pic_dec);
  }

  // Setup poc and output status on main thread
  pic_dec->Init(*segment_header, pic_header, std::move(ref_pic_list),
                user_data);

  // Special handling of inter dependency ref counting for lowest layer
  if (pic_header.tid == 0) {
    // Increase ref count for current picture, since must live a few sub-gops
    pic_dec->AddReferenceCount(1 + segment_header->num_ref_pics);
    // Also decrease ref-count of all previous lowest layer pictures by one
    for (auto prev_pic : pic_decoders_) {
      if (prev_pic->GetPicData()->GetTid() == 0 &&
          prev_pic->GetPicData()->GetPoc() < pic_header.poc) {
        // TODO(PH) Note that ref count can probably go negative here...
        prev_pic->RemoveReferenceCount(1);
      }
    }
  }

  if (thread_decoder_) {
    thread_decoder_->DecodeAsync(std::move(segment_header), std::move(pic_dec),
                                 std::move(inter_dependencies), std::move(nal),
                                 pic_bit_reader.GetPosition());
    if (state_ == State::kSegmentHeaderDecoded) {
      state_ = State::kPicDecoded;
    }
  } else {
    // Synchronous decode
    bool success = pic_dec->Decode(*segment_header, &pic_bit_reader);
    OnPictureDecoded(pic_dec, success, inter_dependencies);
  }
}

void Decoder::FlushBufferedTailPics() {
  // Return if there are still Nal Units waiting
  // to be decoded.
  if (nal_buffer_.size() > static_cast<size_t>(num_tail_pics_)) {
    return;
  }
  // Remove restriction of minimum picture buffer size
  enforce_sliding_window_ = false;
  // Preparing to start a new segment
  soc_++;
  prev_segment_header_ = curr_segment_header_;
  // Check if there are buffered nal units.
  if (nal_buffer_.size() > 0) {
    if (curr_segment_header_->open_gop) {
      // Throw away buffered Nal Units.
      num_pics_in_buffer_ -= static_cast<uint32_t>(nal_buffer_.size());
      nal_buffer_.clear();
    } else {
      // Step over the missing key picture and then decode the buffered
      // Nal Units.
      doc_++;
      sub_gop_start_poc_ = sub_gop_end_poc_;
      sub_gop_end_poc_ += sub_gop_length_;
      DecodeAllBufferedNals();
    }
  }
}

bool Decoder::GetDecodedPicture(xvc_decoded_picture *output_pic) {
  // Prevent outputing pictures if non are available
  // otherwise reference pictures might be corrupted
  if (enforce_sliding_window_ && !HasPictureReadyForOutput()) {
    output_pic->size = 0;
    output_pic->bytes = nullptr;
    for (int c = 0; c < constants::kMaxYuvComponents; c++) {
      output_pic->planes[c] = nullptr;
      output_pic->stride[c] = 0;
    }
    return false;
  }

  // Find the picture with lowest poc that has not been output.
  std::shared_ptr<PictureDecoder> pic_dec;
  PicNum lowest_poc = std::numeric_limits<PicNum>::max();
  for (auto &pic : pic_decoders_) {
    auto pd = pic->GetPicData();
    if (pic->GetOutputStatus() != OutputStatus::kHasBeenOutput &&
        pd->GetPoc() < lowest_poc) {
      pic_dec = pic;
      lowest_poc = pd->GetPoc();
    }
  }
  if (!pic_dec) {
    output_pic->size = 0;
    output_pic->bytes = nullptr;
    for (int c = 0; c < constants::kMaxYuvComponents; c++) {
      output_pic->planes[c] = nullptr;
      output_pic->stride[c] = 0;
    }
    return false;
  }

  // Wait for picture to finish decoding
  if (thread_decoder_) {
    thread_decoder_->WaitForPicture(
      pic_dec, [this](std::shared_ptr<PictureDecoder> pic, bool success,
                      const PicDecList &deps) {
      OnPictureDecoded(pic, success, deps);
    });
  }

  pic_dec->SetOutputStatus(OutputStatus::kHasBeenOutput);
  SetOutputStats(pic_dec, output_pic);
  auto decoded_pic = pic_dec->GetRecPic();
  decoded_pic->CopyTo(&output_pic_bytes_, output_width_, output_height_,
                      output_chroma_format_, output_bitdepth_,
                      output_color_matrix_);
  const int sample_size = output_bitdepth_ == 8 ? 1 : 2;
  output_pic->size = output_pic_bytes_.size();
  output_pic->bytes = output_pic_bytes_.empty() ? nullptr :
    reinterpret_cast<char *>(&output_pic_bytes_[0]);
  output_pic->planes[0] = output_pic->bytes;
  output_pic->stride[0] = output_width_ * sample_size;
  output_pic->planes[1] =
    output_pic->planes[0] + output_pic->stride[0] * output_height_;
  output_pic->stride[1] =
    util::ScaleChromaX(output_width_, output_chroma_format_) * sample_size;
  output_pic->planes[2] = output_pic->planes[1] + output_pic->stride[1] *
    util::ScaleChromaY(output_height_, output_chroma_format_);
  output_pic->stride[2] = output_pic->stride[1];

  // Decrease counter for how many decoded pictures are buffered.
  num_pics_in_buffer_--;

  if (nal_buffer_.size() > static_cast<size_t>(num_tail_pics_) &&
      num_pics_in_buffer_ - nal_buffer_.size() < pic_buffering_num_) {
    auto &&nal = nal_buffer_.front();
    DecodeOneBufferedNal(std::move(nal.first), nal.second);
    nal_buffer_.pop_front();
  }
  return true;
}

std::shared_ptr<PictureDecoder>
Decoder::GetFreePictureDecoder(const SegmentHeader &segment) {
  if (pic_decoders_.size() < pic_buffering_num_) {
    auto pic =
      std::make_shared<PictureDecoder>(simd_, segment.chroma_format,
                                       segment.GetInternalWidth(),
                                       segment.GetInternalHeight(),
                                       segment.internal_bitdepth);
    pic_decoders_.push_back(pic);
    return pic;
  }

  auto pic_dec_it = pic_decoders_.end();
  for (auto it = pic_decoders_.begin(); it != pic_decoders_.end(); ++it) {
    // A picture decoder has two independent variables that indicates usage
    // 1. output status - if decoded samples has been sent to application
    // 2. ref count - if picture is no longer referenced by any other pictures
    if ((*it)->IsReferenced() ||
      (*it)->GetOutputStatus() != OutputStatus::kHasBeenOutput) {
      continue;
    }
    pic_dec_it = it;
    break;
  }
  if (pic_dec_it == pic_decoders_.end()) {
    return std::shared_ptr<PictureDecoder>();
  }

  // Replace the PictureDecoder if the picture format has changed.
  auto pic_data = (*pic_dec_it)->GetPicData();
  if (segment.GetInternalWidth() !=
      pic_data->GetPictureWidth(YuvComponent::kY) ||
      segment.GetInternalHeight() !=
      pic_data->GetPictureHeight(YuvComponent::kY) ||
      segment.chroma_format != pic_data->GetChromaFormat() ||
      segment.internal_bitdepth != pic_data->GetBitdepth()) {
    pic_dec_it->reset(new PictureDecoder(simd_, segment.chroma_format,
                                         segment.GetInternalWidth(),
                                         segment.GetInternalHeight(),
                                         segment.internal_bitdepth));
  }
  return *pic_dec_it;
}

void Decoder::OnPictureDecoded(std::shared_ptr<PictureDecoder> pic_dec,
                               bool success, const PicDecList &inter_deps) {
  pic_dec->SetOutputStatus(OutputStatus::kHasNotBeenOutput);
  for (auto pic_dep : inter_deps) {
    pic_dep->RemoveReferenceCount(1);
  }
  if (success) {
    if (state_ != State::kChecksumMismatch) {
      state_ = State::kPicDecoded;
    }
  } else {
    state_ = State::kChecksumMismatch;
    num_corrupted_pics_++;
  }
}

void Decoder::SetOutputStats(std::shared_ptr<PictureDecoder> pic_dec,
                             xvc_decoded_picture *output_pic) {
  auto pic_data = pic_dec->GetPicData();
  output_pic->user_data = pic_dec->GetNalUserData();
  output_pic->stats.width = output_width_;
  output_pic->stats.height = output_height_;
  output_pic->stats.bitdepth = output_bitdepth_;
  output_pic->stats.chroma_format =
    xvc_dec_chroma_format(output_chroma_format_);
  output_pic->stats.color_matrix =
    xvc_dec_color_matrix(output_color_matrix_);
  output_pic->stats.bitstream_bitdepth = pic_data->GetBitdepth();
  output_pic->stats.framerate =
    SegmentHeader::GetFramerate(max_tid_, curr_segment_header_->bitstream_ticks,
                                curr_segment_header_->max_sub_gop_length);
  output_pic->stats.bitstream_framerate =
    SegmentHeader::GetFramerate(0, curr_segment_header_->bitstream_ticks, 1);
  output_pic->stats.nal_unit_type =
    static_cast<uint32_t>(pic_data->GetNalType());

  // Expose the 32 least significant bits of poc and doc.
  output_pic->stats.poc = static_cast<uint32_t>(pic_data->GetPoc());
  output_pic->stats.doc = static_cast<uint32_t>(pic_data->GetDoc());
  output_pic->stats.soc = static_cast<uint32_t>(pic_data->GetSoc());
  output_pic->stats.tid = pic_data->GetTid();
  output_pic->stats.qp = pic_data->GetPicQp()->GetQpRaw(YuvComponent::kY);

  // Expose the first five reference pictures in L0 and L1.
  ReferencePictureLists* rpl = pic_data->GetRefPicLists();
  int length = sizeof(output_pic->stats.l0) / sizeof(output_pic->stats.l0[0]);
  for (int i = 0; i < length; i++) {
    if (i < rpl->GetNumRefPics(RefPicList::kL0)) {
      output_pic->stats.l0[i] =
        static_cast<int32_t>(rpl->GetRefPoc(RefPicList::kL0, i));
    } else {
      output_pic->stats.l0[i] = -1;
    }
    if (i < rpl->GetNumRefPics(RefPicList::kL1)) {
      output_pic->stats.l1[i] =
        static_cast<int32_t>(rpl->GetRefPoc(RefPicList::kL1, i));
    } else {
      output_pic->stats.l1[i] = -1;
    }
  }
}

}   // namespace xvc
