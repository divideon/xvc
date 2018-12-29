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

#include "xvc_enc_lib/picture_encoder.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <numeric>
#include <utility>

#include "xvc_common_lib/deblocking_filter.h"
#include "xvc_common_lib/quantize.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"
#include "xvc_enc_lib/cu_encoder.h"
#include "xvc_enc_lib/entropy_encoder.h"

namespace xvc {

PictureEncoder::PictureEncoder(const EncoderSimdFunctions &simd,
                               const PictureFormat &pic_fmt,
                               int crop_width, int crop_height)
  : simd_(simd),
  orig_pic_(std::make_shared<YuvPicture>(pic_fmt.chroma_format, pic_fmt.width,
                                         pic_fmt.height, pic_fmt.bitdepth,
                                         false, crop_width, crop_height)),
  pic_data_(std::make_shared<PictureData>(pic_fmt.chroma_format, pic_fmt.width,
                                          pic_fmt.height, pic_fmt.bitdepth)),
  rec_pic_(std::make_shared<YuvPicture>(pic_fmt.chroma_format, pic_fmt.width,
                                        pic_fmt.height, pic_fmt.bitdepth,
                                        true, 0, 0)) {
}

void PictureEncoder::Init(const SegmentHeader &segment, PicNum doc, PicNum poc,
                          int tid, bool is_access_picture) {
  const int max_tid = SegmentHeader::GetMaxTid(segment.max_sub_gop_length);
  output_status_ = OutputStatus::kReady;
  buffer_flag_ = false;
  pic_data_->SetDoc(doc);
  pic_data_->SetPoc(poc);
  pic_data_->SetTid(tid);
  pic_data_->SetSoc(segment.soc);
  pic_data_->SetSubGopLength(segment.max_sub_gop_length);
  pic_data_->SetHighestLayer(tid == max_tid && !segment.low_delay);
  pic_data_->SetAdaptiveQp(segment.adaptive_qp);
  pic_data_->SetBetaOffset(segment.beta_offset);
  pic_data_->SetTcOffset(segment.tc_offset);
  switch (segment.deblocking_mode) {
    case DeblockingMode::kDisabled:
      pic_data_->SetDeblock(false);
      break;
    case DeblockingMode::kEnabled:
    case DeblockingMode::kCustom:
      pic_data_->SetDeblock(true);
      break;
    case DeblockingMode::kPerPicture:
      pic_data_->SetDeblock(tid == 0);
      break;
    default:
      assert(0);
  }
  if (is_access_picture) {
    pic_data_->SetNalType(NalUnitType::kIntraAccessPicture);
  } else if (segment.num_ref_pics == 0) {
    pic_data_->SetNalType(NalUnitType::kIntraPicture);
  } else if (Restrictions::Get().disable_inter_bipred) {
    pic_data_->SetNalType(NalUnitType::kPredictedPicture);
  } else {
    pic_data_->SetNalType(NalUnitType::kBipredictedPicture);
  }
}

const std::vector<uint8_t>*
PictureEncoder::Encode(const SegmentHeader &segment, int segment_qp,
                       int buffer_flag,
                       const EncoderSettings &encoder_settings) {
  const PicturePredictionType picture_type = pic_data_->GetPredictionType();
  int sub_gop_length = static_cast<int>(segment.max_sub_gop_length);
  int max_tid = SegmentHeader::GetMaxTid(sub_gop_length);
  int pic_tid = pic_data_->GetTid();
  if (encoder_settings.flat_lambda > 0) {
    sub_gop_length = std::min(sub_gop_length, encoder_settings.flat_lambda);
    max_tid = SegmentHeader::GetMaxTid(sub_gop_length);
    pic_tid = max_tid;
  }
  const int pic_qp =
    DerivePictureQp(encoder_settings, segment_qp, picture_type, pic_tid);
  const double pic_lambda =
    CalculateLambda(encoder_settings, segment, pic_qp, picture_type,
                    sub_gop_length, pic_tid, max_tid);
  const int scaled_qp =
    GetQpFromLambda(pic_data_->GetBitdepth(), pic_lambda);
  Qp base_qp(scaled_qp, pic_data_->GetChromaFormat(), pic_data_->GetBitdepth(),
             pic_lambda, encoder_settings.chroma_qp_offset_table,
             encoder_settings.chroma_qp_offset_u,
             encoder_settings.chroma_qp_offset_v);

  pic_data_->Init(segment, base_qp, encoder_settings.adaptive_qp > 0);
  const bool allow_lic = DetermineAllowLic(pic_data_->GetPredictionType(),
                                           *pic_data_->GetRefPicLists());
  pic_data_->SetUseLocalIlluminationCompensation(allow_lic);

  bit_writer_.Clear();
  if (encoder_settings.encapsulation_mode != 0) {
    bit_writer_.WriteBits(constants::kEncapsulationCode, 8);
    bit_writer_.WriteBits(1, 8);
  }
  WriteHeader(segment, *pic_data_, sub_gop_length, buffer_flag, &bit_writer_);

  SyntaxWriter writer(base_qp, pic_data_->GetPredictionType(),
                      &bit_writer_);
  std::unique_ptr<CuEncoder>
    cu_encoder(new CuEncoder(simd_, *orig_pic_, rec_pic_.get(), pic_data_.get(),
                             encoder_settings));
  int num_ctus = pic_data_->GetNumberOfCtu();
  for (int rsaddr = 0; rsaddr < num_ctus; rsaddr++) {
    cu_encoder->EncodeCtu(rsaddr, &writer);
  }
  if (pic_data_->GetDeblock()) {
    DeblockingFilter deblocker(pic_data_.get(), rec_pic_.get(),
                               pic_data_->GetBetaOffset(),
                               pic_data_->GetTcOffset());
    deblocker.DeblockPicture();
  }
  writer.Finish();

  if (pic_data_->GetTid() == 0 || !pic_data_->IsHighestLayer()) {
    rec_pic_->PadBorder();
  }
  pic_data_->GetRefPicLists()->ZeroOutReferences();
  if (pic_data_->GetTid() == 0 ||
      segment.checksum_mode == Checksum::Mode::kMaxRobust) {
    WriteChecksum(segment, &bit_writer_, segment.checksum_mode);
  } else {
    pic_hash_.clear();
  }
  rec_sse_ = CalculatePicMetric(base_qp);
  rec_psnr_y_ = CalculatePsnr(base_qp, YuvComponent::kY);
  rec_psnr_u_ = CalculatePsnr(base_qp, YuvComponent::kU);
  rec_psnr_v_ = CalculatePsnr(base_qp, YuvComponent::kV);
  return bit_writer_.GetBytes();
}

std::shared_ptr<YuvPicture>
PictureEncoder::GetAlternativeRecPic(const PictureFormat &pic_fmt,
                                     int crop_width, int crop_height) const {
  assert(0);
  return std::shared_ptr<YuvPicture>();
}

void PictureEncoder::WriteHeader(const SegmentHeader &segment,
                                 const PictureData &pic_data,
                                 PicNum sub_gop_length, int buffer_flag,
                                 BitWriter *bit_writer) {
  bit_writer->WriteBits(1, 1);  // xvc_bit_one
  bit_writer->WriteBits(0, 1);  // nal_rfe
  bit_writer->WriteBits(static_cast<uint8_t>(pic_data.GetNalType()), 5);
  bit_writer->WriteBits(1, 1);  // nal_rfl
  bit_writer->WriteBits(buffer_flag, 1);
  assert(pic_data.GetTid() >= 0);
  bit_writer->WriteBits(pic_data.GetTid(), 3);
  const int pic_qp = pic_data.GetPicQp()->GetQpRaw(YuvComponent::kY);
  assert(pic_qp + constants::kQpSignalBase < (1 << 7));
  bit_writer->WriteBits(pic_qp + constants::kQpSignalBase, 7);
  const bool allow_lic = pic_data.GetUseLocalIlluminationCompensation();
  if (!Restrictions::Get().disable_ext2_inter_local_illumination_comp) {
    bit_writer->WriteBit(allow_lic ? 1 : 0);
  } else {
    assert(!allow_lic);
  }
  if (segment.deblocking_mode == DeblockingMode::kPerPicture) {
    bit_writer->WriteBit(pic_data.GetDeblock() ? 1 : 0);
  }
  bit_writer->PadZeroBits();
}

void PictureEncoder::WriteChecksum(const SegmentHeader &segment,
                                   BitWriter *bit_writer,
                                   Checksum::Mode checksum_mode) {
  Checksum::Method checksum_method =
    Restrictions::Get().disable_high_level_default_checksum_method ?
    Checksum::kFallbackMethod : Checksum::kDefaultMethod;
  Checksum checksum(checksum_method, checksum_mode);
  checksum.HashPicture(*rec_pic_);
  pic_hash_ = checksum.GetHash();
  assert(pic_hash_.size() < UINT8_MAX);
  if (segment.major_version <= 1) {
    // This is only needed to generate bitstream for hls unit tests
    bit_writer->WriteByte(static_cast<uint8_t>(pic_hash_.size()));
  }
  bit_writer->WriteBytes(&pic_hash_[0], pic_hash_.size());
}

int
PictureEncoder::DerivePictureQp(const EncoderSettings &encoder_settings,
                                int segment_qp, PicturePredictionType pic_type,
                                int tid) const {
  int pic_qp;
  if (pic_type == PicturePredictionType::kIntra) {
    pic_qp = segment_qp + encoder_settings.intra_qp_offset;
  } else {
    pic_qp = segment_qp + tid + 1;
  }
  return util::Clip3(pic_qp, constants::kMinAllowedQp,
                     constants::kMaxAllowedQp);
}

bool
PictureEncoder::DetermineAllowLic(PicturePredictionType pic_type,
                                  const ReferencePictureLists &ref_pics) const {
  static const double kSampleThreshold = 0.06;
  const YuvComponent comp = YuvComponent::kY;
  const int pic_width = orig_pic_->GetWidth(comp);
  const int pic_height = orig_pic_->GetHeight(comp);
  const int num_buckets = 1 << orig_pic_->GetBitdepth();
  const auto build_histogram =
    [pic_width, pic_height](const SampleBufferConst &buffer, bool increment,
                            std::vector<int32_t> *histogram) {
    const Sample *sample = buffer.GetDataPtr();
    for (int y = 0; y < pic_height; y++) {
      for (int x = 0; x < pic_width; x++) {
        Sample val = sample[x];
        (*histogram)[val] += increment ? 1 : -1;
      }
      sample += buffer.GetStride();
    }
  };

  if (pic_type == PicturePredictionType::kIntra ||
      Restrictions::Get().disable_ext2_inter_local_illumination_comp) {
    return false;
  }

  std::vector<int32_t> histogram_orig(num_buckets, 0);
  std::vector<int32_t> histogram_ref(num_buckets, 0);
  SampleBuffer orig_buffer = orig_pic_->GetSampleBuffer(comp, 0, 0);
  build_histogram(orig_buffer, true, &histogram_orig);

  int num_ref_lists = pic_type == PicturePredictionType::kBi ? 2 : 1;
  for (int ref_list_idx = 0; ref_list_idx < num_ref_lists; ref_list_idx++) {
    const RefPicList ref_list = static_cast<RefPicList>(ref_list_idx);
    const int num_ref_pics = ref_pics.GetNumRefPics(ref_list);
    for (int ref_idx = 0; ref_idx < num_ref_pics; ref_idx++) {
      const YuvPicture *ref_pic = ref_pics.GetRefOrigPic(ref_list, ref_idx);
      SampleBufferConst ref_buffer = ref_pic->GetSampleBuffer(comp, 0, 0);
      histogram_ref = histogram_orig;
      build_histogram(ref_buffer, false, &histogram_ref);
      int err_sum = 0;
      for (int i = 0; i < num_buckets; i++) {
        err_sum += std::abs(histogram_ref[i]);
      }
      if (err_sum >
          static_cast<int>(kSampleThreshold * pic_width * pic_height)) {
        return true;
      }
    }
  }
  return false;
}

uint64_t PictureEncoder::CalculatePicMetric(const Qp &qp) const {
  // Force bitdepth 8 to prevent truncating metric precision
  const int metric_bitdepth = 8;
  // Force luma component to prevent distortion scaling for chroma
  const YuvComponent metric_comp = YuvComponent::kY;
  SampleMetric metric(simd_.sample_metric, metric_bitdepth, MetricType::kSsd);
  uint64_t mse = 0;
  for (int c = 0; c < pic_data_->GetMaxNumComponents(); c++) {
    const YuvComponent comp = static_cast<YuvComponent>(c);
    mse += metric.ComparePicture(qp, comp, metric_comp, *orig_pic_, *rec_pic_);
  }
  return mse;
}

double PictureEncoder::CalculatePsnr(const Qp &qp, YuvComponent c) const {
  // Force luma component to prevent distortion scaling for chroma
  const YuvComponent metric_comp = YuvComponent::kY;
  SampleMetric metric(simd_.sample_metric, pic_data_->GetBitdepth(),
                      MetricType::kSsd);
  return metric.ComputePsnr(qp, c, metric_comp, *orig_pic_, *rec_pic_);
}

int PictureEncoder::GetQpFromLambda(int bitdepth, double lambda) {
  int qp = static_cast<int>(
    std::floor((3.0 * (log(lambda / 0.57) / log(2.0))) + 0.5));
  return util::Clip3(12 + qp, constants::kMinAllowedQp,
                     constants::kMaxAllowedQp);
}

double
PictureEncoder::CalculateLambda(const EncoderSettings &encoder_settings,
                                const SegmentHeader &segment_header,
                                int qp, PicturePredictionType pic_type,
                                int sub_gop_length, int temporal_id,
                                int max_temporal_id) {
  const int qp_temp = qp - 12;
  const double lambda = pow(2.0, qp_temp / 3.0);
  double scale_factor = encoder_settings.lambda_scale_a *
    pow(2.0, encoder_settings.lambda_scale_b * qp_temp);
  double pic_type_factor =
    (pic_type == PicturePredictionType::kIntra ? 0.57 : 0.68);
  double subgop_factor =
    1.0 - util::Clip3(0.05 * (sub_gop_length - 1), 0.0, 0.5);
  double hierarchical_factor = 1;
  if (temporal_id > 0 && temporal_id == max_temporal_id &&
      !segment_header.low_delay) {
    subgop_factor = 1.0;
    hierarchical_factor = util::Clip3(qp_temp / 6.0, 2.0, 4.0);
  } else if (temporal_id > 0) {
    hierarchical_factor = util::Clip3(qp_temp / 6.0, 2.0, 4.0);
    hierarchical_factor *= 0.8;
  }
  if (sub_gop_length == 16 && pic_type != PicturePredictionType::kIntra &&
      !segment_header.low_delay) {
    if (encoder_settings.smooth_lambda_scaling == 0) {
      static const std::array<double, 5> temporal_factor = { {
          0.6, 0.2, 0.33, 0.33, 0.4
        } };
      hierarchical_factor =
        temporal_id == 0 ? 1 : util::Clip3(qp_temp / 6.0, 2.0, 4.0);
      return temporal_factor[temporal_id] * hierarchical_factor * lambda;
    } else {
      static const std::array<double, 5> temporal_factor = { {
          0.14, 0.2, 0.33, 0.33, 0.4
        } };
      hierarchical_factor = util::Clip3(qp_temp / 6.0, 2.0, 4.0);
      return temporal_factor[temporal_id] * hierarchical_factor * lambda;
    }
  }
  return lambda *
    scale_factor * pic_type_factor * subgop_factor * hierarchical_factor;
}

}   // namespace xvc
