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

#ifndef XVC_TEST_ENCODER_HELPER_H_
#define XVC_TEST_ENCODER_HELPER_H_

#include <memory>
#include <vector>

#include "googletest/include/gtest/gtest.h"

#include "xvc_enc_lib/encoder.h"

namespace xvc_test {

using NalUnit = std::vector<uint8_t>;
using OutputPic = std::vector<uint8_t>;

class EncoderHelper {
public:
  static const int kDefaultQp = 27;

  void Init(int internal_bitdepth = 8) {
    encoded_nal_units_.clear();
    xvc::EncoderSettings encoder_settings = GetDefaultEncoderSettings();
    SetupEncoder(encoder_settings, 0, 0, internal_bitdepth, kDefaultQp);
  }

  xvc::EncoderSettings GetDefaultEncoderSettings() {
    xvc::EncoderSettings encoder_settings;
    encoder_settings.Initialize(xvc::SpeedMode::kSlow);
    encoder_settings.Tune(xvc::TuneMode::kPsnr);
    return encoder_settings;
  }

  void SetupEncoder(const xvc::EncoderSettings &encoder_settings,
                    int width, int height, int internal_bitdepth, int qp) {
    encoder_.reset(new xvc::Encoder(internal_bitdepth));
    encoder_->SetEncoderSettings(encoder_settings);
    encoder_->SetResolution(width, height);
    encoder_->SetChromaFormat(xvc::ChromaFormat::k420);
    encoder_->SetInputBitdepth(8);
    encoder_->SetSegmentLength(64);
    encoder_->SetSubGopLength(1);
    encoder_->SetFramerate(30);
    encoder_->SetChecksumMode(xvc::Checksum::Mode::kMaxRobust);
    encoder_->SetDeblockingMode(xvc::DeblockingMode::kEnabled);
    encoder_->SetQp(qp);
  }

  std::vector<uint8_t> CreateSampleBuffer(xvc::Sample fill_sample,
                                          int bitdepth) {
    const xvc::SegmentHeader *segment = encoder_->GetCurrentSegment();
    size_t sample_size = bitdepth == 8 ? sizeof(uint8_t) : sizeof(uint16_t);
    size_t plane_samples =
      segment->GetInternalWidth() * segment->GetInternalHeight() * sample_size;
    std::vector<uint8_t> pic_bytes(plane_samples *
                                   ::xvc::constants::kMaxYuvComponents);
    if (bitdepth == 8) {
      std::fill(pic_bytes.begin(), pic_bytes.end(),
                static_cast<uint8_t>(fill_sample));
    } else {
      std::fill(reinterpret_cast<uint16_t*>(&pic_bytes[0]),
                reinterpret_cast<uint16_t*>(&pic_bytes[pic_bytes.size() - 2]),
                static_cast<uint16_t>(fill_sample));
    }
    return pic_bytes;
  }

  std::vector<xvc_enc_nal_stats>
    EncodeOneFrame(const std::vector<uint8_t> &pic_bytes, int bitdepth) {
    std::vector<xvc_enc_nal_stats> encoded_nal_stats;
    const uint8_t *in_pic = pic_bytes.empty() ? nullptr : &pic_bytes[0];
    encoder_->SetInputBitdepth(bitdepth);
    xvc_enc_pic_buffer rec_pic;
    bool success = encoder_->Encode(in_pic, &rec_pic);
    EXPECT_TRUE(success);
    for (xvc_enc_nal_unit &nal_unit : encoder_->GetOutputNals()) {
      encoded_nal_units_.push_back(
        NalUnit(nal_unit.bytes, nal_unit.bytes + nal_unit.size));
      encoded_nal_stats.push_back(nal_unit.stats);
    }
    if (rec_pic.size > 0) {
      rec_pics_.emplace_back(rec_pic.pic, rec_pic.pic + rec_pic.size);
    }
    return encoded_nal_stats;
  }

  std::vector<xvc_enc_nal_stats> EncoderFlush() {
    std::vector<xvc_enc_nal_stats> encoded_nal_stats;
    xvc_enc_pic_buffer rec_pic;
    while (true) {
      bool success = encoder_->Flush(&rec_pic);
      for (xvc_enc_nal_unit &nal_unit : encoder_->GetOutputNals()) {
        encoded_nal_units_.push_back(
          NalUnit(nal_unit.bytes, nal_unit.bytes + nal_unit.size));
        encoded_nal_stats.push_back(nal_unit.stats);
      }
      rec_pics_.emplace_back(rec_pic.pic, rec_pic.pic + rec_pic.size);
      if (!success) {
        break;
      }
    }
    return encoded_nal_stats;
  }

  void ResetBitstreamPosition() {
    decoded_nals_units = 0;
  }

  NalUnit& GetNextNalToDecode() {
    return encoded_nal_units_[decoded_nals_units++];
  }

  bool HasMoreNals() {
    return decoded_nals_units < static_cast<int>(encoded_nal_units_.size());
  }

protected:
  std::unique_ptr<xvc::Encoder> encoder_;
  std::vector<NalUnit> encoded_nal_units_;
  std::vector<OutputPic> rec_pics_;
  int decoded_nals_units = 0;
};

}   // namespace xvc_test

#endif  // XVC_TEST_ENCODER_HELPER_H_
