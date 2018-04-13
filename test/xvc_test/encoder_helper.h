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
    encoder_->SetDeblock(1);
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

  void EncodeFirstFrame(const std::vector<uint8_t> &pic_bytes, int bitdepth) {
    const uint8_t *in_pic = pic_bytes.empty() ? nullptr : &pic_bytes[0];
    xvc_enc_nal_unit *nal_units = nullptr;
    xvc_enc_pic_buffer rec_pic;
    encoder_->SetInputBitdepth(bitdepth);
    int num_nals = encoder_->Encode(in_pic, &nal_units, &rec_pic);
    ASSERT_EQ(2, num_nals);
    EXPECT_EQ(static_cast<uint32_t>(xvc::NalUnitType::kSegmentHeader),
              nal_units[0].stats.nal_unit_type);
    EXPECT_EQ(static_cast<uint32_t>(xvc::NalUnitType::kIntraAccessPicture),
              nal_units[1].stats.nal_unit_type);
    for (int i = 0; i < 2; i++) {
      encoded_nal_units_.push_back(
        NalUnit(nal_units[i].bytes, nal_units[i].bytes + nal_units[i].size));
    }
    if (rec_pic.size > 0) {
      rec_pics_.emplace_back(rec_pic.pic, rec_pic.pic + rec_pic.size);
    }
  }

  std::vector<xvc_enc_nal_stats>
    EncodeOneFrame(const std::vector<uint8_t> &pic_bytes, int bitdepth) {
    std::vector<xvc_enc_nal_stats> encoded_nal_stats;
    const uint8_t *in_pic = pic_bytes.empty() ? nullptr : &pic_bytes[0];
    xvc_enc_nal_unit *nal_units = nullptr;
    encoder_->SetInputBitdepth(bitdepth);
    xvc_enc_pic_buffer rec_pic;
    int num_nals = encoder_->Encode(in_pic, &nal_units, &rec_pic);
    for (int i = 0; i < num_nals; i++) {
      encoded_nal_units_.push_back(
        NalUnit(nal_units[i].bytes, nal_units[i].bytes + nal_units[i].size));
      encoded_nal_stats.push_back(nal_units[i].stats);
    }
    if (rec_pic.size > 0) {
      rec_pics_.emplace_back(rec_pic.pic, rec_pic.pic + rec_pic.size);
    }
    return encoded_nal_stats;
  }

  std::vector<xvc_enc_nal_stats> EncoderFlush() {
    std::vector<xvc_enc_nal_stats> encoded_nal_stats;
    xvc_enc_nal_unit *nal_units = nullptr;
    xvc_enc_pic_buffer rec_pic;
    while (true) {
      int num_nals = encoder_->Flush(&nal_units, &rec_pic);
      for (int i = 0; i < num_nals; i++) {
        encoded_nal_units_.push_back(
          NalUnit(nal_units[i].bytes, nal_units[i].bytes + nal_units[i].size));
        encoded_nal_stats.push_back(nal_units[i].stats);
      }
      if (rec_pic.size == 0 && !num_nals) {
        break;
      }
      rec_pics_.emplace_back(rec_pic.pic, rec_pic.pic + rec_pic.size);
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
