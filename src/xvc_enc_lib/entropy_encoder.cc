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

#include "xvc_enc_lib/entropy_encoder.h"

#include "xvc_common_lib/cabac.h"
#include "xvc_enc_lib/encoder_settings.h"

namespace xvc {

EntropyEncoder::EntropyEncoder(BitWriter *bit_writer)
  : EntropyEncoder(bit_writer, 0, 0) {
}

EntropyEncoder::EntropyEncoder(BitWriter *bit_writer, uint32_t written_bits,
                               uint32_t fractional_bits)
  : bit_writer_(bit_writer) {
  Start();
  frac_bits_ = (written_bits << 15) | (fractional_bits & 32767);
}

void EntropyEncoder::EncodeBin(uint32_t binval, ContextModel *ctx) {
  uint32_t ctxmps = ctx->GetMps();
  uint32_t lps = ctx->GetLps(range_);

  if (!bit_writer_) {
    frac_bits_ += ctx->GetEntropyBits(binval);
    if (binval != ctxmps) {
      ctx->UpdateLPS();
    } else {
      ctx->UpdateMPS();
    }
    return;
  }
  if (EncoderSettings::kEncoderCountActualWrittenBits) {
    frac_bits_ += ctx->GetEntropyBits(binval);
  }
  range_ -= lps;

  int num_bits;
  if (binval != ctxmps) {
    num_bits = ctx->GetRenormBitsLps(lps);
    low_ += range_;
    range_ = lps;
    ctx->UpdateLPS();
  } else {
    num_bits = range_ < 256 ? 1 : 0;
    ctx->UpdateMPS();
  }

  low_ <<= num_bits;
  range_ <<= num_bits;
  bits_left_ -= num_bits;

  if (num_bits) {
    WriteIfPossible();
  }
}

void EntropyEncoder::EncodeBypass(uint32_t binval) {
  if (!bit_writer_) {
    frac_bits_ += ContextModel::kEntropyBypassBits;
    return;
  }
  if (EncoderSettings::kEncoderCountActualWrittenBits) {
    frac_bits_ += ContextModel::kEntropyBypassBits;
  }
  low_ <<= 1;
  if (binval) {
    low_ += range_;
  }
  bits_left_--;
  WriteIfPossible();
}

void EntropyEncoder::EncodeBypassBins(uint32_t binvals, int num_bins) {
  if (!bit_writer_) {
    frac_bits_ += ContextModel::kEntropyBypassBits * num_bins;
    return;
  }
  if (EncoderSettings::kEncoderCountActualWrittenBits) {
    frac_bits_ += ContextModel::kEntropyBypassBits * num_bins;
  }
  while (num_bins > 8) {
    num_bins -= 8;
    uint32_t pattern = binvals >> num_bins;
    low_ <<= 8;
    low_ += range_ * pattern;
    binvals -= pattern << num_bins;
    bits_left_ -= 8;

    WriteIfPossible();
  }

  low_ <<= num_bins;
  low_ += range_ * binvals;
  bits_left_ -= num_bins;

  WriteIfPossible();
}

void EntropyEncoder::EncodeBinTrm(uint32_t binval) {
  if (!bit_writer_) {
    frac_bits_ += ContextModel::GetEntropyBitsTrm(binval);
    return;
  }
  if (EncoderSettings::kEncoderCountActualWrittenBits) {
    frac_bits_ += ContextModel::GetEntropyBitsTrm(binval);
  }
  range_ -= 2;
  int num_bits;
  if (binval) {
    low_ += range_;
    range_ = 2;
    num_bits = 7;
  } else {
    num_bits = range_ < 256 ? 1 : 0;
  }

  if (num_bits >= 0) {
    low_ <<= num_bits;
    range_ <<= num_bits;
    bits_left_ -= num_bits;
    WriteIfPossible();
  }
}

void EntropyEncoder::Start() {
  low_ = 0;
  range_ = 510;
  bits_left_ = 23;
  num_buffered_bytes_ = 0;
  buffered_byte_ = 0xff;
  frac_bits_ = 0;
}

void EntropyEncoder::Finish() {
  if (!bit_writer_) {
    return;
  }
  if (low_ >> (32 - bits_left_)) {
    bit_writer_->WriteByte(static_cast<uint8_t>(buffered_byte_ + 1));
    while (num_buffered_bytes_ > 1) {
      bit_writer_->WriteByte(0x00);
      num_buffered_bytes_--;
    }
    low_ -= 1 << (32 - bits_left_);
  } else {
    if (num_buffered_bytes_ > 0) {
      bit_writer_->WriteByte(static_cast<uint8_t>(buffered_byte_));
    }
    while (num_buffered_bytes_ > 1) {
      bit_writer_->WriteByte(0xff);
      num_buffered_bytes_--;
    }
  }
  bit_writer_->WriteBits(low_ >> 8, 24 - bits_left_);
  bit_writer_->WriteBits(1, 1);
  bit_writer_->PadZeroBits();
}

void EntropyEncoder::WriteOut() {
  uint32_t lead_byte = low_ >> (24 - bits_left_);
  bits_left_ += 8;
  low_ &= 0xffffffffu >> bits_left_;

  if (lead_byte == 0xff) {
    ++num_buffered_bytes_;
  } else {
    if (num_buffered_bytes_ > 0) {
      uint32_t carry = lead_byte >> 8;
      uint32_t byte = buffered_byte_ + carry;
      buffered_byte_ = lead_byte & 0xff;
      bit_writer_->WriteByte(static_cast<uint8_t>(byte));
      byte = (0xff + carry) & 0xff;
      while (num_buffered_bytes_ > 1) {
        bit_writer_->WriteByte(static_cast<uint8_t>(byte));
        --num_buffered_bytes_;
      }
    } else {
      num_buffered_bytes_ = 1;
      buffered_byte_ = lead_byte;
    }
  }
}

void EntropyEncoder::WriteIfPossible() {
  if (bits_left_ < 12) {
    WriteOut();
  }
}

}   // namespace xvc
