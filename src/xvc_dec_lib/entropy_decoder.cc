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

#include "xvc_dec_lib/entropy_decoder.h"

#include "xvc_common_lib/cabac.h"

namespace xvc {

template<typename Ctx>
EntropyDecoder<Ctx>::EntropyDecoder(BitReader *bit_reader)
  : bit_reader_(bit_reader) {
  range_ = 510;
  bits_needed_ = -24;
  value_ = 0;
}

template<typename Ctx>
uint32_t EntropyDecoder<Ctx>::DecodeBin(Ctx *ctx) {
  uint32_t ctxmps = ctx->GetMps();
  uint32_t lps = ctx->GetLps(range_);

  range_ -= lps;
  uint32_t scaled_range = range_ << 7;

  uint32_t binval;
  int num_bits;
  if (value_ < scaled_range) {
    binval = ctxmps;
    ctx->UpdateMPS();
    num_bits = (scaled_range < (256 << 7)) ? 1 : 0;
  } else {
    binval = 1 - ctxmps;
    value_ -= scaled_range;
    range_ = lps;
    ctx->UpdateLPS();
    num_bits = ctx->GetRenormBitsLps(lps);
  }

  value_ <<= num_bits;
  range_ <<= num_bits;
  bits_needed_ += num_bits;

  if (bits_needed_ >= 0) {
    value_ |= bit_reader_->ReadByte() << bits_needed_;
    bits_needed_ -= 8;
  }
  return binval;
}

template<typename Ctx>
uint32_t EntropyDecoder<Ctx>::DecodeBypass() {
  value_ += value_;

  if (++bits_needed_ >= 0) {
    bits_needed_ = -8;
    value_ += bit_reader_->ReadByte();
  }

  uint32_t binval = 0;
  uint32_t scaled_range = range_ << 7;
  if (value_ >= scaled_range) {
    binval = 1;
    value_ -= scaled_range;
  }
  return binval;
}

template<typename Ctx>
uint32_t EntropyDecoder<Ctx>::DecodeBypassBins(int num_bins) {
  uint32_t bins = 0;
  while (num_bins > 8) {
    value_ = (value_ << 8) + (bit_reader_->ReadByte() << (8 + bits_needed_));
    uint32_t scaled_range = range_ << 15;
    for (int i = 0; i < 8; i++) {
      bins += bins;
      scaled_range >>= 1;
      if (value_ >= scaled_range) {
        bins++;
        value_ -= scaled_range;
      }
    }
    num_bins -= 8;
  }
  bits_needed_ += num_bins;
  value_ <<= num_bins;

  if (bits_needed_ >= 0) {
    value_ += bit_reader_->ReadByte() << bits_needed_;
    bits_needed_ -= 8;
  }

  uint32_t scaled_range = range_ << (num_bins + 7);
  for (int i = 0; i < num_bins; i++) {
    bins += bins;
    scaled_range >>= 1;
    if (value_ >= scaled_range) {
      bins++;
      value_ -= scaled_range;
    }
  }
  return bins;
}

template<typename Ctx>
uint32_t EntropyDecoder<Ctx>::DecodeBinTrm() {
  range_ -= 2;
  uint32_t scaled_range = range_ << 7;
  if (value_ >= scaled_range) {
    bit_reader_->Rewind(-bits_needed_);
    return 1;
  }
  if (scaled_range < (256 << 7)) {
    range_ = scaled_range >> 6;
    value_ <<= 1;
    if (++bits_needed_ == 0) {
      bits_needed_ = -8;
      value_ += bit_reader_->ReadByte();
    }
  }
  return 0;
}

template<typename Ctx>
void EntropyDecoder<Ctx>::Start() {
  range_ = 510;
  bits_needed_ = -8;
  value_ = (bit_reader_->ReadByte() << 8);
  value_ |= bit_reader_->ReadByte();
}

template<typename Ctx>
void EntropyDecoder<Ctx>::Finish() {
  bit_reader_->ReadBits(1);
  bit_reader_->SkipBits();
}

template class EntropyDecoder<ContextModelDynamic>;
template class EntropyDecoder<ContextModelStatic>;

}   // namespace xvc
