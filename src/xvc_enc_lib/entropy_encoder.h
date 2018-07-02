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

#ifndef XVC_ENC_LIB_ENTROPY_ENCODER_H_
#define XVC_ENC_LIB_ENTROPY_ENCODER_H_

#include "xvc_common_lib/context_model.h"
#include "xvc_enc_lib/bit_writer.h"

namespace xvc {

class EntropyEncoder {
public:
  explicit EntropyEncoder(BitWriter *bit_writer);
  EntropyEncoder(BitWriter *bit_writer, uint32_t written_bits,
                 uint32_t fractional_bits);

  void EncodeBin(uint32_t binval, ContextModel *ctx);
  void EncodeBypass(uint32_t binval);
  void EncodeBypassBins(uint32_t binvals, int num_bins);
  void EncodeBinTrm(uint32_t binval);

  void ResetBitCounting() { frac_bits_ &= 32767; }
  void Start();
  void Finish();
  Bits GetNumWrittenBits() const {
    return static_cast<uint32_t>(frac_bits_ >> 15);
  }
  Bits GetFractionalBits() const {
    return static_cast<uint32_t>(frac_bits_ & 32767);
  }

private:
  void WriteOut();
  void WriteIfPossible();

  uint32_t low_;
  uint32_t range_;
  uint32_t buffered_byte_;
  int num_buffered_bytes_;
  int bits_left_;
  uint64_t frac_bits_ = 0;
  BitWriter *bit_writer_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_ENTROPY_ENCODER_H_
