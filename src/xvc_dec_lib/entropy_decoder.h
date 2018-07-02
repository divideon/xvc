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

#ifndef XVC_DEC_LIB_ENTROPY_DECODER_H_
#define XVC_DEC_LIB_ENTROPY_DECODER_H_

#include "xvc_common_lib/context_model.h"
#include "xvc_dec_lib/bit_reader.h"

namespace xvc {

template<typename ContextModel>
class EntropyDecoder {
public:
  explicit EntropyDecoder(BitReader *bit_reader);

  uint32_t DecodeBin(ContextModel *ctx);
  uint32_t DecodeBypass();
  uint32_t DecodeBypassBins(int num_bins);
  uint32_t DecodeBinTrm();

  void Start();
  void Finish();

private:
  uint32_t range_;
  uint32_t value_;
  int bits_needed_;
  BitReader *bit_reader_;
};

extern template class EntropyDecoder<ContextModelDynamic>;
extern template class EntropyDecoder<ContextModelStatic>;

}   // namespace xvc

#endif  // XVC_DEC_LIB_ENTROPY_DECODER_H_
