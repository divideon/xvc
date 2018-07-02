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

#ifndef XVC_COMMON_LIB_SAMPLE_BUFFER_H_
#define XVC_COMMON_LIB_SAMPLE_BUFFER_H_

#include <array>
#include <cstring>
#include <memory>

#include "xvc_common_lib/common.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

template<typename T>
class DataBuffer {
public:
  DataBuffer(T *data, ptrdiff_t stride) : data_(data), stride_(stride) {}
  operator DataBuffer<const T>() const {
    return DataBuffer<const T>(data_, stride_);
  }
  DataBuffer<T> Offset(int x, int y) {
    return DataBuffer<T>(GetDataPtr() + GetStride() * y + x, GetStride());
  }

  const T* GetDataPtr() const { return data_; }
  T* GetDataPtr() { return data_; }
  ptrdiff_t GetStride() const { return stride_; }

  void CopyFrom(int width, int height, const DataBuffer<const T> &src_buffer) {
    const T *src = src_buffer.GetDataPtr();
    T *dst = GetDataPtr();
    for (int y = 0; y < height; y++) {
      std::memcpy(dst, src, width * sizeof(T));
      src += src_buffer.GetStride();
      dst += GetStride();
    }
  }

private:
  T *data_;
  ptrdiff_t stride_;
};

using SampleBufferConst = DataBuffer<const Sample>;
class SampleBuffer : public DataBuffer<Sample> {
public:
  SampleBuffer(Sample *data, ptrdiff_t stride) : DataBuffer(data, stride) {}
  SampleBuffer Offset(int x, int y) {
    return SampleBuffer(GetDataPtr() + GetStride() * y + x, GetStride());
  }

  void AddClip(int width, int height,
               const DataBuffer<const Sample> &pred_buffer,
               const DataBuffer<const Residual> &residual_buffer,
               Sample min_val, Sample max_val) {
    const Sample *src1 = pred_buffer.GetDataPtr();
    const Residual *src2 = residual_buffer.GetDataPtr();
    Sample *dst = GetDataPtr();
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        dst[x] = util::Clip3<Sample>(src1[x] + src2[x], min_val, max_val);
      }
      src1 += pred_buffer.GetStride();
      src2 += residual_buffer.GetStride();
      dst += GetStride();
    }
  }

  void AddAvg(int width, int height,
              const DataBuffer<const int16_t> &src1_buffer,
              const DataBuffer<const int16_t> &src2_buffer,
              int offset, int shift,
              Sample min_val, Sample max_val) {
    const int16_t *src1 = src1_buffer.GetDataPtr();
    const int16_t *src2 = src2_buffer.GetDataPtr();
    Sample *dst = GetDataPtr();
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        dst[x] = util::Clip3<Sample>((src1[x] + src2[x] + offset) >> shift,
                                     min_val, max_val);
      }
      src1 += src1_buffer.GetStride();
      src2 += src2_buffer.GetStride();
      dst += GetStride();
    }
  }

  void AddLinearModel(int width, int height,
                      const DataBuffer<const Sample> &ref_buffer,
                      int scale, int shift, int offset,
                      Sample min_val, Sample max_val) {
    const Sample *ref = ref_buffer.GetDataPtr();
    Sample *dst = GetDataPtr();
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        dst[x] = util::Clip3<Sample>(((scale * ref[x]) >> shift) + offset,
                                     min_val, max_val);
      }
      ref += ref_buffer.GetStride();
      dst += GetStride();
    }
  }
};

using ResidualBufferConst = DataBuffer<const Residual>;
class ResidualBuffer : public DataBuffer<Residual> {
public:
  ResidualBuffer(Residual *data, ptrdiff_t stride) : DataBuffer(data, stride) {}

  template<typename Sample1, typename Sample2>
  void Subtract(int width, int height,
                const DataBuffer<Sample1> &src1_buffer,
                const DataBuffer<Sample2> &src2_buffer) {
    const Sample1 *src1 = src1_buffer.GetDataPtr();
    const Sample2 *src2 = src2_buffer.GetDataPtr();
    Residual *dst = GetDataPtr();
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        dst[x] = static_cast<Residual>(src1[x] - src2[x]);
      }
      src1 += src1_buffer.GetStride();
      src2 += src2_buffer.GetStride();
      dst += GetStride();
    }
  }

  void SubtractWeighted(int width, int height,
                        const DataBuffer<const Sample> &src1_buffer,
                        const DataBuffer<const Sample> &src2_buffer) {
    const Sample *src1 = src1_buffer.GetDataPtr();
    const Sample *src2 = src2_buffer.GetDataPtr();
    Residual *dst = GetDataPtr();
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        dst[x] = static_cast<Residual>((2 * src1[x]) - src2[x]);
      }
      src1 += src1_buffer.GetStride();
      src2 += src2_buffer.GetStride();
      dst += GetStride();
    }
  }
};

using CoeffBufferConst = DataBuffer<const Coeff>;
class CoeffBuffer : public DataBuffer<Coeff> {
public:
  CoeffBuffer(Coeff *data, ptrdiff_t stride) : DataBuffer(data, stride) {}

  void ZeroOut(int width, int height) {
    Coeff *dst = GetDataPtr();
    for (int y = 0; y < height; y++) {
      std::fill(dst, dst + width, static_cast<Coeff>(0));
      dst += GetStride();
    }
  }
};

class SampleBufferStorage
  : public std::unique_ptr<Sample[]>, public SampleBuffer {
public:
  SampleBufferStorage(int width, int height)
    : std::unique_ptr<Sample[]>(new Sample[width * height]),
    SampleBuffer(std::unique_ptr<Sample[]>::get(), width) {
  }
};

class ResidualBufferStorage
  : public std::unique_ptr<Residual[]>, public ResidualBuffer {
public:
  ResidualBufferStorage(int width, int height)
    : std::unique_ptr<Residual[]>(new Residual[width * height]),
    ResidualBuffer(std::unique_ptr<Residual[]>::get(), width) {
  }
};

class CoeffBufferStorage
  : public std::unique_ptr<Coeff[]>, public CoeffBuffer {
public:
  CoeffBufferStorage(int width, int height)
    : std::unique_ptr<Coeff[]>(new Coeff[width * height]),
    CoeffBuffer(std::unique_ptr<Coeff[]>::get(), width) {
  }
};

class CoeffCtuBuffer {
public:
  CoeffCtuBuffer(int chroma_shift_x, int chroma_shift_y) :
    // For getting relative position within CTU
    pos_mask_x_({ { constants::kMaxBlockSize - 1,
                (constants::kMaxBlockSize >> chroma_shift_x) - 1,
                (constants::kMaxBlockSize >> chroma_shift_x) - 1 } }),
    pos_mask_y_({ { constants::kMaxBlockSize - 1,
                (constants::kMaxBlockSize >> chroma_shift_y) - 1,
                (constants::kMaxBlockSize >> chroma_shift_y) - 1 } }) {
  }
  CoeffCtuBuffer(const CoeffCtuBuffer&) = delete;
  CoeffCtuBuffer(const CoeffCtuBuffer&&) = delete;
  CoeffCtuBuffer& operator=(const CoeffCtuBuffer&) = delete;

  CoeffBuffer GetBuffer(YuvComponent comp, int posx, int posy) {
    posx = posx & pos_mask_x_[static_cast<int>(comp)];
    posy = posy & pos_mask_y_[static_cast<int>(comp)];
    Coeff *data = &comp_storage_[static_cast<int>(comp)][0];
    return CoeffBuffer(data + posy * kStride + posx, kStride);
  }
  DataBuffer<const Coeff> GetBuffer(YuvComponent comp,
                                    int posx, int posy) const {
    posx = posx & pos_mask_x_[static_cast<int>(comp)];
    posy = posy & pos_mask_y_[static_cast<int>(comp)];
    const Coeff *data = &comp_storage_[static_cast<int>(comp)][0];
    return DataBuffer<const Coeff>(data + posy * kStride + posx, kStride);
  }

private:
  static const int kStride = constants::kMaxBlockSize;
  std::array<int, constants::kMaxYuvComponents> pos_mask_x_;
  std::array<int, constants::kMaxYuvComponents> pos_mask_y_;
  std::array<std::array<Coeff, constants::kMaxBlockSamples * kStride>,
    constants::kMaxYuvComponents> comp_storage_;
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_SAMPLE_BUFFER_H_
