/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_SAMPLE_BUFFER_H_
#define XVC_COMMON_LIB_SAMPLE_BUFFER_H_

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

class SampleBuffer : public DataBuffer<Sample> {
public:
  SampleBuffer(Sample *data, ptrdiff_t stride) : DataBuffer(data, stride) {}

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
};

class ResidualBuffer : public DataBuffer<Residual> {
public:
  ResidualBuffer(Residual *data, ptrdiff_t stride) : DataBuffer(data, stride) {}

  void Subtract(int width, int height,
                const DataBuffer<const Sample> &src1_buffer,
                const DataBuffer<const Sample> &src2_buffer) {
    const Sample *src1 = src1_buffer.GetDataPtr();
    const Sample *src2 = src2_buffer.GetDataPtr();
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

class CoeffBuffer : public DataBuffer<Coeff> {
public:
  CoeffBuffer(Coeff *data, ptrdiff_t stride) : DataBuffer(data, stride) {}
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

}   // namespace xvc

#endif  // XVC_COMMON_LIB_SAMPLE_BUFFER_H_
