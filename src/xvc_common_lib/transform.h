/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#ifndef XVC_COMMON_LIB_TRANSFORM_H_
#define XVC_COMMON_LIB_TRANSFORM_H_

#include <array>
#include <vector>

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/common.h"

namespace xvc {

enum class ScanOrder : int {
  kDiagonal = 0,
  kHorizontal = 1,
  kVertical = 2,
  kTotalNumber = 3,
};

class InverseTransform {
public:
  explicit InverseTransform(int bitdepth) : bitdepth_(bitdepth) {}
  void Transform(int size, const Coeff *coeff, ptrdiff_t coeff_stride,
                 Residual *resi, ptrdiff_t resi_stride,
                 bool dst_transform = false);
  void TransformDST(int size, const Coeff *coeff, ptrdiff_t coeff_stride,
                    Residual *resi, ptrdiff_t resi_stride);

private:
  static const ptrdiff_t kBufferStride_ = constants::kMaxBlockSize;
  void InvPartialDST4(int shift, const Coeff *in,
                      ptrdiff_t in_stride, Coeff *out, ptrdiff_t out_stride);
  void InvPartialTransform4(int shift, int lines,
                            const Coeff *in, ptrdiff_t in_stride,
                            Coeff *out, ptrdiff_t out_stride);
  void InvPartialTransform8(int shift, int lines,
                            const Coeff *in, ptrdiff_t in_stride,
                            Coeff *out, ptrdiff_t out_stride);
  void InvPartialTransform16(int shift, int lines,
                             const Coeff *in, ptrdiff_t in_stride,
                             Coeff *out, ptrdiff_t out_stride);
  void InvPartialTransform32(int shift, int lines,
                             const Coeff *in, ptrdiff_t in_stride,
                             Coeff *out, ptrdiff_t out_stride);

  int bitdepth_;
  std::array<Coeff, kBufferStride_ * kBufferStride_> coeff_temp_;
};

class ForwardTransform {
public:
  explicit ForwardTransform(int bitdepth) : bitdepth_(bitdepth) {}
  void Transform(int size, const Residual *resi, ptrdiff_t resi_stride,
                 Coeff *coeff, ptrdiff_t coeff_stride,
                 bool dst_transform = false);
  void TransformDST(int size, const Residual *resi, ptrdiff_t resi_stride,
                    Coeff *coeff, ptrdiff_t coeff_stride);

private:
  static const ptrdiff_t kBufferStride_ = constants::kMaxBlockSize;
  void FwdPartialDST4(int shift, const Coeff *in, ptrdiff_t in_stride,
                      Coeff *out, ptrdiff_t out_stride);
  void FwdPartialTransform4(int shift, int lines,
                            const Coeff *in, ptrdiff_t in_stride,
                            Coeff *out, ptrdiff_t out_stride);
  void FwdPartialTransform8(int shift, int lines,
                            const Coeff *in, ptrdiff_t in_stride,
                            Coeff *out, ptrdiff_t out_stride);
  void FwdPartialTransform16(int shift, int lines,
                             const Coeff *in, ptrdiff_t in_stride,
                             Coeff *out, ptrdiff_t out_stride);
  void FwdPartialTransform32(int shift, int lines,
                             const Coeff *in, ptrdiff_t in_stride,
                             Coeff *out, ptrdiff_t out_stride);

  int bitdepth_;
  std::array<Coeff, kBufferStride_ * kBufferStride_> coeff_temp_;
};

class TransformHelper {
public:
  static const std::array<uint8_t, 128> kLastPosGroupIdx;
  static const std::array<uint8_t, 14> kLastPosMinInGroup;

  static ScanOrder DetermineScanOrder(const CodingUnit & cu,
                                      YuvComponent comp);
  static const uint16_t *GetScanTable(int width, int height,
                                      ScanOrder scan_order);
  static const uint16_t *GetScanTableSubblock(int width, int height,
                                              ScanOrder scan_order);
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_TRANSFORM_H_
