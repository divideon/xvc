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
  void Transform(int width, int height, bool is_luma_intra, const Coeff *coeff,
                 ptrdiff_t coeff_stride, Residual *resi, ptrdiff_t resi_stride);

private:
  static const ptrdiff_t kBufferStride_ = constants::kMaxBlockSize;
  void InvPartialDST4(int shift, const Coeff *in,
                      ptrdiff_t in_stride, Coeff *out, ptrdiff_t out_stride);
  void InvPartialTransform2(int shift, int lines,
                            const Coeff *in, ptrdiff_t in_stride,
                            Coeff *out, ptrdiff_t out_stride);
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
  void InvPartialTransform64(int shift, int lines, bool skip_full_height,
                             const Coeff *in, ptrdiff_t in_stride,
                             Coeff *out, ptrdiff_t out_stride);

  int bitdepth_;
  std::array<Coeff, kBufferStride_ * kBufferStride_> coeff_temp_;
};

class ForwardTransform {
public:
  explicit ForwardTransform(int bitdepth) : bitdepth_(bitdepth) {}
  void Transform(int width, int height, bool is_luma_intra,
                 const Residual *resi, ptrdiff_t resi_stride,
                 Coeff *coeff, ptrdiff_t coeff_stride);

private:
  static const ptrdiff_t kBufferStride_ = constants::kMaxBlockSize;
  void FwdPartialDST4(int shift, const Coeff *in, ptrdiff_t in_stride,
                      Coeff *out, ptrdiff_t out_stride);
  void FwdPartialTransform2(int shift, int lines,
                            const Coeff *in, ptrdiff_t in_stride,
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
  template<bool ZeroWdt, bool ZeroHgt>
  void FwdPartialTransform64(int shift, int lines,
                             const Coeff *in, ptrdiff_t in_stride,
                             Coeff *out, ptrdiff_t out_stride);

  int bitdepth_;
  std::array<Coeff, kBufferStride_ * kBufferStride_> coeff_temp_;
};

class TransformHelper {
public:
  static const std::array<uint8_t, 128> kLastPosGroupIdx;
  static const std::array<uint8_t, 14> kLastPosMinInGroup;
  static const std::array<std::array<uint8_t, 4>, 3> kScanCoeff2x2;
  static const std::array<std::array<uint8_t, 16>, 3> kScanCoeff4x4;

  static ScanOrder DetermineScanOrder(const CodingUnit & cu,
                                      YuvComponent comp);
  static void DeriveSubblockScan(ScanOrder scan_order, int width,
                                 int height, uint16_t *scan_table);
  static const uint8_t* GetCoeffScanTable2x2(ScanOrder scan_order) {
    return &kScanCoeff2x2[static_cast<int>(scan_order)][0];
  }
  static const uint8_t* GetCoeffScanTable4x4(ScanOrder scan_order) {
    return &kScanCoeff4x4[static_cast<int>(scan_order)][0];
  }
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_TRANSFORM_H_
