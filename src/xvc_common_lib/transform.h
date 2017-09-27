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

#ifndef XVC_COMMON_LIB_TRANSFORM_H_
#define XVC_COMMON_LIB_TRANSFORM_H_

#include <array>
#include <vector>

#include "xvc_common_lib/coding_unit.h"
#include "xvc_common_lib/common.h"
#include "xvc_common_lib/transform_data.h"

namespace xvc {

enum class ScanOrder : int {
  kDiagonal = 0,
  kHorizontal = 1,
  kVertical = 2,
  kTotalNumber = 3,
};

class InverseTransform : public TransformData {
public:
  explicit InverseTransform(int bitdepth) : bitdepth_(bitdepth) {}
  void Transform(const CodingUnit &cu, YuvComponent comp,
                 const CoeffBuffer &in_buffer, ResidualBuffer *out_buffer);
  void TransformSkip(int width, int height,
                     const CoeffBuffer &in_buffer, ResidualBuffer *out_buffer);

private:
  static const int kTransformExtendedPrecision = 2;
  static const ptrdiff_t kBufferStride_ = constants::kMaxBlockSize;
  void InvPartialDST4(int shift, bool high_prec, const Coeff *in,
                      ptrdiff_t in_stride, Coeff *out, ptrdiff_t out_stride);
  void InvDct2(int size, int shift, int lines, bool high_prec, bool zero_out,
               const Coeff *in, ptrdiff_t in_stride,
               Coeff *out, ptrdiff_t out_stride);
  void InvDct5(int size, int shift, int lines, bool high_prec, bool zero_out,
               const Coeff *in, ptrdiff_t in_stride,
               Coeff *out, ptrdiff_t out_stride);
  void InvDct8(int size, int shift, int lines, bool high_prec, bool zero_out,
               const Coeff *in, ptrdiff_t in_stride,
               Coeff *out, ptrdiff_t out_stride);
  void InvDst1(int size, int shift, int lines, bool high_prec, bool zero_out,
               const Coeff *in, ptrdiff_t in_stride,
               Coeff *out, ptrdiff_t out_stride);
  void InvDst7(int size, int shift, int lines, bool high_prec, bool zero_out,
               const Coeff *in, ptrdiff_t in_stride,
               Coeff *out, ptrdiff_t out_stride);
  void InvDct2Transform2(int shift, int lines,
                         bool high_prec, bool zero_out,
                         const Coeff *in, ptrdiff_t in_stride,
                         Coeff *out, ptrdiff_t out_stride);
  void InvDct2Transform4(int shift, int lines,
                         bool high_prec, bool zero_out,
                         const Coeff *in, ptrdiff_t in_stride,
                         Coeff *out, ptrdiff_t out_stride);
  void InvDct2Transform8(int shift, int lines,
                         bool high_prec, bool zero_out,
                         const Coeff *in, ptrdiff_t in_stride,
                         Coeff *out, ptrdiff_t out_stride);
  void InvDct2Transform16(int shift, int lines,
                          bool high_prec, bool zero_out,
                          const Coeff *in, ptrdiff_t in_stride,
                          Coeff *out, ptrdiff_t out_stride);
  void InvDct2Transform32(int shift, int lines,
                          bool high_prec, bool zero_out,
                          const Coeff *in, ptrdiff_t in_stride,
                          Coeff *out, ptrdiff_t out_stride);
  void InvDct2Transform64(int shift, int lines,
                          bool high_prec, bool zero_out,
                          const Coeff *in, ptrdiff_t in_stride,
                          Coeff *out, ptrdiff_t out_stride);
  template<int N>
  void InvGenericTransformN(int shift, int lines, bool zero_out,
                            const Coeff(&kMatrix)[N * N],
                            const Coeff *in, ptrdiff_t in_stride,
                            Coeff *out, ptrdiff_t out_stride);

  int bitdepth_;
  std::array<Coeff, kBufferStride_ * kBufferStride_> coeff_temp_;
};

class ForwardTransform : public TransformData {
public:
  explicit ForwardTransform(int bitdepth) : bitdepth_(bitdepth) {}
  void Transform(const CodingUnit &cu, YuvComponent comp,
                 const ResidualBuffer &in_buffer, CoeffBuffer *out_buffer);
  void TransformSkip(int width, int height,
                     const ResidualBuffer &in_buffer, CoeffBuffer *out_buffer);

private:
  static const ptrdiff_t kBufferStride_ = constants::kMaxBlockSize;
  void FwdDct2(int size, int shift, int lines, bool high_prec, bool zero_out,
               const Coeff *in, ptrdiff_t in_stride,
               Coeff *out, ptrdiff_t out_stride);
  void FwdDct5(int size, int shift, int lines, bool high_prec, bool zero_out,
               const Coeff *in, ptrdiff_t in_stride,
               Coeff *out, ptrdiff_t out_stride);
  void FwdDct8(int size, int shift, int lines, bool high_prec, bool zero_out,
               const Coeff *in, ptrdiff_t in_stride,
               Coeff *out, ptrdiff_t out_stride);
  void FwdDst1(int size, int shift, int lines, bool high_prec, bool zero_out,
               const Coeff *in, ptrdiff_t in_stride,
               Coeff *out, ptrdiff_t out_stride);
  void FwdDst7(int size, int shift, int lines, bool high_prec, bool zero_out,
               const Coeff *in, ptrdiff_t in_stride,
               Coeff *out, ptrdiff_t out_stride);
  void FwdPartialDST4(int shift, bool high_prec,
                      const Coeff *in, ptrdiff_t in_stride,
                      Coeff *out, ptrdiff_t out_stride);
  void FwdDct2Transform2(int shift, int lines,
                         bool high_prec, bool zero_out,
                         const Coeff *in, ptrdiff_t in_stride,
                         Coeff *out, ptrdiff_t out_stride);
  void FwdDct2Transform4(int shift, int lines,
                         bool high_prec, bool zero_out,
                         const Coeff *in, ptrdiff_t in_stride,
                         Coeff *out, ptrdiff_t out_stride);
  void FwdDct2Transform8(int shift, int lines,
                         bool high_prec, bool zero_out,
                         const Coeff *in, ptrdiff_t in_stride,
                         Coeff *out, ptrdiff_t out_stride);
  void FwdDct2Transform16(int shift, int lines,
                          bool high_prec, bool zero_out,
                          const Coeff *in, ptrdiff_t in_stride,
                          Coeff *out, ptrdiff_t out_stride);
  void FwdDct2Transform32(int shift, int lines,
                          bool high_prec, bool zero_out,
                          const Coeff *in, ptrdiff_t in_stride,
                          Coeff *out, ptrdiff_t out_stride);
  void FwdDct2Transform64(int shift, int lines,
                          bool high_prec, bool zero_out,
                          const Coeff *in, ptrdiff_t in_stride,
                          Coeff *out, ptrdiff_t out_stride);
  template<int N>
  void FwdGenericTransformN(int shift, int lines, bool zero_out,
                            const Coeff(&kMatrix)[N * N],
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

  static ScanOrder DetermineScanOrder(const CodingUnit &cu,
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
