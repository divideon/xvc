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

#include "xvc_common_lib/transform.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "xvc_common_lib/intra_prediction.h"
#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"

#if _MSC_VER
// Disable warning 4244 in Visual Studio in order to be able to do implicit
// type conversions (without static_cast).
#pragma warning(disable:4244)
// Disables C6294 Ill-defined for-loop: initial condition does not satisfy test.
// Some transform logic depends on the zero out threshold constant
#pragma warning(disable:6294)
#endif

namespace xvc {

static const int kTransformHighPrecisionShift = 2;  // 8 bits instead of 6

const std::array<uint8_t, 128> TransformHelper::kLastPosGroupIdx = { {
  0, 1, 2, 3, 4, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9,
  9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10,
  10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12,
  12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12,
  12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13,
  13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13,
  13, 13, 13, 13
} };

const std::array<uint8_t, 14> TransformHelper::kLastPosMinInGroup = { {
  0, 1, 2, 3, 4, 6, 8, 12, 16, 24, 32, 48, 64, 96
} };

const std::array<uint8_t, 10> TransformHelper::kGolombRiceRangeExt = { {
  6, 5, 6, 3, 3, 3, 3, 3, 3, 3
} };

const std::array<std::array<uint8_t, 4>, 3> TransformHelper::kScanCoeff2x2 = { {
  { 0, 2, 1, 3 },
  { 0, 1, 2, 3 },
  { 0, 2, 1, 3 },
} };

const
std::array<std::array<uint8_t, 16>, 3> TransformHelper::kScanCoeff4x4 = { {
  { 0, 4, 1, 8, 5, 2, 12, 9, 6, 3, 13, 10, 7, 14, 11, 15 },
  { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15 },
  { 0, 4, 8, 12, 1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15 },
} };

void InverseTransform::Transform(const CodingUnit &cu, YuvComponent comp,
                                 const CoeffBuffer &in_buffer,
                                 ResidualBuffer *out_buffer) {
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const bool can_dst_4x4 = util::IsLuma(comp) && cu.IsIntra();
  const bool default_high_precision =
    !Restrictions::Get().disable_ext_transform_high_precision;
  // TODO(PH) Should remove legacy behavior and only look at restriction flag
  const bool high_prec1 = default_high_precision || height >= 64 || height == 2;
  const bool high_prec2 = default_high_precision || width >= 64 || width == 2;
  const int shift1 = 7 +
    (high_prec1 ? kTransformHighPrecisionShift : 0);
  const int shift2 = 20 - bitdepth_ +
    (high_prec2 ? kTransformHighPrecisionShift : 0);
  const Coeff *coeff = in_buffer.GetDataPtr();
  const ptrdiff_t coeff_stride = in_buffer.GetStride();
  Residual *resi = out_buffer->GetDataPtr();
  const ptrdiff_t resi_stride = out_buffer->GetStride();
  Coeff *temp_ptr = &coeff_temp_[0];
  const ptrdiff_t temp_stride = kBufferStride_;

  switch (cu.GetTransformType(comp, 0)) {
    case TransformType::kDefault:
      if (can_dst_4x4 && width == 4 && height == 4) {
        InvPartialDST4(shift1, high_prec1, coeff, coeff_stride,
                       temp_ptr, temp_stride);
      } else {
        InvDct2(height, shift1, width, high_prec1, true,
                coeff, coeff_stride, temp_ptr, temp_stride);
      }
      break;
    case TransformType::kDCT2:
      InvDct2(height, shift1, width, high_prec1, true,
              coeff, coeff_stride, temp_ptr, temp_stride);
      break;
    case TransformType::kDCT5:
      InvDct5(height, shift1, width, high_prec1, true,
              coeff, coeff_stride, temp_ptr, temp_stride);
      break;
    case TransformType::kDCT8:
      InvDct8(height, shift1, width, high_prec1, true,
              coeff, coeff_stride, temp_ptr, temp_stride);
      break;
    case TransformType::kDST1:
      InvDst1(height, shift1, width, high_prec1, true,
              coeff, coeff_stride, temp_ptr, temp_stride);
      break;
    case TransformType::kDST7:
      InvDst7(height, shift1, width, high_prec1, true,
              coeff, coeff_stride, temp_ptr, temp_stride);
      break;
    default:
      assert(0);
      break;
  }

  switch (cu.GetTransformType(comp, 1)) {
    case TransformType::kDefault:
      if (can_dst_4x4 && width == 4 && height == 4) {
        InvPartialDST4(shift2, high_prec2, temp_ptr, temp_stride,
                       resi, resi_stride);
      } else {
        InvDct2(width, shift2, height, high_prec2, false,
                temp_ptr, temp_stride, resi, resi_stride);
      }
      break;
    case TransformType::kDCT2:
      InvDct2(width, shift2, height, high_prec2, false,
              temp_ptr, temp_stride, resi, resi_stride);
      break;
    case TransformType::kDCT5:
      InvDct5(width, shift2, height, high_prec2, false,
              temp_ptr, temp_stride, resi, resi_stride);
      break;
    case TransformType::kDCT8:
      InvDct8(width, shift2, height, high_prec2, false,
              temp_ptr, temp_stride, resi, resi_stride);
      break;
    case TransformType::kDST1:
      InvDst1(width, shift2, height, high_prec2, false,
              temp_ptr, temp_stride, resi, resi_stride);
      break;
    case TransformType::kDST7:
      InvDst7(width, shift2, height, high_prec2, false,
              temp_ptr, temp_stride, resi, resi_stride);
      break;
    default:
      assert(0);
      break;
  }
}

void InverseTransform::TransformSkip(int width, int height,
                                     const CoeffBuffer &in_buffer,
                                     ResidualBuffer *out_buffer) {
  const Coeff *coeff = in_buffer.GetDataPtr();
  const ptrdiff_t coeff_stride = in_buffer.GetStride();
  Residual *resi = out_buffer->GetDataPtr();
  const ptrdiff_t resi_stride = out_buffer->GetStride();

  // TODO(PH) Duplicate code that should be extracted to common function
  const bool size_rounding_bias =
    (util::SizeToLog2(width) + util::SizeToLog2(height)) % 2 != 0;
  const int transform_shift =
    Quantize::GetTransformShift(width, height, bitdepth_);
  const int shift = transform_shift + (size_rounding_bias ? 7 : 0);
  const int scale = size_rounding_bias ? 181 : 1;
  if (shift > 0) {
    const int offset = (1 << (shift - 1));
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        resi[y * resi_stride + x] =
          (coeff[y * coeff_stride + x] * scale + offset) >> shift;
      }
    }
  } else {
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        resi[y * resi_stride + x] =
          (coeff[y * coeff_stride + x] * scale) << -shift;
      }
    }
  }
}

void InverseTransform::InvPartialDST4(int shift, bool high_prec,
                                      const Coeff *in, ptrdiff_t in_stride,
                                      Coeff *out, ptrdiff_t out_stride) {
  // No support for high precision for DST 4x4
  shift -= high_prec ? kTransformHighPrecisionShift : 0;
  const int add = 1 << (shift - 1);
  int c[4];

  for (int i = 0; i < 4; i++) {
    c[0] = in[0] + in[2 * in_stride];
    c[1] = in[2 * in_stride] + in[3 * in_stride];
    c[2] = in[0] - in[3 * in_stride];
    c[3] = 74 * in[1 * in_stride];
    out[0] = util::Clip3((29 * c[0] + 55 * c[1] + c[3] + add) >> shift,
                         constants::kInt16Min, constants::kInt16Max);
    out[1] = util::Clip3((55 * c[2] - 29 * c[1] + c[3] + add) >> shift,
                         constants::kInt16Min, constants::kInt16Max);
    out[2] = util::Clip3((74 * (in[0] - in[2 * in_stride] +
                                in[3 * in_stride]) + add) >> shift,
                         constants::kInt16Min, constants::kInt16Max);
    out[3] = util::Clip3((55 * c[0] + 29 * c[2] - c[3] + add) >> shift,
                         constants::kInt16Min, constants::kInt16Max);
    in++;
    out += out_stride;
  }
}

void InverseTransform::InvDct2(int size, int shift, int lines,
                               bool high_prec, bool zero_out,
                               const Coeff *in, ptrdiff_t in_stride,
                               Coeff *out, ptrdiff_t out_stride) {
  switch (size) {
    case 2:
      InvDct2Transform2(shift, lines, high_prec, zero_out,
                        in, in_stride, out, out_stride);
      break;
    case 4:
      InvDct2Transform4(shift, lines, high_prec, zero_out,
                        in, in_stride, out, out_stride);
      break;
    case 8:
      InvDct2Transform8(shift, lines, high_prec, zero_out,
                        in, in_stride, out, out_stride);
      break;
    case 16:
      InvDct2Transform16(shift, lines, high_prec, zero_out,
                         in, in_stride, out, out_stride);
      break;
    case 32:
      InvDct2Transform32(shift, lines, high_prec, zero_out,
                         in, in_stride, out, out_stride);
      break;
    case 64:
      InvDct2Transform64(shift, lines, high_prec, zero_out,
                         in, in_stride, out, out_stride);
      break;
    default:
      assert(0);
      break;
  }
}

void InverseTransform::InvDct5(int size, int shift, int lines,
                               bool high_prec, bool zero_out,
                               const Coeff *in, ptrdiff_t in_stride,
                               Coeff *out, ptrdiff_t out_stride) {
  // Only high precision matrices supported
  shift += !high_prec ? kTransformHighPrecisionShift : 0;
  switch (size) {
    case 4:
      InvGenericTransformN<4>(shift, lines, zero_out, kDct5Transform4High,
                              in, in_stride, out, out_stride);
      break;
    case 8:
      InvGenericTransformN<8>(shift, lines, zero_out, kDct5Transform8High,
                              in, in_stride, out, out_stride);
      break;
    case 16:
      InvGenericTransformN<16>(shift, lines, zero_out, kDct5Transform16High,
                               in, in_stride, out, out_stride);
      break;
    case 32:
      InvGenericTransformN<32>(shift, lines, zero_out, kDct5Transform32High,
                               in, in_stride, out, out_stride);
      break;
    case 64:
      InvGenericTransformN<64>(shift, lines, zero_out, kDct5Transform64High,
                               in, in_stride, out, out_stride);
      break;
    default:
      assert(0);
      break;
  }
}

void InverseTransform::InvDct8(int size, int shift, int lines,
                               bool high_prec, bool zero_out,
                               const Coeff *in, ptrdiff_t in_stride,
                               Coeff *out, ptrdiff_t out_stride) {
  // Only high precision matrices supported
  shift += !high_prec ? kTransformHighPrecisionShift : 0;
  switch (size) {
    case 4:
      InvGenericTransformN<4>(shift, lines, zero_out, kDct8Transform4High,
                              in, in_stride, out, out_stride);
      break;
    case 8:
      InvGenericTransformN<8>(shift, lines, zero_out, kDct8Transform8High,
                              in, in_stride, out, out_stride);
      break;
    case 16:
      InvGenericTransformN<16>(shift, lines, zero_out, kDct8Transform16High,
                               in, in_stride, out, out_stride);
      break;
    case 32:
      InvGenericTransformN<32>(shift, lines, zero_out, kDct8Transform32High,
                               in, in_stride, out, out_stride);
      break;
    case 64:
      InvGenericTransformN<64>(shift, lines, zero_out, kDct8Transform64High,
                               in, in_stride, out, out_stride);
      break;
    default:
      assert(0);
      break;
  }
}

void InverseTransform::InvDst1(int size, int shift, int lines,
                               bool high_prec, bool zero_out,
                               const Coeff *in, ptrdiff_t in_stride,
                               Coeff *out, ptrdiff_t out_stride) {
  // Only high precision matrices supported
  shift += !high_prec ? kTransformHighPrecisionShift : 0;
  switch (size) {
    case 4:
      InvGenericTransformN<4>(shift, lines, zero_out, kDst1Transform4High,
                              in, in_stride, out, out_stride);
      break;
    case 8:
      InvGenericTransformN<8>(shift, lines, zero_out, kDst1Transform8High,
                              in, in_stride, out, out_stride);
      break;
    case 16:
      InvGenericTransformN<16>(shift, lines, zero_out, kDst1Transform16High,
                               in, in_stride, out, out_stride);
      break;
    case 32:
      InvGenericTransformN<32>(shift, lines, zero_out, kDst1Transform32High,
                               in, in_stride, out, out_stride);
      break;
    case 64:
      InvGenericTransformN<64>(shift, lines, zero_out, kDst1Transform64High,
                               in, in_stride, out, out_stride);
      break;
    default:
      assert(0);
      break;
  }
}

void InverseTransform::InvDst7(int size, int shift, int lines,
                               bool high_prec, bool zero_out,
                               const Coeff *in, ptrdiff_t in_stride,
                               Coeff *out, ptrdiff_t out_stride) {
  // Only high precision matrices supported
  shift += !high_prec ? kTransformHighPrecisionShift : 0;
  switch (size) {
    case 4:
      InvGenericTransformN<4>(shift, lines, zero_out, kDst7Transform4High,
                              in, in_stride, out, out_stride);
      break;
    case 8:
      InvGenericTransformN<8>(shift, lines, zero_out, kDst7Transform8High,
                              in, in_stride, out, out_stride);
      break;
    case 16:
      InvGenericTransformN<16>(shift, lines, zero_out, kDst7Transform16High,
                               in, in_stride, out, out_stride);
      break;
    case 32:
      InvGenericTransformN<32>(shift, lines, zero_out, kDst7Transform32High,
                               in, in_stride, out, out_stride);
      break;
    case 64:
      InvGenericTransformN<64>(shift, lines, zero_out, kDst7Transform64High,
                               in, in_stride, out, out_stride);
      break;
    default:
      assert(0);
      break;
  }
}

void
InverseTransform::InvDct2Transform2(int shift, int lines,
                                    bool high_prec, bool zero_out,
                                    const Coeff *in, ptrdiff_t in_stride,
                                    Coeff *out, ptrdiff_t out_stride) {
  // Only high precision matrices supported
  shift += !high_prec ? kTransformHighPrecisionShift : 0;
  const auto &kMatrix = kDct2Transform2High;
  const int add = 1 << (shift - 1);
  const int tx_lines = zero_out ?
    std::min(lines, constants::kTransformZeroOutMinSize) : lines;
  int O[1], E[1];

  for (int y = 0; y < tx_lines; y++) {
    O[0] = kMatrix[1][0] * in[0 * in_stride] -
      kMatrix[1][0] * in[1 * in_stride];
    E[0] = kMatrix[0][0] * in[0 * in_stride] +
      kMatrix[0][0] * in[1 * in_stride];
    out[0] = util::Clip3((E[0] + add) >> shift,
                         constants::kInt16Min, constants::kInt16Max);
    out[1] = util::Clip3((O[0] + add) >> shift,
                         constants::kInt16Min, constants::kInt16Max);
    in++;
    out += out_stride;
  }
  for (int y = tx_lines; y < lines; y++) {
    memset(out, 0, sizeof(Coeff) * 2);
    out += out_stride;
  }
}

void
InverseTransform::InvDct2Transform4(int shift, int lines,
                                    bool high_prec, bool zero_out,
                                    const Coeff *in, ptrdiff_t in_stride,
                                    Coeff *out, ptrdiff_t out_stride) {
  const auto &kMatrix = high_prec ? kDct2Transform4High : kDct2Transform4;
  const int add = 1 << (shift - 1);
  const int tx_lines = zero_out ?
    std::min(lines, constants::kTransformZeroOutMinSize) : lines;
  int O[2], E[2];

  for (int y = 0; y < tx_lines; y++) {
    O[0] = kMatrix[1][0] * in[1 * in_stride] +
      kMatrix[3][0] * in[3 * in_stride];
    O[1] = kMatrix[1][1] * in[1 * in_stride] +
      kMatrix[3][1] * in[3 * in_stride];
    E[0] = kMatrix[0][0] * in[0 * in_stride] +
      kMatrix[2][0] * in[2 * in_stride];
    E[1] = kMatrix[0][1] * in[0 * in_stride] +
      kMatrix[2][1] * in[2 * in_stride];
    out[0] = util::Clip3((E[0] + O[0] + add) >> shift,
                         constants::kInt16Min, constants::kInt16Max);
    out[1] = util::Clip3((E[1] + O[1] + add) >> shift,
                         constants::kInt16Min, constants::kInt16Max);
    out[2] = util::Clip3((E[1] - O[1] + add) >> shift,
                         constants::kInt16Min, constants::kInt16Max);
    out[3] = util::Clip3((E[0] - O[0] + add) >> shift,
                         constants::kInt16Min, constants::kInt16Max);
    in++;
    out += out_stride;
  }
  for (int y = tx_lines; y < lines; y++) {
    memset(out, 0, sizeof(Coeff) * 4);
    out += out_stride;
  }
}

void
InverseTransform::InvDct2Transform8(int shift, int lines,
                                    bool high_prec, bool zero_out,
                                    const Coeff *in, ptrdiff_t in_stride,
                                    Coeff *out, ptrdiff_t out_stride) {
  const auto &kMatrix = high_prec ? kDct2Transform8High : kDct2Transform8;
  const int add = 1 << (shift - 1);
  const int tx_lines = zero_out ?
    std::min(lines, constants::kTransformZeroOutMinSize) : lines;
  int O[4], E[4];
  int EE[2], EO[2];

  for (int y = 0; y < tx_lines; y++) {
    for (int k = 0; k < 4; k++) {
      O[k] = kMatrix[1][k] * in[1 * in_stride] +
        kMatrix[3][k] * in[3 * in_stride] +
        kMatrix[5][k] * in[5 * in_stride] +
        kMatrix[7][k] * in[7 * in_stride];
    }
    EO[0] = kMatrix[2][0] * in[2 * in_stride] +
      kMatrix[6][0] * in[6 * in_stride];
    EO[1] = kMatrix[2][1] * in[2 * in_stride] +
      kMatrix[6][1] * in[6 * in_stride];
    EE[0] = kMatrix[0][0] * in[0 * in_stride] +
      kMatrix[4][0] * in[4 * in_stride];
    EE[1] = kMatrix[0][1] * in[0 * in_stride] +
      kMatrix[4][1] * in[4 * in_stride];
    E[0] = EE[0] + EO[0];
    E[3] = EE[0] - EO[0];
    E[1] = EE[1] + EO[1];
    E[2] = EE[1] - EO[1];
    for (int k = 0; k < 4; ++k) {
      out[k] = util::Clip3((E[k] + O[k] + add) >> shift,
                           constants::kInt16Min, constants::kInt16Max);
      out[k + 4] = util::Clip3((E[3 - k] - O[3 - k] + add) >> shift,
                               constants::kInt16Min, constants::kInt16Max);
    }
    in++;
    out += out_stride;
  }
  for (int y = tx_lines; y < lines; y++) {
    memset(out, 0, sizeof(Coeff) * 8);
    out += out_stride;
  }
}

void
InverseTransform::InvDct2Transform16(int shift, int lines,
                                     bool high_prec, bool zero_out,
                                     const Coeff *in, ptrdiff_t in_stride,
                                     Coeff *out, ptrdiff_t out_stride) {
  const auto &kMatrix = high_prec ? kDct2Transform16High : kDct2Transform16;
  const int add = 1 << (shift - 1);
  const int tx_lines = zero_out ?
    std::min(lines, constants::kTransformZeroOutMinSize) : lines;
  int O[8], E[8];
  int EO[4], EE[4];
  int EEO[2], EEE[2];

  for (int y = 0; y < tx_lines; y++) {
    for (int k = 0; k < 8; k++) {
      O[k] = kMatrix[1][k] * in[in_stride]
        + kMatrix[3][k] * in[3 * in_stride]
        + kMatrix[5][k] * in[5 * in_stride]
        + kMatrix[7][k] * in[7 * in_stride]
        + kMatrix[9][k] * in[9 * in_stride]
        + kMatrix[11][k] * in[11 * in_stride]
        + kMatrix[13][k] * in[13 * in_stride]
        + kMatrix[15][k] * in[15 * in_stride];
    }
    for (int k = 0; k < 4; k++) {
      EO[k] = kMatrix[2][k] * in[2 * in_stride]
        + kMatrix[6][k] * in[6 * in_stride]
        + kMatrix[10][k] * in[10 * in_stride]
        + kMatrix[14][k] * in[14 * in_stride];
    }
    EEO[0] = kMatrix[4][0] * in[4 * in_stride]
      + kMatrix[12][0] * in[12 * in_stride];
    EEE[0] = kMatrix[0][0] * in[0 * in_stride]
      + kMatrix[8][0] * in[8 * in_stride];
    EEO[1] = kMatrix[4][1] * in[4 * in_stride]
      + kMatrix[12][1] * in[12 * in_stride];
    EEE[1] = kMatrix[0][1] * in[0 * in_stride]
      + kMatrix[8][1] * in[8 * in_stride];
    for (int k = 0; k < 2; k++) {
      EE[k] = EEE[k] + EEO[k];
      EE[k + 2] = EEE[1 - k] - EEO[1 - k];
    }
    for (int k = 0; k < 4; k++) {
      E[k] = EE[k] + EO[k];
      E[k + 4] = EE[3 - k] - EO[3 - k];
    }
    for (int k = 0; k < 8; k++) {
      out[k] = util::Clip3((E[k] + O[k] + add) >> shift,
                           constants::kInt16Min, constants::kInt16Max);
      out[k + 8] = util::Clip3((E[7 - k] - O[7 - k] + add) >> shift,
                               constants::kInt16Min, constants::kInt16Max);
    }
    in++;
    out += out_stride;
  }
  for (int y = tx_lines; y < lines; y++) {
    memset(out, 0, sizeof(Coeff) * 16);
    out += out_stride;
  }
}

void
InverseTransform::InvDct2Transform32(int shift, int lines,
                                     bool high_prec, bool zero_out,
                                     const Coeff *in, ptrdiff_t in_stride,
                                     Coeff *out, ptrdiff_t out_stride) {
  const auto &kMatrix = high_prec ? kDct2Transform32High : kDct2Transform32;
  const int add = 1 << (shift - 1);
  const int tx_lines = zero_out ?
    std::min(lines, constants::kTransformZeroOutMinSize) : lines;
  int O[16], E[16];
  int EO[8], EE[8];
  int EEO[4], EEE[4];
  int EEEO[2], EEEE[2];

  for (int y = 0; y < tx_lines; y++) {
    for (int k = 0; k < 16; k++) {
      O[k] =
        kMatrix[1][k] * in[1 * in_stride] +
        kMatrix[3][k] * in[3 * in_stride] +
        kMatrix[5][k] * in[5 * in_stride] +
        kMatrix[7][k] * in[7 * in_stride] +
        kMatrix[9][k] * in[9 * in_stride] +
        kMatrix[11][k] * in[11 * in_stride] +
        kMatrix[13][k] * in[13 * in_stride] +
        kMatrix[15][k] * in[15 * in_stride] +
        kMatrix[17][k] * in[17 * in_stride] +
        kMatrix[19][k] * in[19 * in_stride] +
        kMatrix[21][k] * in[21 * in_stride] +
        kMatrix[23][k] * in[23 * in_stride] +
        kMatrix[25][k] * in[25 * in_stride] +
        kMatrix[27][k] * in[27 * in_stride] +
        kMatrix[29][k] * in[29 * in_stride] +
        kMatrix[31][k] * in[31 * in_stride];
    }

    for (int k = 0; k < 8; k++) {
      EO[k] =
        kMatrix[2][k] * in[2 * in_stride] +
        kMatrix[6][k] * in[6 * in_stride] +
        kMatrix[10][k] * in[10 * in_stride] +
        kMatrix[14][k] * in[14 * in_stride] +
        kMatrix[18][k] * in[18 * in_stride] +
        kMatrix[22][k] * in[22 * in_stride] +
        kMatrix[26][k] * in[26 * in_stride] +
        kMatrix[30][k] * in[30 * in_stride];
    }
    for (int k = 0; k < 4; k++) {
      EEO[k] =
        kMatrix[4][k] * in[4 * in_stride] +
        kMatrix[12][k] * in[12 * in_stride] +
        kMatrix[20][k] * in[20 * in_stride] +
        kMatrix[28][k] * in[28 * in_stride];
    }
    EEEO[0] = kMatrix[8][0] * in[8 * in_stride] +
      kMatrix[24][0] * in[24 * in_stride];
    EEEO[1] = kMatrix[8][1] * in[8 * in_stride] +
      kMatrix[24][1] * in[24 * in_stride];
    EEEE[0] = kMatrix[0][0] * in[0] +
      kMatrix[16][0] * in[16 * in_stride];
    EEEE[1] = kMatrix[0][1] * in[0] +
      kMatrix[16][1] * in[16 * in_stride];
    EEE[0] = EEEE[0] + EEEO[0];
    EEE[3] = EEEE[0] - EEEO[0];
    EEE[1] = EEEE[1] + EEEO[1];
    EEE[2] = EEEE[1] - EEEO[1];
    for (int k = 0; k < 4; k++) {
      EE[k] = EEE[k] + EEO[k];
      EE[k + 4] = EEE[3 - k] - EEO[3 - k];
    }
    for (int k = 0; k < 8; k++) {
      E[k] = EE[k] + EO[k];
      E[k + 8] = EE[7 - k] - EO[7 - k];
    }
    for (int k = 0; k < 16; k++) {
      out[k] = util::Clip3((E[k] + O[k] + add) >> shift,
                           constants::kInt16Min, constants::kInt16Max);
      out[k + 16] = util::Clip3((E[15 - k] - O[15 - k] + add) >> shift,
                                constants::kInt16Min, constants::kInt16Max);
    }
    in++;
    out += out_stride;
  }
  for (int y = tx_lines; y < lines; y++) {
    memset(out, 0, sizeof(Coeff) * 32);
    out += out_stride;
  }
}

void
InverseTransform::InvDct2Transform64(int shift, int lines,
                                     bool high_prec, bool zero_out,
                                     const Coeff *in, ptrdiff_t in_stride,
                                     Coeff *out, ptrdiff_t out_stride) {
  // Only high precision matrices supported
  shift += !high_prec ? kTransformHighPrecisionShift : 0;
  const auto &kMatrix = kDct2Transform64High;
  const int add = 1 << (shift - 1);
  const int tx_lines = zero_out ?
    std::min(lines, constants::kTransformZeroOutMinSize) : lines;
  const int in_rows = std::min(64, constants::kTransformZeroOutMinSize);
  int E[32], O[32];
  int EO[16], EE[16];
  int EEO[8], EEE[8];
  int EEEO[4], EEEE[4];
  int EEEEO[2], EEEEE[2];

  for (int y = 0; y < tx_lines; y++) {
    for (int k = 0; k < 32; k++) {
      O[k] =
        kMatrix[1][k] * in[1 * in_stride] +
        kMatrix[3][k] * in[3 * in_stride] +
        kMatrix[5][k] * in[5 * in_stride] +
        kMatrix[7][k] * in[7 * in_stride] +
        kMatrix[9][k] * in[9 * in_stride] +
        kMatrix[11][k] * in[11 * in_stride] +
        kMatrix[13][k] * in[13 * in_stride] +
        kMatrix[15][k] * in[15 * in_stride] +
        kMatrix[17][k] * in[17 * in_stride] +
        kMatrix[19][k] * in[19 * in_stride] +
        kMatrix[21][k] * in[21 * in_stride] +
        kMatrix[23][k] * in[23 * in_stride] +
        kMatrix[25][k] * in[25 * in_stride] +
        kMatrix[27][k] * in[27 * in_stride] +
        kMatrix[29][k] * in[29 * in_stride] +
        kMatrix[31][k] * in[31 * in_stride];
      if (in_rows > 32) {
        O[k] +=
          kMatrix[33][k] * in[33 * in_stride] +
          kMatrix[35][k] * in[35 * in_stride] +
          kMatrix[37][k] * in[37 * in_stride] +
          kMatrix[39][k] * in[39 * in_stride] +
          kMatrix[41][k] * in[41 * in_stride] +
          kMatrix[43][k] * in[43 * in_stride] +
          kMatrix[45][k] * in[45 * in_stride] +
          kMatrix[47][k] * in[47 * in_stride] +
          kMatrix[49][k] * in[49 * in_stride] +
          kMatrix[51][k] * in[51 * in_stride] +
          kMatrix[53][k] * in[53 * in_stride] +
          kMatrix[55][k] * in[55 * in_stride] +
          kMatrix[57][k] * in[57 * in_stride] +
          kMatrix[59][k] * in[59 * in_stride] +
          kMatrix[61][k] * in[61 * in_stride] +
          kMatrix[63][k] * in[63 * in_stride];
      }
    }
    for (int k = 0; k < 16; k++) {
      EO[k] =
        kMatrix[2][k] * in[2 * in_stride] +
        kMatrix[6][k] * in[6 * in_stride] +
        kMatrix[10][k] * in[10 * in_stride] +
        kMatrix[14][k] * in[14 * in_stride] +
        kMatrix[18][k] * in[18 * in_stride] +
        kMatrix[22][k] * in[22 * in_stride] +
        kMatrix[26][k] * in[26 * in_stride] +
        kMatrix[30][k] * in[30 * in_stride];
      if (in_rows > 32) {
        EO[k] += kMatrix[34][k] * in[34 * in_stride] +
          kMatrix[38][k] * in[38 * in_stride] +
          kMatrix[42][k] * in[42 * in_stride] +
          kMatrix[46][k] * in[46 * in_stride] +
          kMatrix[50][k] * in[50 * in_stride] +
          kMatrix[54][k] * in[54 * in_stride] +
          kMatrix[58][k] * in[58 * in_stride] +
          kMatrix[62][k] * in[62 * in_stride];
      }
    }
    for (int k = 0; k < 8; k++) {
      EEO[k] =
        kMatrix[4][k] * in[4 * in_stride] +
        kMatrix[12][k] * in[12 * in_stride] +
        kMatrix[20][k] * in[20 * in_stride] +
        kMatrix[28][k] * in[28 * in_stride];
      if (in_rows > 32) {
        EEO[k] += kMatrix[36][k] * in[36 * in_stride] +
          kMatrix[44][k] * in[44 * in_stride] +
          kMatrix[52][k] * in[52 * in_stride] +
          kMatrix[60][k] * in[60 * in_stride];
      }
    }
    for (int k = 0; k < 4; k++) {
      EEEO[k] =
        kMatrix[8][k] * in[8 * in_stride] +
        kMatrix[24][k] * in[24 * in_stride];
      if (in_rows > 32) {
        EEEO[k] += kMatrix[40][k] * in[40 * in_stride] +
          kMatrix[56][k] * in[56 * in_stride];
      }
    }
    EEEEO[0] = kMatrix[16][0] * in[16 * in_stride];
    if (in_rows > 32) {
      EEEEO[0] += kMatrix[48][0] * in[48 * in_stride];
    }
    EEEEO[1] = kMatrix[16][1] * in[16 * in_stride];
    if (in_rows > 32) {
      EEEEO[1] += kMatrix[48][1] * in[48 * in_stride];
    }
    EEEEE[0] = kMatrix[0][0] * in[0];
    if (in_rows > 32) {
      EEEEE[0] += kMatrix[32][0] * in[32 * in_stride];
    }
    EEEEE[1] = kMatrix[0][1] * in[0];
    if (in_rows > 32) {
      EEEEE[1] += kMatrix[32][1] * in[32 * in_stride];
    }
    EEEE[0] = EEEEE[0] + EEEEO[0];
    EEEE[1] = EEEEE[1] + EEEEO[1];
    EEEE[2] = EEEEE[1] - EEEEO[1];
    EEEE[3] = EEEEE[0] - EEEEO[0];
    for (int k = 0; k < 4; k++) {
      EEE[k] = EEEE[k] + EEEO[k];
      EEE[k + 4] = EEEE[3 - k] - EEEO[3 - k];
    }
    for (int k = 0; k < 8; k++) {
      EE[k] = EEE[k] + EEO[k];
      EE[k + 8] = EEE[7 - k] - EEO[7 - k];
    }
    for (int k = 0; k < 16; k++) {
      E[k] = EE[k] + EO[k];
      E[k + 16] = EE[15 - k] - EO[15 - k];
    }
    for (int k = 0; k < 32; k++) {
      out[k] = util::Clip3((E[k] + O[k] + add) >> shift,
                           constants::kInt16Min, constants::kInt16Max);
      out[k + 32] = util::Clip3((E[31 - k] - O[31 - k] + add) >> shift,
                                constants::kInt16Min, constants::kInt16Max);
    }
    in++;
    out += out_stride;
  }
  for (int y = tx_lines; y < lines; y++) {
    memset(out, 0, sizeof(Coeff) * 64);
    out += out_stride;
  }
}

template<int N>
void
InverseTransform::InvGenericTransformN(int shift, int lines, bool zero_out,
                                       const Coeff(&kMatrix)[N * N],
                                       const Coeff *in, ptrdiff_t in_stride,
                                       Coeff *out, ptrdiff_t out_stride) {
  const int add = 1 << (shift - 1);
  const int tx_lines = zero_out ?
    std::min(lines, constants::kTransformZeroOutMinSize) : lines;
  const int in_rows = std::min(N, constants::kTransformZeroOutMinSize);

  for (int y = 0; y < tx_lines; y++) {
    for (int x = 0; x < N; x++) {
      int sum = 0;
      for (int k = 0; k < in_rows; k++) {
        sum += kMatrix[k * N + x] * in[k * in_stride];
      }
      out[x] = util::Clip3((sum + add) >> shift,
                           constants::kInt16Min, constants::kInt16Max);
    }
    in++;
    out += out_stride;
  }
  for (int y = tx_lines; y < lines; y++) {
    memset(out, 0, sizeof(Coeff) * N);
    out += out_stride;
  }
}

void ForwardTransform::Transform(const CodingUnit &cu, YuvComponent comp,
                                 const ResidualBuffer &in_buffer,
                                 CoeffBuffer *out_buffer) {
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  const bool can_dst_4x4 = util::IsLuma(comp) && cu.IsIntra();
  // TODO(PH) Should remove legacy behavior and only look at restriction flag
  const bool default_high_precision =
    !Restrictions::Get().disable_ext_transform_high_precision;
  const bool high_prec1 = default_high_precision || width >= 64 || width == 2;
  const bool high_prec2 = default_high_precision || height >= 64 || height == 2;
  const int shift1 = util::SizeToLog2(width) + bitdepth_ - 9 +
    (high_prec1 ? kTransformHighPrecisionShift : 0);
  const int shift2 = util::SizeToLog2(height) + 6 +
    (high_prec2 ? kTransformHighPrecisionShift : 0);
  const Residual *resi_ptr = in_buffer.GetDataPtr();
  ptrdiff_t resi_stride = in_buffer.GetStride();
  Coeff *coeff_ptr = out_buffer->GetDataPtr();
  const ptrdiff_t coeff_stride = out_buffer->GetStride();
  Coeff *temp_ptr = &coeff_temp_[0];
  const ptrdiff_t temp_stride = kBufferStride_;

  switch (cu.GetTransformType(comp, 1)) {
    case TransformType::kDefault:
      if (can_dst_4x4 && width == 4 && height == 4) {
        FwdPartialDst4(shift1, high_prec1, resi_ptr, resi_stride,
                       temp_ptr, temp_stride);
      } else {
        FwdDct2(width, shift1, height, high_prec1, false,
                resi_ptr, resi_stride, temp_ptr, temp_stride);
      }
      break;
    case TransformType::kDCT2:
      FwdDct2(width, shift1, height, high_prec1, false,
              resi_ptr, resi_stride, temp_ptr, temp_stride);
      break;
    case TransformType::kDCT5:
      FwdDct5(width, shift1, height, high_prec1, false,
              resi_ptr, resi_stride, temp_ptr, temp_stride);
      break;
    case TransformType::kDCT8:
      FwdDct8(width, shift1, height, high_prec1, false,
              resi_ptr, resi_stride, temp_ptr, temp_stride);
      break;
    case TransformType::kDST1:
      FwdDst1(width, shift1, height, high_prec1, false,
              resi_ptr, resi_stride, temp_ptr, temp_stride);
      break;
    case TransformType::kDST7:
      FwdDst7(width, shift1, height, high_prec1, false,
              resi_ptr, resi_stride, temp_ptr, temp_stride);
      break;
    default:
      assert(0);
      break;
  }

  switch (cu.GetTransformType(comp, 0)) {
    case TransformType::kDefault:
      if (can_dst_4x4 && width == 4 && height == 4) {
        FwdPartialDst4(shift2, high_prec2, temp_ptr, temp_stride,
                       coeff_ptr, coeff_stride);
      } else {
        FwdDct2(height, shift2, width, high_prec2, true,
                temp_ptr, temp_stride, coeff_ptr, coeff_stride);
      }
      break;
    case TransformType::kDCT2:
      FwdDct2(height, shift2, width, high_prec2, true,
              temp_ptr, temp_stride, coeff_ptr, coeff_stride);
      break;
    case TransformType::kDCT5:
      FwdDct5(height, shift2, width, high_prec2, true,
              temp_ptr, temp_stride, coeff_ptr, coeff_stride);
      break;
    case TransformType::kDCT8:
      FwdDct8(height, shift2, width, high_prec2, true,
              temp_ptr, temp_stride, coeff_ptr, coeff_stride);
      break;
    case TransformType::kDST1:
      FwdDst1(height, shift2, width, high_prec2, true,
              temp_ptr, temp_stride, coeff_ptr, coeff_stride);
      break;
    case TransformType::kDST7:
      FwdDst7(height, shift2, width, high_prec2, true,
              temp_ptr, temp_stride, coeff_ptr, coeff_stride);
      break;
    default:
      assert(0);
      break;
  }
}

void
ForwardTransform::TransformSkip(int width, int height,
                                const ResidualBuffer &in_buffer,
                                CoeffBuffer *out_buffer) {
  const Residual *resi = in_buffer.GetDataPtr();
  ptrdiff_t resi_stride = in_buffer.GetStride();
  Coeff *coeff = out_buffer->GetDataPtr();
  const ptrdiff_t coeff_stride = out_buffer->GetStride();

  // TODO(PH) Duplicate code that should be extracted to common function
  const bool size_rounding_bias =
    (util::SizeToLog2(width) + util::SizeToLog2(height)) % 2 != 0;
  const int transform_shift =
    Quantize::GetTransformShift(width, height, bitdepth_);
  const int shift = transform_shift + (size_rounding_bias ? -8 : 0);
  const int scale = size_rounding_bias ? 181 : 1;
  if (shift > 0) {
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        coeff[y * coeff_stride + x] =
          (resi[y * resi_stride + x] * scale) * (1 << shift);
      }
    }
  } else {
    const int offset = 1 << (-shift - 1);
    for (int y = 0; y < height; y++) {
      for (int x = 0; x < width; x++) {
        coeff[y * coeff_stride + x] =
          (resi[y * resi_stride + x] * scale + offset) >> -shift;
      }
    }
  }
}

void ForwardTransform::FwdPartialDst4(int shift, bool high_prec,
                                      const Coeff *in, ptrdiff_t in_stride,
                                      Coeff *out, ptrdiff_t out_stride) {
  // No support for high precision for DST 4x4
  shift -= high_prec ? kTransformHighPrecisionShift : 0;
  const int add = 1 << (shift - 1);

  for (int i = 0; i < 4; i++) {
    int c[4];
    c[0] = in[0] + in[3];
    c[1] = in[1] + in[3];
    c[2] = in[0] - in[1];
    c[3] = 74 * in[2];
    out[0 * out_stride] = (29 * c[0] + 55 * c[1] + c[3] + add) >> shift;
    out[1 * out_stride] = (74 * (in[0] + in[1] - in[3]) + add) >> shift;
    out[2 * out_stride] = (29 * c[2] + 55 * c[0] - c[3] + add) >> shift;
    out[3 * out_stride] = (55 * c[2] - 29 * c[1] + c[3] + add) >> shift;
    in += in_stride;
    out++;
  }
}

void ForwardTransform::FwdDct2(int size, int shift, int lines,
                               bool high_prec, bool zero_out,
                               const Coeff *in, ptrdiff_t in_stride,
                               Coeff *out, ptrdiff_t out_stride) {
  switch (size) {
    case 2:
      FwdDct2Transform2(shift, lines, high_prec, zero_out,
                        in, in_stride, out, out_stride);
      break;
    case 4:
      FwdDct2Transform4(shift, lines, high_prec, zero_out,
                        in, in_stride, out, out_stride);
      break;
    case 8:
      FwdDct2Transform8(shift, lines, high_prec, zero_out,
                        in, in_stride, out, out_stride);
      break;
    case 16:
      FwdDct2Transform16(shift, lines, high_prec, zero_out,
                         in, in_stride, out, out_stride);
      break;
    case 32:
      FwdDct2Transform32(shift, lines, high_prec, zero_out,
                         in, in_stride, out, out_stride);
      break;
    case 64:
      FwdDct2Transform64(shift, lines, high_prec, zero_out,
                         in, in_stride, out, out_stride);
      break;
    default:
      assert(0);
      break;
  }
}

void ForwardTransform::FwdDct5(int size, int shift, int lines,
                               bool high_prec, bool zero_out,
                               const Coeff *in, ptrdiff_t in_stride,
                               Coeff *out, ptrdiff_t out_stride) {
  // Only high precision matrices supported
  shift += !high_prec ? kTransformHighPrecisionShift : 0;
  switch (size) {
    case 4:
      FwdGenericTransformN<4>(shift, lines, zero_out, kDct5Transform4High,
                              in, in_stride, out, out_stride);
      break;
    case 8:
      FwdGenericTransformN<8>(shift, lines, zero_out, kDct5Transform8High,
                              in, in_stride, out, out_stride);
      break;
    case 16:
      FwdGenericTransformN<16>(shift, lines, zero_out, kDct5Transform16High,
                               in, in_stride, out, out_stride);
      break;
    case 32:
      FwdGenericTransformN<32>(shift, lines, zero_out, kDct5Transform32High,
                               in, in_stride, out, out_stride);
      break;
    case 64:
      FwdGenericTransformN<64>(shift, lines, zero_out, kDct5Transform64High,
                               in, in_stride, out, out_stride);
      break;
    default:
      assert(0);
      break;
  }
}

void ForwardTransform::FwdDct8(int size, int shift, int lines,
                               bool high_prec, bool zero_out,
                               const Coeff *in, ptrdiff_t in_stride,
                               Coeff *out, ptrdiff_t out_stride) {
  // Only high precision matrices supported
  shift += !high_prec ? kTransformHighPrecisionShift : 0;
  switch (size) {
    case 4:
      FwdGenericTransformN<4>(shift, lines, zero_out, kDct8Transform4High,
                              in, in_stride, out, out_stride);
      break;
    case 8:
      FwdGenericTransformN<8>(shift, lines, zero_out, kDct8Transform8High,
                              in, in_stride, out, out_stride);
      break;
    case 16:
      FwdGenericTransformN<16>(shift, lines, zero_out, kDct8Transform16High,
                               in, in_stride, out, out_stride);
      break;
    case 32:
      FwdGenericTransformN<32>(shift, lines, zero_out, kDct8Transform32High,
                               in, in_stride, out, out_stride);
      break;
    case 64:
      FwdGenericTransformN<64>(shift, lines, zero_out, kDct8Transform64High,
                               in, in_stride, out, out_stride);
      break;
    default:
      assert(0);
      break;
  }
}

void ForwardTransform::FwdDst1(int size, int shift, int lines,
                               bool high_prec, bool zero_out,
                               const Coeff *in, ptrdiff_t in_stride,
                               Coeff *out, ptrdiff_t out_stride) {
  // Only high precision matrices supported
  shift += !high_prec ? kTransformHighPrecisionShift : 0;
  switch (size) {
    case 4:
      FwdGenericTransformN<4>(shift, lines, zero_out, kDst1Transform4High,
                              in, in_stride, out, out_stride);
      break;
    case 8:
      FwdGenericTransformN<8>(shift, lines, zero_out, kDst1Transform8High,
                              in, in_stride, out, out_stride);
      break;
    case 16:
      FwdGenericTransformN<16>(shift, lines, zero_out, kDst1Transform16High,
                               in, in_stride, out, out_stride);
      break;
    case 32:
      FwdGenericTransformN<32>(shift, lines, zero_out, kDst1Transform32High,
                               in, in_stride, out, out_stride);
      break;
    case 64:
      FwdGenericTransformN<64>(shift, lines, zero_out, kDst1Transform64High,
                               in, in_stride, out, out_stride);
      break;
    default:
      assert(0);
      break;
  }
}

void ForwardTransform::FwdDst7(int size, int shift, int lines,
                               bool high_prec, bool zero_out,
                               const Coeff *in, ptrdiff_t in_stride,
                               Coeff *out, ptrdiff_t out_stride) {
  // Only high precision matrices supported
  shift += !high_prec ? kTransformHighPrecisionShift : 0;
  switch (size) {
    case 4:
      FwdGenericTransformN<4>(shift, lines, zero_out, kDst7Transform4High,
                              in, in_stride, out, out_stride);
      break;
    case 8:
      FwdGenericTransformN<8>(shift, lines, zero_out, kDst7Transform8High,
                              in, in_stride, out, out_stride);
      break;
    case 16:
      FwdGenericTransformN<16>(shift, lines, zero_out, kDst7Transform16High,
                               in, in_stride, out, out_stride);
      break;
    case 32:
      FwdGenericTransformN<32>(shift, lines, zero_out, kDst7Transform32High,
                               in, in_stride, out, out_stride);
      break;
    case 64:
      FwdGenericTransformN<64>(shift, lines, zero_out, kDst7Transform64High,
                               in, in_stride, out, out_stride);
      break;
    default:
      assert(0);
      break;
  }
}

void
ForwardTransform::FwdDct2Transform2(int shift, int lines,
                                    bool high_prec, bool zero_out,
                                    const Coeff *in, ptrdiff_t in_stride,
                                    Coeff *out, ptrdiff_t out_stride) {
  // Only high precision matrices supported
  shift += !high_prec ? kTransformHighPrecisionShift : 0;
  const auto &kMatrix = kDct2Transform2High;
  const int add = 1 << (shift - 1);
  const int tx_lines = zero_out ?
    std::min(lines, constants::kTransformZeroOutMinSize) : lines;
  int E[1], O[1];

  for (int y = 0; y < tx_lines; y++) {
    E[0] = in[0] + in[1];
    O[0] = in[0] - in[1];
    out[0 * out_stride] = (kMatrix[0][0] * E[0] + add) >> shift;
    out[1 * out_stride] = (kMatrix[1][0] * O[0] + add) >> shift;
    in += in_stride;
    out++;
  }
  if (tx_lines < lines) {
    for (int y = 0; y < 2; y++) {
      memset(out + y * out_stride, 0, sizeof(Coeff)*(lines - tx_lines));
    }
  }
}

void
ForwardTransform::FwdDct2Transform4(int shift, int lines,
                                    bool high_prec, bool zero_out,
                                    const Coeff *in, ptrdiff_t in_stride,
                                    Coeff *out, ptrdiff_t out_stride) {
  const auto &kMatrix = high_prec ? kDct2Transform4High : kDct2Transform4;
  const int add = 1 << (shift - 1);
  const int tx_lines = zero_out ?
    std::min(lines, constants::kTransformZeroOutMinSize) : lines;
  int E[2], O[2];

  for (int y = 0; y < tx_lines; y++) {
    E[0] = in[0] + in[3];
    E[1] = in[1] + in[2];
    O[0] = in[0] - in[3];
    O[1] = in[1] - in[2];
    out[0 * out_stride] = (kMatrix[0][0] * E[0] +
                           kMatrix[0][1] * E[1] + add) >> shift;
    out[2 * out_stride] = (kMatrix[2][0] * E[0] +
                           kMatrix[2][1] * E[1] + add) >> shift;
    out[1 * out_stride] = (kMatrix[1][0] * O[0] +
                           kMatrix[1][1] * O[1] + add) >> shift;
    out[3 * out_stride] = (kMatrix[3][0] * O[0] +
                           kMatrix[3][1] * O[1] + add) >> shift;
    in += in_stride;
    out++;
  }
  if (tx_lines < lines) {
    for (int y = 0; y < 4; y++) {
      memset(out + y * out_stride, 0, sizeof(Coeff)*(lines - tx_lines));
    }
  }
}

void
ForwardTransform::FwdDct2Transform8(int shift, int lines,
                                    bool high_prec, bool zero_out,
                                    const Coeff *in, ptrdiff_t in_stride,
                                    Coeff *out, ptrdiff_t out_stride) {
  const auto &kMatrix = high_prec ? kDct2Transform8High : kDct2Transform8;
  const int add = 1 << (shift - 1);
  const int tx_lines = zero_out ?
    std::min(lines, constants::kTransformZeroOutMinSize) : lines;
  int E[4], O[4];
  int EE[2], EO[2];

  for (int y = 0; y < tx_lines; y++) {
    for (int x = 0; x < 4; x++) {
      E[x] = in[x] + in[7 - x];
      O[x] = in[x] - in[7 - x];
    }
    EE[0] = E[0] + E[3];
    EO[0] = E[0] - E[3];
    EE[1] = E[1] + E[2];
    EO[1] = E[1] - E[2];

    out[0 * out_stride] = (kMatrix[0][0] * EE[0] +
                           kMatrix[0][1] * EE[1] + add) >> shift;
    out[4 * out_stride] = (kMatrix[4][0] * EE[0] +
                           kMatrix[4][1] * EE[1] + add) >> shift;
    out[2 * out_stride] = (kMatrix[2][0] * EO[0] +
                           kMatrix[2][1] * EO[1] + add) >> shift;
    out[6 * out_stride] = (kMatrix[6][0] * EO[0] +
                           kMatrix[6][1] * EO[1] + add) >> shift;
    out[1 * out_stride] = (kMatrix[1][0] * O[0] +
                           kMatrix[1][1] * O[1] +
                           kMatrix[1][2] * O[2] +
                           kMatrix[1][3] * O[3] + add) >> shift;
    out[3 * out_stride] = (kMatrix[3][0] * O[0] +
                           kMatrix[3][1] * O[1] +
                           kMatrix[3][2] * O[2] +
                           kMatrix[3][3] * O[3] + add) >> shift;
    out[5 * out_stride] = (kMatrix[5][0] * O[0] +
                           kMatrix[5][1] * O[1] +
                           kMatrix[5][2] * O[2] +
                           kMatrix[5][3] * O[3] + add) >> shift;
    out[7 * out_stride] = (kMatrix[7][0] * O[0] +
                           kMatrix[7][1] * O[1] +
                           kMatrix[7][2] * O[2] +
                           kMatrix[7][3] * O[3] + add) >> shift;
    in += in_stride;
    out++;
  }
  if (tx_lines < lines) {
    for (int y = 0; y < 8; y++) {
      memset(out + y * out_stride, 0, sizeof(Coeff)*(lines - tx_lines));
    }
  }
}

void
ForwardTransform::FwdDct2Transform16(int shift, int lines,
                                     bool high_prec, bool zero_out,
                                     const Coeff *in, ptrdiff_t in_stride,
                                     Coeff *out, ptrdiff_t out_stride) {
  const auto &kMatrix = high_prec ? kDct2Transform16High : kDct2Transform16;
  const int add = 1 << (shift - 1);
  const int tx_lines = zero_out ?
    std::min(lines, constants::kTransformZeroOutMinSize) : lines;
  int E[8], O[8];
  int EE[4], EO[4];
  int EEE[2], EEO[2];

  for (int y = 0; y < tx_lines; y++) {
    for (int k = 0; k < 8; k++) {
      E[k] = in[k] + in[15 - k];
      O[k] = in[k] - in[15 - k];
    }
    for (int k = 0; k < 4; k++) {
      EE[k] = E[k] + E[7 - k];
      EO[k] = E[k] - E[7 - k];
    }
    EEE[0] = EE[0] + EE[3];
    EEO[0] = EE[0] - EE[3];
    EEE[1] = EE[1] + EE[2];
    EEO[1] = EE[1] - EE[2];
    out[0 * out_stride] = (kMatrix[0][0] * EEE[0] +
                           kMatrix[0][1] * EEE[1] + add) >> shift;
    out[8 * out_stride] = (kMatrix[8][0] * EEE[0] +
                           kMatrix[8][1] * EEE[1] + add) >> shift;
    out[4 * out_stride] = (kMatrix[4][0] * EEO[0] +
                           kMatrix[4][1] * EEO[1] + add) >> shift;
    out[12 * out_stride] = (kMatrix[12][0] * EEO[0] +
                            kMatrix[12][1] * EEO[1] + add) >> shift;
    for (int k = 2; k < 16; k += 4) {
      out[k*out_stride] = (kMatrix[k][0] * EO[0] +
                           kMatrix[k][1] * EO[1] +
                           kMatrix[k][2] * EO[2] +
                           kMatrix[k][3] * EO[3] + add) >> shift;
    }
    for (int k = 1; k < 16; k += 2) {
      out[k*out_stride] = (kMatrix[k][0] * O[0] +
                           kMatrix[k][1] * O[1] +
                           kMatrix[k][2] * O[2] +
                           kMatrix[k][3] * O[3] +
                           kMatrix[k][4] * O[4] +
                           kMatrix[k][5] * O[5] +
                           kMatrix[k][6] * O[6] +
                           kMatrix[k][7] * O[7] + add) >> shift;
    }
    in += in_stride;
    out++;
  }
  if (tx_lines < lines) {
    for (int y = 0; y < 16; y++) {
      memset(out + y * out_stride, 0, sizeof(Coeff)*(lines - tx_lines));
    }
  }
}

void
ForwardTransform::FwdDct2Transform32(int shift, int lines,
                                     bool high_prec, bool zero_out,
                                     const Coeff *in, ptrdiff_t in_stride,
                                     Coeff *out, ptrdiff_t out_stride) {
  const auto &kMatrix = high_prec ? kDct2Transform32High : kDct2Transform32;
  const int add = 1 << (shift - 1);
  const int tx_lines = zero_out ?
    std::min(lines, constants::kTransformZeroOutMinSize) : lines;
  int E[16], O[16];
  int EE[8], EO[8];
  int EEE[4], EEO[4];
  int EEEE[2], EEEO[2];

  for (int y = 0; y < tx_lines; y++) {
    for (int k = 0; k < 16; k++) {
      E[k] = in[k] + in[31 - k];
      O[k] = in[k] - in[31 - k];
    }
    for (int k = 0; k < 8; k++) {
      EE[k] = E[k] + E[15 - k];
      EO[k] = E[k] - E[15 - k];
    }
    for (int k = 0; k < 4; k++) {
      EEE[k] = EE[k] + EE[7 - k];
      EEO[k] = EE[k] - EE[7 - k];
    }
    EEEE[0] = EEE[0] + EEE[3];
    EEEO[0] = EEE[0] - EEE[3];
    EEEE[1] = EEE[1] + EEE[2];
    EEEO[1] = EEE[1] - EEE[2];
    out[0 * out_stride] = (kMatrix[0][0] * EEEE[0] +
                           kMatrix[0][1] * EEEE[1] + add) >> shift;
    out[16 * out_stride] = (kMatrix[16][0] * EEEE[0] +
                            kMatrix[16][1] * EEEE[1] + add) >> shift;
    out[8 * out_stride] = (kMatrix[8][0] * EEEO[0] +
                           kMatrix[8][1] * EEEO[1] + add) >> shift;
    out[24 * out_stride] = (kMatrix[24][0] * EEEO[0] +
                            kMatrix[24][1] * EEEO[1] + add) >> shift;
    for (int k = 4; k < 32; k += 8) {
      out[k*out_stride] = (kMatrix[k][0] * EEO[0] +
                           kMatrix[k][1] * EEO[1] +
                           kMatrix[k][2] * EEO[2] +
                           kMatrix[k][3] * EEO[3] + add) >> shift;
    }
    for (int k = 2; k < 32; k += 4) {
      out[k*out_stride] = (kMatrix[k][0] * EO[0] +
                           kMatrix[k][1] * EO[1] +
                           kMatrix[k][2] * EO[2] +
                           kMatrix[k][3] * EO[3] +
                           kMatrix[k][4] * EO[4] +
                           kMatrix[k][5] * EO[5] +
                           kMatrix[k][6] * EO[6] +
                           kMatrix[k][7] * EO[7] + add) >> shift;
    }
    for (int k = 1; k < 32; k += 2) {
      out[k*out_stride] = (kMatrix[k][0] * O[0] +
                           kMatrix[k][1] * O[1] +
                           kMatrix[k][2] * O[2] +
                           kMatrix[k][3] * O[3] +
                           kMatrix[k][4] * O[4] +
                           kMatrix[k][5] * O[5] +
                           kMatrix[k][6] * O[6] +
                           kMatrix[k][7] * O[7] +
                           kMatrix[k][8] * O[8] +
                           kMatrix[k][9] * O[9] +
                           kMatrix[k][10] * O[10] +
                           kMatrix[k][11] * O[11] +
                           kMatrix[k][12] * O[12] +
                           kMatrix[k][13] * O[13] +
                           kMatrix[k][14] * O[14] +
                           kMatrix[k][15] * O[15] + add) >> shift;
    }
    in += in_stride;
    out++;
  }
  if (tx_lines < lines) {
    for (int y = 0; y < 32; y++) {
      memset(out + y * out_stride, 0, sizeof(Coeff)*(lines - tx_lines));
    }
  }
}

void
ForwardTransform::FwdDct2Transform64(int shift, int lines,
                                     bool high_prec, bool zero_out,
                                     const Coeff *in, ptrdiff_t in_stride,
                                     Coeff *out, ptrdiff_t out_stride) {
  // Only high precision matrices supported
  shift += !high_prec ? kTransformHighPrecisionShift : 0;
  const auto &kMatrix = kDct2Transform64High;
  const int add = 1 << (shift - 1);
  const int tx_lines = zero_out ?
    std::min(lines, constants::kTransformZeroOutMinSize) : lines;
  const int out_rows = std::min(64, constants::kTransformZeroOutMinSize);
  int E[32], O[32];
  int EE[16], EO[16];
  int EEE[8], EEO[8];
  int EEEE[4], EEEO[4];
  int EEEEE[2], EEEEO[2];
  Coeff *orig_out = out;

  for (int y = 0; y < tx_lines; y++) {
    for (int k = 0; k < 32; k++) {
      E[k] = in[k] + in[63 - k];
      O[k] = in[k] - in[63 - k];
    }
    for (int k = 0; k < 16; k++) {
      EE[k] = E[k] + E[31 - k];
      EO[k] = E[k] - E[31 - k];
    }
    for (int k = 0; k < 8; k++) {
      EEE[k] = EE[k] + EE[15 - k];
      EEO[k] = EE[k] - EE[15 - k];
    }
    for (int k = 0; k < 4; k++) {
      EEEE[k] = EEE[k] + EEE[7 - k];
      EEEO[k] = EEE[k] - EEE[7 - k];
    }
    EEEEE[0] = EEEE[0] + EEEE[3];
    EEEEO[0] = EEEE[0] - EEEE[3];
    EEEEE[1] = EEEE[1] + EEEE[2];
    EEEEO[1] = EEEE[1] - EEEE[2];
    out[0 * out_stride] = (kMatrix[0][0] * EEEEE[0] +
                           kMatrix[0][1] * EEEEE[1] + add) >> shift;
    out[16 * out_stride] = (kMatrix[16][0] * EEEEO[0] +
                            kMatrix[16][1] * EEEEO[1] + add) >> shift;
    if (out_rows > 32) {
      out[32 * out_stride] = (kMatrix[32][0] * EEEEE[0] +
                              kMatrix[32][1] * EEEEE[1] + add) >> shift;
      out[48 * out_stride] = (kMatrix[48][0] * EEEEO[0] +
                              kMatrix[48][1] * EEEEO[1] + add) >> shift;
    }
    for (int k = 8; k < out_rows; k += 16) {
      out[k*out_stride] = (kMatrix[k][0] * EEEO[0] +
                           kMatrix[k][1] * EEEO[1] +
                           kMatrix[k][2] * EEEO[2] +
                           kMatrix[k][3] * EEEO[3] + add) >> shift;
    }
    for (int k = 4; k < out_rows; k += 8) {
      out[k*out_stride] = (kMatrix[k][0] * EEO[0] +
                           kMatrix[k][1] * EEO[1] +
                           kMatrix[k][2] * EEO[2] +
                           kMatrix[k][3] * EEO[3] +
                           kMatrix[k][4] * EEO[4] +
                           kMatrix[k][5] * EEO[5] +
                           kMatrix[k][6] * EEO[6] +
                           kMatrix[k][7] * EEO[7] + add) >> shift;
    }
    for (int k = 2; k < out_rows; k += 4) {
      out[k*out_stride] = (kMatrix[k][0] * EO[0] +
                           kMatrix[k][1] * EO[1] +
                           kMatrix[k][2] * EO[2] +
                           kMatrix[k][3] * EO[3] +
                           kMatrix[k][4] * EO[4] +
                           kMatrix[k][5] * EO[5] +
                           kMatrix[k][6] * EO[6] +
                           kMatrix[k][7] * EO[7] +
                           kMatrix[k][8] * EO[8] +
                           kMatrix[k][9] * EO[9] +
                           kMatrix[k][10] * EO[10] +
                           kMatrix[k][11] * EO[11] +
                           kMatrix[k][12] * EO[12] +
                           kMatrix[k][13] * EO[13] +
                           kMatrix[k][14] * EO[14] +
                           kMatrix[k][15] * EO[15] + add) >> shift;
    }
    for (int k = 1; k < out_rows; k += 2) {
      out[k*out_stride] = (kMatrix[k][0] * O[0] +
                           kMatrix[k][1] * O[1] +
                           kMatrix[k][2] * O[2] +
                           kMatrix[k][3] * O[3] +
                           kMatrix[k][4] * O[4] +
                           kMatrix[k][5] * O[5] +
                           kMatrix[k][6] * O[6] +
                           kMatrix[k][7] * O[7] +
                           kMatrix[k][8] * O[8] +
                           kMatrix[k][9] * O[9] +
                           kMatrix[k][10] * O[10] +
                           kMatrix[k][11] * O[11] +
                           kMatrix[k][12] * O[12] +
                           kMatrix[k][13] * O[13] +
                           kMatrix[k][14] * O[14] +
                           kMatrix[k][15] * O[15] +
                           kMatrix[k][16] * O[16] +
                           kMatrix[k][17] * O[17] +
                           kMatrix[k][18] * O[18] +
                           kMatrix[k][19] * O[19] +
                           kMatrix[k][20] * O[20] +
                           kMatrix[k][21] * O[21] +
                           kMatrix[k][22] * O[22] +
                           kMatrix[k][23] * O[23] +
                           kMatrix[k][24] * O[24] +
                           kMatrix[k][25] * O[25] +
                           kMatrix[k][26] * O[26] +
                           kMatrix[k][27] * O[27] +
                           kMatrix[k][28] * O[28] +
                           kMatrix[k][29] * O[29] +
                           kMatrix[k][30] * O[30] +
                           kMatrix[k][31] * O[31] + add) >> shift;
    }
    in += in_stride;
    out++;
  }
  if (tx_lines < lines) {
    for (int y = 0; y < out_rows; y++) {
      memset(out + y * out_stride, 0, sizeof(Coeff)*(lines - tx_lines));
    }
  }
  if (out_rows < 64) {
    for (int y = out_rows; y < 64; y++) {
      std::memset(orig_out + y * out_stride, 0, sizeof(Coeff) * lines);
    }
  }
}

template<int N>
void
ForwardTransform::FwdGenericTransformN(int shift, int lines, bool zero_out,
                                       const Coeff(&kMatrix)[N * N],
                                       const Coeff *in, ptrdiff_t in_stride,
                                       Coeff *out, ptrdiff_t out_stride) {
  const int add = 1 << (shift - 1);
  const int tx_lines = zero_out ?
    std::min(lines, constants::kTransformZeroOutMinSize) : lines;
  const int out_rows = std::min(N, constants::kTransformZeroOutMinSize);

  for (int y = 0; y < tx_lines; y++) {
    for (int x = 0; x < out_rows; x++) {
      int sum = 0;
      for (int k = 0; k < N; k++) {
        sum += kMatrix[x * N + k] * in[k];
      }
      out[x * out_stride + y] = (sum + add) >> shift;
    }
    in += in_stride;
  }
  if (tx_lines < lines) {
    for (int y = 0; y < out_rows; y++) {
      memset(out + y * out_stride + tx_lines, 0,
             sizeof(Coeff) * (lines - tx_lines));
    }
  }
  if (out_rows < N) {
    for (int y = out_rows; y < N; y++) {
      std::memset(out + y * out_stride, 0, sizeof(Coeff) * lines);
    }
  }
}

ScanOrder TransformHelper::DetermineScanOrder(const CodingUnit &cu,
                                              YuvComponent comp) {
  static const int kSizeThreshold = 16;
  const int angle_threshold =
    !Restrictions::Get().disable_ext_intra_extra_modes ? 10 : 5;
  if (cu.GetPredMode() != PredictionMode::kIntra ||
      Restrictions::Get().disable_transform_adaptive_scan_order) {
    return ScanOrder::kDiagonal;
  }
  if (cu.GetWidth(YuvComponent::kY) >= kSizeThreshold ||
      cu.GetHeight(YuvComponent::kY) >= kSizeThreshold) {
    return ScanOrder::kDiagonal;
  }
  int intra_mode = static_cast<int>(cu.GetIntraMode(comp));
  if (std::abs(intra_mode - IntraPrediction::Convert(IntraAngle::kVertical)) <
      angle_threshold) {
    return ScanOrder::kHorizontal;
  }
  if (std::abs(intra_mode - IntraPrediction::Convert(IntraAngle::kHorizontal)) <
      angle_threshold) {
    return ScanOrder::kVertical;
  }
  return ScanOrder::kDiagonal;
}

void TransformHelper::DeriveSubblockScan(ScanOrder scan_order, int width,
                                         int height, uint16_t *scan_table) {
  int nbr_subblocks = width * height;
  int pos_x = 0;
  int pos_y = 0;
  if (scan_order == ScanOrder::kDiagonal) {
    for (int i = 0; i < nbr_subblocks; i++) {
      scan_table[i] = static_cast<uint16_t>(pos_y * width + pos_x);
      if ((pos_x == width - 1) || (pos_y == 0)) {
        pos_y += pos_x + 1;
        pos_x = 0;
        if (pos_y >= height) {
          pos_x += pos_y - (height - 1);
          pos_y = height - 1;
        }
      } else {
        pos_x++;
        pos_y--;
      }
    }
  } else if (scan_order == ScanOrder::kHorizontal) {
    for (int i = 0; i < nbr_subblocks; i++) {
      scan_table[i] = static_cast<uint16_t>(pos_y * width + pos_x);
      if (pos_x == width - 1) {
        pos_x = 0;
        pos_y++;
      } else {
        pos_x++;
      }
    }
  } else if (scan_order == ScanOrder::kVertical) {
    for (int i = 0; i < nbr_subblocks; i++) {
      scan_table[i] = static_cast<uint16_t>(pos_y * width + pos_x);
      if (pos_y == height - 1) {
        pos_x++;
        pos_y = 0;
      } else {
        pos_y++;
      }
    }
  }
}

}   // namespace xvc
