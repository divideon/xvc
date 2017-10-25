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

#include "xvc_dec_lib/cu_decoder.h"

#include <cassert>

#include "xvc_common_lib/restrictions.h"
#include "xvc_common_lib/utils.h"

namespace xvc {

CuDecoder::CuDecoder(const SimdFunctions &simd, YuvPicture *decoded_pic,
                     PictureData *pic_data)
  : min_pel_(0),
  max_pel_((1 << decoded_pic->GetBitdepth()) - 1),
  decoded_pic_(*decoded_pic),
  pic_data_(*pic_data),
  inter_pred_(simd.inter_prediction, decoded_pic->GetBitdepth()),
  intra_pred_(decoded_pic->GetBitdepth()),
  inv_transform_(decoded_pic->GetBitdepth()),
  quantize_(),
  cu_reader_(pic_data, intra_pred_),
  temp_pred_(kBufferStride_, constants::kMaxBlockSize),
  temp_resi_(kBufferStride_, constants::kMaxBlockSize),
  temp_coeff_(kBufferStride_, constants::kMaxBlockSize) {
}

void CuDecoder::DecodeCtu(int rsaddr, SyntaxReader *reader) {
  ReadCtu(rsaddr, reader);

  CodingUnit *ctu = pic_data_.GetCtu(CuTree::Primary, rsaddr);
  pic_data_.ClearMarkCuInPic(ctu);
  DecompressCu(ctu);
  if (pic_data_.HasSecondaryCuTree()) {
    CodingUnit *ctu2 = pic_data_.GetCtu(CuTree::Secondary, rsaddr);
    pic_data_.ClearMarkCuInPic(ctu2);
    DecompressCu(ctu2);
  }
}

void CuDecoder::ReadCtu(int rsaddr, SyntaxReader * reader) {
  CodingUnit *ctu = pic_data_.GetCtu(CuTree::Primary, rsaddr);
  bool read_delta_qp = cu_reader_.ReadCtu(ctu, reader);
  if (pic_data_.HasSecondaryCuTree()) {
    CodingUnit *ctu2 = pic_data_.GetCtu(CuTree::Secondary, rsaddr);
    read_delta_qp |= cu_reader_.ReadCtu(ctu2, reader);
  }
  int qp = pic_data_.GetPicQp()->GetQpRaw(YuvComponent::kY);
  if (pic_data_.GetAdaptiveQp() && read_delta_qp) {
    qp = reader->ReadQp();
  }
  ctu->SetQp(qp);
  if (pic_data_.HasSecondaryCuTree()) {
    CodingUnit *ctu2 = pic_data_.GetCtu(CuTree::Secondary, rsaddr);
    ctu2->SetQp(qp);
  }
  if (Restrictions::Get().disable_ext_implicit_last_ctu) {
    if (reader->ReadEndOfSlice()) {
      assert(0);
    }
  }
}

void CuDecoder::DecompressCu(CodingUnit *cu) {
  if (cu->GetSplit() != SplitType::kNone) {
    for (CodingUnit *sub_cu : cu->GetSubCu()) {
      if (sub_cu) {
        sub_cu->SetQp(cu->GetQp(YuvComponent::kY));
        DecompressCu(sub_cu);
      }
    }
  } else {
    pic_data_.MarkUsedInPic(cu);
    for (YuvComponent comp : pic_data_.GetComponents(cu->GetCuTree())) {
      DecompressComponent(cu, comp, cu->GetQp());
    }
  }
}

void CuDecoder::DecompressComponent(CodingUnit *cu, YuvComponent comp,
                                    const Qp &qp) {
  int cu_x = cu->GetPosX(comp);
  int cu_y = cu->GetPosY(comp);
  int width = cu->GetWidth(comp);
  int height = cu->GetHeight(comp);
  bool cbf = cu->GetCbf(comp);
  SampleBuffer dec_buffer = decoded_pic_.GetSampleBuffer(comp, cu_x, cu_y);
  SampleBuffer &pred_buffer = cbf ? temp_pred_ : dec_buffer;

  // Predict
  if (cu->IsIntra()) {
    IntraMode intra_mode = cu->GetIntraMode(comp);
    intra_pred_.Predict(intra_mode, *cu, comp, decoded_pic_, &pred_buffer);
  } else {
    inter_pred_.CalculateMV(cu);
    inter_pred_.MotionCompensation(*cu, comp, pred_buffer.GetDataPtr(),
                                   pred_buffer.GetStride());
  }
  if (!cbf) {
    return;
  }

  // Dequant
  CoeffBuffer cu_coeff_buf = cu->GetCoeff(comp);
  quantize_.Inverse(comp, qp, width, height, decoded_pic_.GetBitdepth(),
                    cu_coeff_buf.GetDataPtr(), cu_coeff_buf.GetStride(),
                    temp_coeff_.GetDataPtr(), temp_coeff_.GetStride());

  // Inverse transform
  if (!cu->GetTransformSkip(comp)) {
    inv_transform_.Transform(*cu, comp, temp_coeff_, &temp_resi_);
  } else {
    inv_transform_.TransformSkip(width, height, temp_coeff_, &temp_resi_);
  }

  // Reconstruct
  dec_buffer.AddClip(width, height, temp_pred_, temp_resi_, min_pel_, max_pel_);
}

}   // namespace xvc
