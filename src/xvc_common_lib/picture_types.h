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

#ifndef XVC_COMMON_LIB_PICTURE_TYPES_H_
#define XVC_COMMON_LIB_PICTURE_TYPES_H_

namespace xvc {

enum class NalUnitType {
  kIntraPicture = 0,
  kIntraAccessPicture = 1,
  kPredictedPicture = 2,
  kPredictedAccessPicture = 3,
  kBipredictedPicture = 4,
  kBipredictedAccessPicture = 5,
  kReservedPictureType6 = 6,
  kReservedPictureType7 = 7,
  kReservedPictureType8 = 8,
  kReservedPictureType9 = 9,
  kReservedPictureType10 = 10,
  kSegmentHeader = 16,
  kSei = 17,
  kAccessUnitDelimiter = 18,
  kEndOfSegment = 19,
};

enum class OutputStatus {
  kReady,
  kProcessing,
  kPostProcessing,
  kFinishedProcessing,
  kHasNotBeenOutput,
  kHasBeenOutput,
};

enum class PicturePredictionType {
  kIntra = 2,
  kUni = 1,
  kBi = 0,
  kInvalid = 99,
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_PICTURE_TYPES_H_
