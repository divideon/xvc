/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
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

enum class PicturePredictionType {
  kIntra = 2,
  kUni = 1,
  kBi = 0,
  kInvalid = 99,
};

}   // namespace xvc

#endif  // XVC_COMMON_LIB_PICTURE_TYPES_H_
