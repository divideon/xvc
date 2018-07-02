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
