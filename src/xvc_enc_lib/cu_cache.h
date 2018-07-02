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

#ifndef XVC_ENC_LIB_CU_CACHE_H_
#define XVC_ENC_LIB_CU_CACHE_H_

#include <array>

#include "xvc_common_lib/picture_data.h"
#include "xvc_common_lib/coding_unit.h"

namespace xvc {

// Class for caching coding unit objects with the same location and size
// that are coded more than one time due to different split configurations
// When using binary split depth 3 the following split configurations will
// result in "identical" coding unit objects:
// hor, hor, ver (inverse: ver, ver, hor)
// hor, ver, hor (inverse: ver, hor, ver)
// ver, hor, ver (inverse, hor, ver, ver)
// quad, hor     (inverse, quad, ver)
//
// The following combinations results in cu objects that are only coded once,
// so therefore they are not cached:
// hor hor hor (inverse: ver ver ver)
class CuCache {
public:
  struct Result {
    const CodingUnit *cu;
    bool cacheable;
    bool any_intra;
    bool any_inter;
    bool any_skip;
  };
  explicit CuCache(PictureData *pic_data);
  ~CuCache();

  void Invalidate(CuTree cu_tree, int depth);
  Result Lookup(const CodingUnit &cu);
  bool Store(const CodingUnit &cu);

private:
  // number of cu objects to store per cache entry,
  // not needed if only using feature flags
  static const int kNumCuPerEntry = 0;
  // Number of cache partitions is related to number of binary splits
  // binary split depth = 0 => 0 (no cu objects are coded twice)
  // binary split depth = 1 => 1 (only full size, ver+hor and hor+ver)
  // binary split depth = 2 => 5 (full + binary split partion with depth 1)
  static const int kNumCachePartitions = 5;
  enum class CachePartition {
    kFull = 0, kHorizontal0, kHorizontal1, kVertical0, kVertical1, kOther
  };
  enum CacheFeature {
    kFeatureValid = 0,
    kFeatureAnyIntra = 1,
    kFeatureAnyInter = 2,
    kFeatureAnySkip = 3,
  };
  struct CacheEntry {
    std::array<bool, kNumCuPerEntry> valid;
    std::array<CodingUnit*, kNumCuPerEntry> cu;
    uint8_t features;
  };

  CacheEntry* Find(const CodingUnit &cu);
  CachePartition DetermineCuPartition(const CodingUnit &cu);

  PictureData* const pic_data_;
  std::array<std::array<std::array<std::array<CacheEntry,
    kNumCachePartitions>,
    constants::kQuadSplit>,
    constants::kMaxBlockDepth + 2>,
    constants::kMaxNumCuTrees> cu_cache_;
};

}   // namespace xvc

#endif  // XVC_ENC_LIB_CU_CACHE_H_
