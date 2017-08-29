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
#include "xvc_enc_lib/cu_cache.h"

#include <algorithm>
#include <cassert>

#if _MSC_VER
// Disables C6294 Ill-defined for-loop: initial condition does not satisfy test.
// When kNumCuPerEntry is set to 0 the concept of caching cu objects is not used
#pragma warning(disable:6294)
#endif

namespace xvc {

CuCache::CuCache(PictureData *pic_data)
  : pic_data_(pic_data) {
  for (int tree_idx = 0; tree_idx < constants::kMaxNumCuTrees; tree_idx++) {
    const CuTree cu_tree = static_cast<CuTree>(tree_idx);
    const int max_depth = static_cast<int>(cu_cache_[tree_idx].size());
    for (int depth = 0; depth < max_depth; depth++) {
      for (int quad = 0; quad < constants::kQuadSplit; quad++) {
        for (int part = 0; part < kNumCachePartitions; part++) {
          CacheEntry &cache_entry = cu_cache_[tree_idx][depth][quad][part];
          for (int cache_idx = 0; cache_idx < kNumCuPerEntry; cache_idx++) {
            cache_entry.valid[cache_idx] = false;
            cache_entry.cu[cache_idx] =
              pic_data_->CreateCu(cu_tree, depth, -1, -1, 0, 0);
          }
        }
      }
    }
  }
}

CuCache::~CuCache() {
  for (int tree_idx = 0; tree_idx < constants::kMaxNumCuTrees; tree_idx++) {
    const int max_depth = static_cast<int>(cu_cache_[tree_idx].size());
    for (int depth = 0; depth < max_depth; depth++) {
      for (int quad = 0; quad < constants::kQuadSplit; quad++) {
        for (int part = 0; part < kNumCachePartitions; part++) {
          CacheEntry &cache_entry = cu_cache_[tree_idx][depth][quad][part];
          for (int cu_idx = 0; cu_idx < kNumCuPerEntry; cu_idx++) {
            pic_data_->ReleaseCu(cache_entry.cu[cu_idx]);
          }
        }
      }
    }
  }
}

void CuCache::Invalidate(CuTree cu_tree, int cu_depth) {
  const int tree_idx = static_cast<int>(cu_tree);
  auto clear_depth = [this, &tree_idx](int depth) {
    for (int quad = 0; quad < constants::kQuadSplit; quad++) {
      for (int part = 0; part < kNumCachePartitions; part++) {
        CacheEntry &cache_entry = cu_cache_[tree_idx][depth][quad][part];
        cache_entry.features = 0;
        for (int cu_idx = 0; cu_idx < kNumCuPerEntry; cu_idx++) {
          cache_entry.valid[cu_idx] = false;
        }
      }
    }
  };
  if (cu_depth == 0) {
    clear_depth(0);
  }
  clear_depth(cu_depth + 1);
}

CuCache::Result CuCache::Lookup(const CodingUnit &cu) {
  CacheEntry *cache_entry = Find(cu);
  if (!cache_entry) {
    return Result{ nullptr, false, false, false, false };
  }
  bool any_intra = false;
  bool any_inter = false;
  bool any_skip = false;
  if (cache_entry->features & (1 << kFeatureValid)) {
    any_intra = (cache_entry->features & (1 << kFeatureAnyIntra)) != 0;
    any_inter = (cache_entry->features & (1 << kFeatureAnyInter)) != 0;
    any_skip = (cache_entry->features & (1 << kFeatureAnySkip)) != 0;
  }
  const CodingUnit *cached_cu = nullptr;
  for (int cu_idx = 0; cu_idx < kNumCuPerEntry; cu_idx++) {
    if (cache_entry->valid[cu_idx]) {
      cached_cu = cache_entry->cu[cu_idx];
      break;
    }
  }
  return Result{ cached_cu, true, any_intra, any_inter, any_skip };
}

bool CuCache::Store(const CodingUnit &cu) {
  CacheEntry *cache_entry = Find(cu);
  if (!cache_entry) {
    return false;
  }
  cache_entry->features |= 1 << kFeatureValid;
  cache_entry->features |= cu.IsIntra() << kFeatureAnyIntra;
  cache_entry->features |= cu.IsInter() << kFeatureAnyInter;
  cache_entry->features |= cu.GetSkipFlag() << kFeatureAnySkip;
  for (int cu_idx = 0; cu_idx < kNumCuPerEntry; cu_idx++) {
    if (cache_entry->valid[cu_idx]) {
      continue;
    }
    cache_entry->valid[cu_idx] = true;
    *cache_entry->cu[cu_idx] = cu;
    return true;
  }
  return false;
}

CuCache::CacheEntry* CuCache::Find(const CodingUnit &cu) {
  const YuvComponent comp = YuvComponent::kY;
  // Determine partition within smallest enclosing square cu
  const CachePartition partition = DetermineCuPartition(cu);
  if (partition == CachePartition::kOther) {
    return nullptr;
  }

  // Determine depth of smallest enclosing square cu
  const int quad_size = std::max(cu.GetWidth(comp), cu.GetHeight(comp));
  const int quad_depth =
    util::SizeToLog2(constants::kCtuSize) - util::SizeToLog2(quad_size);

  // Determine sub-quad location within parent square cu
  const int parent_quad_size = quad_size << 1;
  const int quad_pos =
    ((cu.GetPosY(comp) & (parent_quad_size - 1)) < quad_size ? 0 : 2) +
    ((cu.GetPosX(comp) & (parent_quad_size - 1)) < quad_size ? 0 : 1);

  const int cu_tree = static_cast<int>(cu.GetCuTree());
  return &cu_cache_[cu_tree][quad_depth][quad_pos][static_cast<int>(partition)];
}

CuCache::CachePartition CuCache::DetermineCuPartition(const CodingUnit &cu) {
  const YuvComponent comp = YuvComponent::kY;
  const int width = cu.GetWidth(comp);
  const int height = cu.GetHeight(comp);
  if (width == height) {
    return CachePartition::kFull;
  }
  if (width == (height << 1)) {
    return (cu.GetPosY(comp) & ((height << 1) - 1)) == 0 ?
      CachePartition::kHorizontal0 : CachePartition::kHorizontal1;
  }
  if ((width << 1) == height) {
    return (cu.GetPosX(comp) & ((width << 1) - 1)) == 0 ?
      CachePartition::kVertical0 : CachePartition::kVertical1;
  }
  return CachePartition::kOther;
}

}   // namespace xvc
