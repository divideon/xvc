/******************************************************************************
* Copyright (C) 2017, Divideon. All rights reserved.
* No part of this code may be reproduced in any form
* without the written permission of the copyright holder.
******************************************************************************/

#include <stdint.h>

#include "xvc_common_lib/simd_cpu.h"

#ifdef _MSC_VER
#include <intrin.h>
#endif  // _MSC_VER

#include <string.h>

namespace xvc {

std::set<CpuCapability> SimdCpu::GetMaskedCaps(uint32_t mask) {
  const auto caps = GetRuntimeCapabilities();
  std::set<CpuCapability> out;
  for (auto it = caps.begin(); it != caps.end(); ++it) {
    CpuCapability cap = *it;
    if (((1 << static_cast<int>(cap)) & mask) != 0) {
      out.insert(cap);
    }
  }
  return out;
}

#if XVC_ARCH_X86

static void cpuinfo_x86(uint32_t eax, uint32_t ecx, int registers[4]) {
#ifdef _MSC_VER
  if (eax <= 1) {
    __cpuid(registers, eax);
  } else {
    __cpuidex(registers, eax, ecx);
  }
#else
  uint32_t ebx, edx;
  asm volatile (  // NOLINT
#if defined( __i386__) && defined(__PIC__)
                "mov %%ebx, %%edi                          \n"
                "cpuid                                     \n"
                "xchg %%edi, %%ebx                         \n"
                : "=D" (ebx),
#else
                "cpuid                                     \n"
                : "=b" (ebx),
#endif  //  defined( __i386__) && defined(__PIC__)
                "+a" (eax), "+c" (ecx), "=d" (edx));
  registers[0] = eax;
  registers[1] = ebx;
  registers[2] = ecx;
  registers[3] = edx;
#endif
}

static bool cpu_xgetbv_x86() {
#ifdef _MSC_VER
  auto xcr0 = _xgetbv(0);
  return (xcr0 & 6) == 6;
#else
  uint32_t xcr0;
  asm(".byte 0x0f, 0x01, 0xd0" : "=a" (xcr0) : "c" (0) : "%edx");
  return (xcr0 & 6) == 6;
#endif
}

std::set<CpuCapability> SimdCpu::GetRuntimeCapabilities() {
  std::set<CpuCapability> caps;
  int cpuinfo[4];
  cpuinfo_x86(0, 0, cpuinfo);
  bool has_extended_caps = cpuinfo[0] >= 7;
  bool has_xgetbv = false;

  cpuinfo_x86(1, 0, cpuinfo);
  if (cpuinfo[3] & 0x00800000) {
    caps.insert(CpuCapability::kMmx);
  }
  if (cpuinfo[3] & 0x02000000) {
    caps.insert(CpuCapability::kSse);
  }
  if (cpuinfo[3] & 0x04000000) {
    caps.insert(CpuCapability::kSse2);
  }
  if (cpuinfo[2] & 0x00000200) {
    caps.insert(CpuCapability::kSse3);
  }
  if (cpuinfo[2] & 0x00080000) {
    caps.insert(CpuCapability::kSse4_1);
  }
  if (cpuinfo[2] & 0x00100000) {
    caps.insert(CpuCapability::kSse4_2);
  }
  if ((cpuinfo[2] & 0x18000000) == 0x18000000) {
    // AVX and AVX2 addionally requires support by OS
    has_xgetbv = cpu_xgetbv_x86();
    if (has_xgetbv) {
      caps.insert(CpuCapability::kAvx);
    }
  }
  if (has_extended_caps && has_xgetbv) {
    cpuinfo_x86(7, 0, cpuinfo);
    if (cpuinfo[1] & 0x00000020) {
      caps.insert(CpuCapability::kAvx2);
    }
  }
  return caps;
}

#elif XVC_ARCH_ARM

#ifdef _MSC_VER
static int cpu_has_neon_arm() {
  return -1;
}
#elif defined(__linux__)
#include <stdio.h>
static int cpu_has_neon_arm() {
  FILE* f = fopen("/proc/cpuinfo", "r");
  if (!f) {
    return -1;
  }
  char buf[512];
  while (fgets(buf, sizeof(buf) - 1, f)) {
    if (memcmp(buf, "Features", 8) == 0) {
      char* p = strstr(buf, " neon");
      if (p && (p[5] == ' ' || p[5] == '\n')) {
        fclose(f);
        return 1;
      }
      p = strstr(buf, " asimd");
      if (p && (p[6] == ' ' || p[6] == '\n')) {
        fclose(f);
        return 1;
      }
    }
  }
  fclose(f);
  return 0;
}
#else
#error "Unknown architecture for runtime NEON detection"
#endif

std::set<CpuCapability> SimdCpu::GetRuntimeCapabilities() {
  std::set<CpuCapability> caps;

#if XVC_HAVE_NEON
  int ret = cpu_has_neon_arm();
  if (ret > 0) {
    caps.insert(CpuCapability::kNeon);
  } else if (ret < 0) {
    // don't know => assume NEON
    caps.insert(CpuCapability::kNeon);
  }
#endif  // XVC_HAVE_NEON

  return caps;
}

#else

std::set<CpuCapability> SimdCpu::GetRuntimeCapabilities() {
  return{};
}

#endif

}   // namespace xvc
