#pragma once

#include <stdint.h>

#define USE_BVH_SIMD
//#define ENABLE_BVH_STATS

namespace alpine {
static constexpr uint32_t MAX_SHAPES = 1024;
static constexpr float RAY_OFFSET = 0.001f;
}
