#pragma once

#include "bvh_common.h"

#include <ispc/ispc_config.h>

#include <array>
#include <vector>

namespace alpine
{
struct BuildWideNode
{
    BoundingBox bbox;
    std::array<BuildWideNode*, SIMD_WIDTH> children{};
    uint32_t offset = 0;
    uint16_t primitiveCount = 0;
    uint8_t dim = 0;

    bool isLeaf() const { return primitiveCount > 0; }
};

BuildWideNode* buildWideBvh(
    const std::vector<Primitive>& primitives,
    std::vector<Primitive>& orderedPrimitives,
    uint8_t leafThreshold,
    std::pmr::monotonic_buffer_resource* arena);

}
