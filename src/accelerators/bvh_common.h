#pragma once

#include <utils/bounding_box.h>

#include <array>
#include <memory_resource>
#include <optional>
#include <vector>

//#define ENABLE_BVH_STATS

namespace alpine {
static constexpr uint32_t MAX_PRIMITIVES = 4 * 1024 * 1024;
static constexpr uint32_t MAX_NODES = MAX_PRIMITIVES;
static constexpr uint8_t STACK_SIZE = 64;

struct Intersection;

class BvhStats
{
public:
    void show();

    void countNodes(uint64_t nodeCount);

    void addNodeAccessCount(uint32_t count, bool any);

    void addTriangleAccessCount(uint32_t count);

private:
    uint64_t mNodeCount = 0;
    std::atomic<uint64_t> mClosestCount = 0;
    std::atomic<uint64_t> mAnyCount = 0;
    std::atomic<uint64_t> mTriangleCount = 0;
};

class NodeAccessCounter
{
public:
    NodeAccessCounter(BvhStats& bvhStats, bool any) : mBvhStats(bvhStats), mAny(any) {};

    ~NodeAccessCounter();

    void incrementNode() {
#ifdef ENABLE_BVH_STATS
        mNodeCounter++;
#endif
    }

    void incrementTriangle() {
#ifdef ENABLE_BVH_STATS
        mTriCounter++;;
#endif
    }

private:
    BvhStats& mBvhStats;
    bool mAny;
    uint32_t mNodeCounter = 0;
    uint32_t mTriCounter = 0;
};

struct BvhShape
{
    std::vector<float3> vertices;
    std::vector<uint3> prims;
    uint32_t primOffset;
};

struct Primitive
{
    const void* ptr = nullptr;
    uint32_t primId = std::numeric_limits<uint32_t>::max();

    // triangle variables
    float3 vertex;
    float3 edges[2];
    float3 ng;

    BoundingBox bbox;

    std::optional<Intersection> intersect(const Ray& ray) const;

    void updateVertices(const BvhShape& bvhShape);
};

struct BuildNode
{
    BoundingBox bbox;
    std::array<BuildNode*, 2> children{};
    uint32_t offset = 0;
    uint16_t primitiveCount = 0;
    uint8_t dim = 0;

    bool isLeaf() const { return primitiveCount > 0; }
};

BuildNode*
buildBvh(
    const std::vector<Primitive>& primitives,
    std::vector<Primitive>& orderedPrimitives,
    uint8_t leafThreshold,
    std::pmr::monotonic_buffer_resource* arena);
}
