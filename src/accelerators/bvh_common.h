#pragma once

#include "utils/bounding_box.h"

#include <array>
#include <memory>
#include <optional>
#include <vector>

#define USE_BVH_SIMD
//#define ENABLE_BVH_STATS

namespace alpine {
static constexpr uint32_t MAX_PRIMITIVES = 4 * 1024 * 1024;
static constexpr uint32_t MAX_NODES = 2 * MAX_PRIMITIVES;

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

    ~NodeAccessCounter()
    {
        mBvhStats.addNodeAccessCount(mNodeCounter, mAny);
        mBvhStats.addTriangleAccessCount(mTriCounter);
    }

    void incrementNode() { mNodeCounter++; }
    void incrementTriangle() { mTriCounter++; }

private:
    BvhStats& mBvhStats;
    bool mAny;
    uint32_t mNodeCounter = 0;
    uint32_t mTriCounter = 0;
};

struct Primitive
{
    BoundingBox bbox;
    const void* ptr = nullptr;
    uint32_t primId = std::numeric_limits<uint32_t>::max();

    // triangle variables
    float3 vertex;
    float3 edges[2];
    float3 ng;

    std::optional<Intersection> intersect(const Ray& ray) const;
};

struct BuildNode
{
    BoundingBox bbox;
    std::array<std::unique_ptr<BuildNode>, 2> children{};
    uint32_t offset = 0;
    uint16_t primitiveCount = 0;
    uint8_t dim = 0;

    bool isLeaf() const { return primitiveCount > 0; }
};

struct BuildBvh
{
    std::unique_ptr<BuildNode> root;
    std::vector<Primitive> orderedPrimitives;
};

BuildBvh
buildBvh(const std::vector<Primitive>& primitives);
}
