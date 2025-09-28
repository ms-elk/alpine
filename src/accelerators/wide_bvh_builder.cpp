#include "wide_bvh_builder.h"

#include <assert.h>
#include <future>
#include <span>

namespace {
constexpr uint8_t MAX_CLUSTERING_ITERATIONS = 8;
}

namespace alpine {
namespace {
struct BuildPrimitive
{
    BoundingBox bbox;
    uint32_t index;
};

class WideBvhBuilder
{
public:
    WideBvhBuilder(
        const std::vector<Primitive>& primitives,
        std::vector<Primitive>& orderedPrimitives,
        uint8_t leafThreshold,
        std::pmr::monotonic_buffer_resource* arena)
        : mPrimitives(primitives)
        , mOrderedPrimitives(orderedPrimitives)
        , mLeafThreshold(leafThreshold)
        , mAllocator(arena)
    {
        mOrderedPrimitives.resize(primitives.size());
    }

    BuildWideNode* buildNode(
        std::span<BuildPrimitive> buildPrimitives,
        std::atomic<uint32_t>& offset,
        std::atomic<uint32_t>& nodeCount);

private:
    BuildWideNode* createLeaf(
        std::span<BuildPrimitive> buildPrimitives,
        const BoundingBox& bbox,
        std::atomic<uint32_t>& offset);

    BuildWideNode* allocate()
    {
        BuildWideNode* p = nullptr;
        {
            std::lock_guard<std::mutex> lock(mMutex);
            p = mAllocator.allocate(1);
        }

        assert(p);
        std::construct_at(p);

        return p;
    }

private:
    const std::vector<Primitive>& mPrimitives;
    std::vector<Primitive>& mOrderedPrimitives;

    uint8_t mLeafThreshold;

    std::pmr::polymorphic_allocator<BuildWideNode> mAllocator;
    std::mutex mMutex;
};

// k-means++
std::vector<float3>
pickInitialCentroids(
    const float3& seed, std::span<BuildPrimitive> buildPrimitives, uint8_t clusterCount)
{
    std::vector<float3> clusterCentroids;
    clusterCentroids.reserve(clusterCount);

    auto firstCentroid = [&]() {
        uint32_t primId = 0;
        float maxDist2 = 0.0f;
        for (uint32_t i = 0; i < buildPrimitives.size(); ++i)
        {
            float3 v = buildPrimitives[i].bbox.getCenter() - seed;
            float dist2 = dot(v, v);

            if (dist2 > maxDist2)
            {
                maxDist2 = dist2;
                primId = i;
            }
        }

        return buildPrimitives[primId].bbox.getCenter();
        }();

    clusterCentroids.push_back(firstCentroid);

    for (uint8_t k = 1; k < clusterCount; ++k)
    {
        uint32_t primId = 0;
        float maxDist2 = 0.0f;
        for (uint32_t i = 0; i < buildPrimitives.size(); ++i)
        {
            float clusterDist2 = std::numeric_limits<float>::max();
            for (const auto& centroid : clusterCentroids)
            {
                float3 v = buildPrimitives[i].bbox.getCenter() - centroid;
                float dist2 = dot(v, v);

                if (dist2 < clusterDist2)
                {
                    clusterDist2 = dist2;
                }
            }

            if (clusterDist2 > maxDist2)
            {
                maxDist2 = clusterDist2;
                primId = i;
            }
        }

        clusterCentroids.push_back(buildPrimitives[primId].bbox.getCenter());
    }

    return clusterCentroids;
}

std::pair<std::vector<std::vector<BuildPrimitive>>, float /* cost */>
clusterPrimitives(std::span<BuildPrimitive> buildPrimitives, uint8_t clusterCount)
{
    if (buildPrimitives.size() <= clusterCount)
    {
        return { std::vector<std::vector<BuildPrimitive>>(), 0.0f };
    }

    auto centroidBox = [&]() {
        BoundingBox cb;
        for (const auto& bp : buildPrimitives)
        {
            float3 c = bp.bbox.getCenter();
            cb = merge(cb, BoundingBox{ c, c });
        }

        return cb;
        }();

    float cbDiagonalMax = maxComponent(centroidBox.getDiagonal());
    if (cbDiagonalMax == 0.0f)
    {
        return { std::vector<std::vector<BuildPrimitive>>(), 0.0f };
    }

    auto clusterCentroids = pickInitialCentroids(
        centroidBox.getCenter(), buildPrimitives, clusterCount);

    std::vector<std::vector<BuildPrimitive>> clusters(clusterCount);
    const uint32_t clusterSize = 2 * static_cast<uint32_t>(buildPrimitives.size()) / clusterCount;
    for (auto& cluster : clusters)
    {
        cluster.reserve(clusterSize);
    }

    // k-means clustering iteration
    for (uint8_t iter = 0; iter < MAX_CLUSTERING_ITERATIONS; ++iter)
    {
        for (auto& cluster : clusters)
        {
            cluster.clear();
        }

        for (const auto& prim : buildPrimitives)
        {
            uint32_t clusterId = 0;
            float minDist2 = std::numeric_limits<float>::max();
            for (uint8_t k = 0; k < clusterCount; ++k)
            {
                float3 v = prim.bbox.getCenter() - clusterCentroids[k];
                float dist2 = dot(v, v);

                if (dist2 < minDist2 && clusters[k].size() < clusterSize)
                {
                    minDist2 = dist2;
                    clusterId = k;
                }
            }

            clusters[clusterId].push_back(prim);
        }

        for (uint8_t k = 0; k < clusterCount; ++k)
        {
            const auto& cluster = clusters[k];

            clusterCentroids[k] = float3(0.0f);

            for (const auto& prim : cluster)
            {
                clusterCentroids[k] += prim.bbox.getCenter();
            }

            clusterCentroids[k] /= static_cast<float>(cluster.size());
        }
    }

    float cost = 0.0f;
    for (const auto& cluster : clusters)
    {
        BoundingBox bbox;
        for (const auto& prim : cluster)
        {
            bbox = merge(bbox, prim.bbox);
        }

        cost += bbox.computeSurfaceArea() * cluster.size();
    }

    return { clusters, cost };
}

void
sortChildNodeByLongestAxis(BuildWideNode* node)
{
    BoundingBox centroidBox;
    for (const auto& child : node->children)
    {
        if (child)
        {
            centroidBox = merge(centroidBox, child->bbox.getCenter());
        }
    }

    float maxDist = 0.0f;
    for (uint8_t dim = 0; dim < 3; ++dim)
    {
        float dist = centroidBox.max[dim] - centroidBox.min[dim];

        if (dist > maxDist)
        {
            maxDist = dist;
            node->dim = dim;
        }
    }

    std::sort(node->children.begin(), node->children.end(),
        [dim = node->dim](const auto* a, const auto* b) {
            if (!a && !b)
            {
                return false;
            }
            if (!a)
            {
                return true;
            }
            if (!b)
            {
                return false;
            }

            return a->bbox.getCenter()[dim] < b->bbox.getCenter()[dim];
        });
}


BuildWideNode*
WideBvhBuilder::buildNode(
    std::span<BuildPrimitive> buildPrimitives,
    std::atomic<uint32_t>& offset,
    std::atomic<uint32_t>& nodeCount)
{
    if (buildPrimitives.empty())
    {
        return nullptr;
    }

    nodeCount++;

    BoundingBox nodeBbox;
    for (const auto& bp : buildPrimitives)
    {
        nodeBbox = merge(nodeBbox, bp.bbox);
    }

    if (buildPrimitives.size() <= mLeafThreshold)
    {
        return createLeaf(buildPrimitives, nodeBbox, offset);
    }

    auto [subset, splitCost] = clusterPrimitives(buildPrimitives, SIMD_WIDTH);
    float leafCost = static_cast<float>(buildPrimitives.size()) * nodeBbox.computeSurfaceArea();

    if (subset.empty() || splitCost >= leafCost)
    {
        return createLeaf(buildPrimitives, nodeBbox, offset);
    }

    auto* node = allocate();
    node->bbox = nodeBbox;

    if (buildPrimitives.size() >= MIN_PRIMITIVES_FOR_PARALLELIZATION)
    {
        std::array<std::future<BuildWideNode*>, SIMD_WIDTH> children;
        for (uint8_t i = 0; i < SIMD_WIDTH; ++i)
        {
            children[i] = std::async([&, i]() {
                return buildNode(subset[i], offset, nodeCount);
                });
        }

        for (uint8_t i = 0; i < SIMD_WIDTH; ++i)
        {
            node->children[i] = children[i].get();
        }
    }
    else
    {
        for (uint8_t i = 0; i < SIMD_WIDTH; ++i)
        {
            node->children[i] = buildNode(subset[i], offset, nodeCount);
        }
    }

    sortChildNodeByLongestAxis(node);

    return node;
}

BuildWideNode*
WideBvhBuilder::createLeaf(
    std::span<BuildPrimitive> buildPrimitives,
    const BoundingBox& bbox,
    std::atomic<uint32_t>& offset)
{
    auto* leaf = allocate();

    leaf->bbox = bbox;
    leaf->primitiveCount = static_cast<uint16_t>(buildPrimitives.size());
    leaf->offset = offset.fetch_add(leaf->primitiveCount);

    for (uint32_t i = 0; i < buildPrimitives.size(); ++i)
    {
        uint32_t srcIdx = buildPrimitives[i].index;
        uint32_t dstIdx = leaf->offset + i;
        assert(dstIdx < mOrderedPrimitives.size());
        mOrderedPrimitives[dstIdx] = mPrimitives[srcIdx];
    }

    //printf("%zd, ", buildPrimitives.size());

    return leaf;
}
}

BuildWideNode*
buildWideBvh(
    const std::vector<Primitive>& primitives,
    std::vector<Primitive>& orderedPrimitives,
    uint8_t leafThreshold,
    std::pmr::monotonic_buffer_resource* arena)
{
    WideBvhBuilder wideBvhBuilder(primitives, orderedPrimitives, leafThreshold, arena);

    std::pmr::vector<BuildPrimitive> buildPrimitives(arena);
    buildPrimitives.resize(primitives.size());
    for (uint32_t i = 0; i < primitives.size(); ++i)
    {
        auto& bp = buildPrimitives[i];
        bp.index = i;
        bp.bbox = primitives[i].bbox;
    }

    std::atomic<uint32_t> buildNodeOffset = 0;
    std::atomic<uint32_t> nodeCount = 0;

    return wideBvhBuilder.buildNode(buildPrimitives, buildNodeOffset, nodeCount);
}
}
