#include "bvh_common.h"

#include <utils/bvh_util.h>
#include <intersection.h>

#include <array>
#include <assert.h>
#include <future>

namespace {
static constexpr uint8_t LEAF_THRESHOLD = 8;
static constexpr uint8_t SPLITS_PER_DIM = 8;
static constexpr uint8_t BIN_COUNT = SPLITS_PER_DIM + 1;
static constexpr uint32_t MIN_PRIMITIVES_FOR_PARALLELIZATION = 1024;
}

namespace alpine {
void
BvhStats::show()
{
    printf("BVH Total Node Count: %zu\n", mNodeCount);
    printf("BVH Node Access Count (Closest): %zu\n", mClosestCount.load());
    printf("BVH Node Access Count (Any): %zu\n", mAnyCount.load());
    printf("BVH Triangle Access Count: %zu\n", mTriangleCount.load());
}

void
BvhStats::countNodes(uint64_t nodeCount)
{
    mNodeCount = nodeCount;
}

void
BvhStats::addNodeAccessCount(uint32_t count, bool any)
{
#ifdef ENABLE_BVH_STATS
    if (any)
    {
        mAnyCount += count;
    }
    else
    {
        mClosestCount += count;
    }
#endif
}

void
BvhStats::addTriangleAccessCount(uint32_t count)
{
#ifdef ENABLE_BVH_STATS
    mTriangleCount += count;
#endif
}

std::optional<Intersection>
Primitive::intersect(const Ray& ray) const
{
    float3 s = ray.org - vertex;
    float3 s1 = cross(ray.dir, edges[1]);
    float3 s2 = cross(s, edges[0]);
    float det = dot(s1, edges[0]);

    if (det == 0.0f)
    {
        return {};
    }

    Intersection isect;
    isect.shapePtr = ptr;
    isect.primId = primId;
    isect.ng = ng;

    isect.t = dot(s2, edges[1]) / det;
    if (isect.t < std::numeric_limits<float>::epsilon())
    {
        return {};
    }

    isect.barycentric[0] = dot(s1, s) / det;
    if (isect.barycentric[0] < 0.0f || isect.barycentric[0] > 1.0f)
    {
        return {};
    }

    isect.barycentric[1] = dot(s2, ray.dir) / det;
    if (isect.barycentric[1] < 0.0f || isect.barycentric[1] > 1.0f - isect.barycentric[0])
    {
        return {};
    }

    return isect;
}

void
Primitive::updateVertices(const Shape& shape)
{
    assert(primId != std::numeric_limits<uint32_t>::max());

    uint32_t idx0 = shape.prims[primId][0];
    uint32_t idx1 = shape.prims[primId][1];
    uint32_t idx2 = shape.prims[primId][2];

    vertex = shape.vertices[idx0];
    edges[0] = shape.vertices[idx1] - shape.vertices[idx0];
    edges[1] = shape.vertices[idx2] - shape.vertices[idx0];
    ng = normalize(cross(edges[0], edges[1]));

    bbox = BoundingBox();
    for (uint8_t i = 0; i < 3; ++i)
    {
        uint32_t idx = shape.prims[primId][i];
        bbox = merge(bbox, shape.vertices[idx]);
    }
}

namespace {
struct BuildPrimitive
{
    BoundingBox bbox;
    uint32_t index;
};

struct BvhBuilder
{
public:
    BvhBuilder(const std::vector<Primitive>& primitives)
        : mPrimitives(primitives)
    {
        mOrderedPrimitives.resize(primitives.size());
    }

    std::unique_ptr<BuildNode> buildNode(
        const std::vector<BuildPrimitive>& buildPrimitives,
        std::atomic<uint32_t>& offset,
        std::atomic<uint32_t>& nodeCount);

    std::unique_ptr<BuildNode> createLeaf(
        const std::vector<BuildPrimitive>& buildPrimitives,
        const BoundingBox& bbox,
        std::atomic<uint32_t>& offset);

    std::vector<Primitive> takeOrderedPrimitives()
    {
        return std::move(mOrderedPrimitives);
    }

private:
    const std::vector<Primitive>& mPrimitives;
    std::vector<Primitive> mOrderedPrimitives;
};

std::pair<std::optional<bvh_util::Split>, float /* cost */>
findSplit(const std::vector<BuildPrimitive>& buildPrimitives)
{
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
        return { {}, 0.0f };
    }

    bvh_util::Split split;
    float minCost = std::numeric_limits<float>::max();

    for (uint8_t dim = 0; dim < 3; ++dim)
    {
        if (centroidBox.min[dim] == centroidBox.max[dim])
        {
            continue;
        }

        struct Bin
        {
            BoundingBox bbox;
            uint32_t count = 0;
        };
        Bin bins[BIN_COUNT];

        // binning
        for (const auto& bp : buildPrimitives)
        {
            float c = bp.bbox.getCenter()[dim];
            uint8_t binIdx = bvh_util::getBinIndex(
                c, centroidBox.min[dim], centroidBox.max[dim], BIN_COUNT);
            binIdx = std::min(binIdx, static_cast<uint8_t>(BIN_COUNT - 1));

            auto& bin = bins[binIdx];
            bin.bbox = merge(bin.bbox, bp.bbox);
            bin.count++;
        }

        float costs[SPLITS_PER_DIM] = { 0.0f };

        const auto accumulateCosts = [&](uint8_t binIdx, uint8_t splitIdx, Bin& binAccum) {
            const auto& bin = bins[binIdx];
            binAccum.bbox = merge(binAccum.bbox, bin.bbox);
            binAccum.count += bin.count;

            costs[splitIdx] += binAccum.count * binAccum.bbox.computeSurfaceArea();
            };

        Bin binBelow;
        for (uint8_t splitIdx = 0; splitIdx < SPLITS_PER_DIM; ++splitIdx)
        {
            uint8_t binIdx = splitIdx;
            accumulateCosts(binIdx, splitIdx, binBelow);
        }

        Bin binAbove;
        for (int8_t splitIdx = SPLITS_PER_DIM - 1; splitIdx >= 0; --splitIdx)
        {
            uint8_t binIdx = splitIdx + 1;
            accumulateCosts(binIdx, splitIdx, binAbove);
        }

        // find the split which has the minimum cost
        for (uint8_t splitIdx = 0; splitIdx < SPLITS_PER_DIM; ++splitIdx)
        {
            const auto& cost = costs[splitIdx];
            if (cost < minCost)
            {
                minCost = cost;
                split = { dim, splitIdx, BIN_COUNT, centroidBox.min[dim], centroidBox.max[dim] };
            }
        }
    }

    return { split, minCost };
}

std::unique_ptr<BuildNode>
BvhBuilder::buildNode(
    const std::vector<BuildPrimitive>& buildPrimitives,
    std::atomic<uint32_t>& offset,
    std::atomic<uint32_t>& nodeCount)
{
    nodeCount++;

    BoundingBox nodeBbox;
    for (const auto& bp : buildPrimitives)
    {
        nodeBbox = merge(nodeBbox, bp.bbox);
    }

    if (buildPrimitives.size() <= LEAF_THRESHOLD)
    {
        return createLeaf(buildPrimitives, nodeBbox, offset);
    }

    auto [split, splitCost] = findSplit(buildPrimitives);
    float leafCost = static_cast<float>(buildPrimitives.size()) * nodeBbox.computeSurfaceArea();

    if (!split.has_value() || splitCost >= leafCost)
    {
        return createLeaf(buildPrimitives, nodeBbox, offset);
    }

    std::array<std::vector<BuildPrimitive>, 2> subsets;
    for (auto& subset : subsets)
    {
        subset.reserve(buildPrimitives.size());
    }

    const auto& s = split.value();

    for (const auto& bp : buildPrimitives)
    {
        uint8_t subsetIdx = s.isBelow(bp.bbox.getCenter()) ? 0 : 1;
        subsets[subsetIdx].push_back(bp);
    }

    assert(!subsets[0].empty());
    assert(!subsets[1].empty());

    auto node = std::make_unique<BuildNode>();
    node->bbox = nodeBbox;
    node->dim = s.dim;

    if (buildPrimitives.size() >= MIN_PRIMITIVES_FOR_PARALLELIZATION)
    {
        std::array<std::future<std::unique_ptr<BuildNode>>, 2> children;
        for (uint8_t i = 0; i < children.size(); ++i)
        {
            children[i] = std::async([&, i]() {
                return buildNode(subsets[i], offset, nodeCount);
                });
        }

        for (uint8_t i = 0; i < node->children.size(); ++i)
        {
            node->children[i] = children[i].get();
        }
    }
    else
    {
        for (uint8_t i = 0; i < node->children.size(); ++i)
        {
            node->children[i] = buildNode(subsets[i], offset, nodeCount);
        }
    }

    return node;
}

std::unique_ptr<BuildNode>
BvhBuilder::createLeaf(
    const std::vector<BuildPrimitive>& buildPrimitives,
    const BoundingBox& bbox,
    std::atomic<uint32_t>& offset)
{
    auto leaf = std::make_unique<BuildNode>();

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

    return leaf;
}
}

BuildBvh
buildBvh(const std::vector<Primitive>& primitives)
{
    BvhBuilder bvhBuilder(primitives);

    std::vector<BuildPrimitive> buildPrimitives(primitives.size());
    for (uint32_t i = 0; i < primitives.size(); ++i)
    {
        auto& bp = buildPrimitives[i];
        bp.index = i;
        bp.bbox = primitives[i].bbox;
    }

    std::atomic<uint32_t> buildNodeOffset = 0;
    std::atomic<uint32_t> nodeCount = 0;

    BuildBvh buildBvh;
    buildBvh.root = bvhBuilder.buildNode(buildPrimitives, buildNodeOffset, nodeCount);
    buildBvh.orderedPrimitives = bvhBuilder.takeOrderedPrimitives();

    return buildBvh;
}
}
