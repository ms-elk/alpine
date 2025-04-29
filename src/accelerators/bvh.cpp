#include "bvh.h"

#include "ray.h"
#include "utils/bounding_box.h"
#include "utils/bvh_util.h"

#include <array>
#include <assert.h>
#include <future>

namespace alpine {
namespace {
static constexpr uint32_t MAX_PRIMITIVES = 128 * 1024;
static constexpr uint8_t CHILD_NODE_COUNT = 2;

static constexpr uint8_t SPLITS_PER_DIM = 4;
static constexpr uint8_t BIN_COUNT = SPLITS_PER_DIM + 1;

static constexpr uint32_t MIN_PRIMITIVES_FOR_PARALLELIZATION = 1024;

static constexpr uint8_t STACK_SIZE = 64;
}

struct Primitive
{
    BoundingBox bbox;
    const void* ptr = nullptr;
    uint32_t primId = std::numeric_limits<uint32_t>::max();

    // triangle variables
    float3 vertex;
    float3 edges[2];
    float3 ng;

    std::optional<Intersection> intersect(const Ray& ray) const
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
};

struct BuildPrimitive
{
    BoundingBox bbox;
    uint32_t index;
};

struct BuildNode
{
    BoundingBox bbox;
    std::array<std::unique_ptr<BuildNode>, CHILD_NODE_COUNT> children{};
    uint32_t offset;
    uint16_t primitiveCount = 0;
    uint8_t dim;

    bool isLeaf() const { return primitiveCount > 0; }
};

struct alignas(32) LinearNode
{
    BoundingBox bbox;
    uint32_t offset;
    uint16_t primitiveCount = 0;
    uint8_t dim = 0;

    bool isLeaf() const { return primitiveCount > 0; }
};

namespace {
class BvhStats
{
public:
    void show()
    {
        printf("BVH Total Node Count: %zu\n", mInteriorCount + mLeafCount);
        printf("BVH Interior Node Count: %zu\n", mInteriorCount);
        printf("BVH Leaf Node Count: %zu\n", mLeafCount);
        printf("BVH Node Access Count (Closest): %zu\n", mClosestCount.load());
        printf("BVH Node Access Count (Any): %zu\n", mAnyCount.load());
    }

    void countNodes(const std::vector<LinearNode>& linearNodes)
    {
        mInteriorCount = 0;
        mLeafCount = 0;

        for (const auto& ln : linearNodes)
        {
            if (ln.isLeaf())
            {
                mLeafCount++;
            }
            else
            {
                mInteriorCount++;
            }
        }
    }

    void addNodeAccessCount(uint32_t count, bool any)
    {
        if (any)
        {
            mAnyCount += count;
        }
        else
        {
            mClosestCount += count;
        }
    }

private:
    uint64_t mInteriorCount = 0;
    uint64_t mLeafCount = 0;
    std::atomic<uint64_t> mClosestCount = 0;
    std::atomic<uint64_t> mAnyCount = 0;
};

BvhStats gBvhStats;

class NodeAccessCounter
{
public:
    NodeAccessCounter(bool any) : mCounter(0), mAny(any) {};

    ~NodeAccessCounter()
    {
        gBvhStats.addNodeAccessCount(mCounter, mAny);
    }

    void increment() { mCounter++; }

private:
    uint32_t mCounter;
    bool mAny;
};
}

Bvh::Bvh()
{
    mPrimitives.reserve(MAX_PRIMITIVES);
    mOrderedPrimitives.reserve(MAX_PRIMITIVES);
}

Bvh::~Bvh()
{
    gBvhStats.show();
}

void
Bvh::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{
    for (uint32_t primId = 0; primId < prims.size(); ++primId)
    {
        Primitive prim;
        prim.ptr = ptr;
        prim.primId = primId;

        uint32_t idx0 = prims[primId][0];
        uint32_t idx1 = prims[primId][1];
        uint32_t idx2 = prims[primId][2];
        prim.vertex = vertices[idx0];
        prim.edges[0] = vertices[idx1] - vertices[idx0];
        prim.edges[1] = vertices[idx2] - vertices[idx0];
        prim.ng = normalize(cross(prim.edges[0], prim.edges[1]));

        for (uint8_t i = 0; i < 3; ++i)
        {
            uint32_t idx = prims[primId][i];
            prim.bbox = merge(prim.bbox, vertices[idx]);
        }

        mPrimitives.push_back(prim);
    }
}

void
Bvh::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{
    printf("ERROR: Sphere intersection has not been implemented yet.");
}

void
Bvh::updateScene()
{
    if (mPrimitives.empty())
    {
        return;
    }

    mOrderedPrimitives.resize(mPrimitives.size());

    std::vector<BuildPrimitive> buildPrimitives(mPrimitives.size());
    for (uint32_t i = 0; i < mPrimitives.size(); ++i)
    {
        auto& bp = buildPrimitives[i];
        bp.index = i;
        bp.bbox = mPrimitives[i].bbox;
    }

    std::atomic<uint32_t> buildNodeOffset = 0;
    std::atomic<uint32_t> nodeCount = 0;
    auto bvh = buildBvh(buildPrimitives, buildNodeOffset, nodeCount);

    mLinearNodes.resize(nodeCount);
    uint32_t linearNodeOffset = 0;
    flatten(bvh.get(), linearNodeOffset);

    gBvhStats.countNodes(mLinearNodes);
}

namespace {
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
            uint8_t binIdx = bvh_util::getBinIndex(c, centroidBox.min[dim], centroidBox.max[dim], BIN_COUNT);
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
}

std::unique_ptr<BuildNode>
Bvh::buildBvh(
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

    if (buildPrimitives.size() <= 2)
    {
        return createLeaf(buildPrimitives, nodeBbox, offset);
    }

    auto [split, splitCost] = findSplit(buildPrimitives);
    float leafCost = static_cast<float>(buildPrimitives.size()) * nodeBbox.computeSurfaceArea();

    if (!split.has_value() || splitCost >= leafCost)
    {
        return createLeaf(buildPrimitives, nodeBbox, offset);
    }

    std::vector<BuildPrimitive> subset[CHILD_NODE_COUNT];
    for (uint8_t i = 0; i < CHILD_NODE_COUNT; ++i)
    {
        subset[i].reserve(buildPrimitives.size());
    }

    const auto& s = split.value();

    for (const auto& bp : buildPrimitives)
    {
        uint8_t subsetIdx = s.isBelow(bp.bbox.getCenter()) ? 0 : 1;
        subset[subsetIdx].push_back(bp);
    }

    assert(!subset[0].empty());
    assert(!subset[1].empty());

    auto node = std::make_unique<BuildNode>();
    node->bbox = nodeBbox;
    node->dim = s.dim;

    if (buildPrimitives.size() >= MIN_PRIMITIVES_FOR_PARALLELIZATION)
    {
        std::future<std::unique_ptr<BuildNode>> children[2];
        for (uint8_t i = 0; i < CHILD_NODE_COUNT; ++i)
        {
            children[i] = std::async([&, i]() {
                return buildBvh(subset[i], offset, nodeCount);
            });
        }

        for (uint8_t i = 0; i < CHILD_NODE_COUNT; ++i)
        {
            node->children[i] = children[i].get();
        }
    }
    else
    {
        for (uint8_t i = 0; i < CHILD_NODE_COUNT; ++i)
        {
            node->children[i] = buildBvh(subset[i], offset, nodeCount);
        }
    }

    return node;
}

std::unique_ptr<BuildNode>
Bvh::createLeaf(
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

uint32_t
Bvh::flatten(const BuildNode* node, uint32_t& offset)
{
    auto& linearNode = mLinearNodes[offset];
    uint32_t nodeOffset = offset++;
    linearNode.bbox = node->bbox;
    if (node->isLeaf())
    {
        linearNode.offset = node->offset;
        linearNode.primitiveCount = node->primitiveCount;
    }
    else
    {
        linearNode.dim = node->dim;
        flatten(node->children[0].get(), offset);
        linearNode.offset = flatten(node->children[1].get(), offset);
    }

    return nodeOffset;
}

std::optional<Intersection>
Bvh::intersect(const Ray& ray) const
{
    return traverse(ray, std::numeric_limits<float>::max(), false);
}

bool
Bvh::intersectAny(const Ray& ray, float tFar) const
{
    const auto intersection = traverse(ray, tFar, true);
    return intersection.has_value();
}

std::optional<Intersection>
Bvh::traverse(const Ray& ray, float tFar, bool any) const
{
    assert(!mLinearNodes.empty());

    NodeAccessCounter counter(any);

    std::optional<Intersection> closestIsect;
    float tNear = tFar;
    float3 invRayDir = float3(1.0f / ray.dir.x, 1.0f / ray.dir.y, 1.0f / ray.dir.z);

    uint32_t stack[STACK_SIZE];
    uint8_t stackIdx = 0;
    uint32_t currentIdx = 0;

    while (true)
    {
        counter.increment();

        const auto& linearNode = mLinearNodes[currentIdx];
        if (linearNode.bbox.intersect(ray, tNear, invRayDir))
        {
            if (!linearNode.isLeaf())
            {
                assert(stackIdx < STACK_SIZE - 1);

                bool isDirNegative = ray.dir[linearNode.dim] < 0.0f;
                if (isDirNegative)
                {
                    stack[stackIdx++] = currentIdx + 1;
                    currentIdx = linearNode.offset;
                }
                else
                {
                    stack[stackIdx++] = linearNode.offset;
                    currentIdx = currentIdx + 1;
                }

                continue;
            }

            for (uint16_t i = 0; i < linearNode.primitiveCount; ++i)
            {
                const auto& prim = mOrderedPrimitives[linearNode.offset + i];
                auto isect = prim.intersect(ray);
                if (!isect.has_value() || isect.value().t >= tNear)
                {
                    continue;
                }

                if (any)
                {
                    return isect;
                }
                else
                {
                    closestIsect = isect.value();
                    tNear = isect.value().t;
                }
            }
        }

        if (stackIdx == 0)
        {
            break;
        }

        currentIdx = stack[--stackIdx];
    }

    return closestIsect;
}
}
