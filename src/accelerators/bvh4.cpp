#include "bvh4.h"

#include "alpine_config.h"
#include "ray.h"
#include "utils/bounding_box.h"
#include "utils/bvh_util.h"

#include <ispc/bounding_box.h>

#include <array>
#include <atomic>
#include <assert.h>
#include <future>

// TODO: 
// - triangle intersection tests using SIMD
// - generalize the code between bvh4 and bvh
// - Node alignment

namespace alpine {
namespace {
static constexpr uint32_t MAX_PRIMITIVES = 128 * 1024;
static constexpr uint8_t CHILD_NODE_COUNT = 4;
static constexpr uint8_t MAX_CLUSTERING_ITERATIONS = 8;
static constexpr uint32_t MIN_PRIMITIVES_FOR_PARALLELIZATION = 1024;

static constexpr uint8_t STACK_SIZE = 64;

static constexpr uint8_t SPLITS_PER_DIM = 4;
static constexpr uint8_t BIN_COUNT = SPLITS_PER_DIM + 1;

class BvhStats
{
public:
    void show()
    {
        printf("BVH Total Node Count: %zu\n", mNodeCount);
        printf("BVH Node Access Count (Closest): %zu\n", mClosestCount.load());
        printf("BVH Node Access Count (Any): %zu\n", mAnyCount.load());
        printf("BVH Triangle Access Count: %zu\n", mTriangleCount.load());
    }

    void countNodes(uint64_t nodeCount)
    {
        mNodeCount = nodeCount;
    }

    void addNodeAccessCount(uint32_t count, bool any)
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

    void addTriangleAccessCount(uint32_t count)
    {
#ifdef ENABLE_BVH_STATS
        mTriangleCount += count;
#endif
    }

private:
    uint64_t mNodeCount = 0;
    std::atomic<uint64_t> mClosestCount = 0;
    std::atomic<uint64_t> mAnyCount = 0;
    std::atomic<uint64_t> mTriangleCount = 0;
};

BvhStats gBvhStats;

class NodeAccessCounter
{
public:
    NodeAccessCounter(bool any) : mAny(any) {};

    ~NodeAccessCounter()
    {
        gBvhStats.addNodeAccessCount(mNodeCounter, mAny);
        gBvhStats.addTriangleAccessCount(mTriCounter);
    }

    void incrementNode() { mNodeCounter++; }
    void incrementTriangle() { mTriCounter++; }

private:
    uint32_t mNodeCounter = 0;
    uint32_t mTriCounter = 0;
    bool mAny;
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
    uint32_t index = 0;
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

struct alignas(32) LinearNode4
{
#ifdef USE_BVH_SIMD
    ispc::BoundingBox4 bbox4;
#else
    BoundingBox bbox[4];
#endif

    static constexpr uint32_t invalid_offset = std::numeric_limits<uint32_t>::max();
    std::array<uint32_t, CHILD_NODE_COUNT - 1> offset;

    uint16_t primitiveCount = 0;
    std::array<uint8_t, 3> dim{};

    bool isLeaf() const { return primitiveCount > 0; }

    LinearNode4()
    {
        std::fill(offset.begin(), offset.end(), invalid_offset);

#ifdef USE_BVH_SIMD
        for (uint8_t i = 0; i < CHILD_NODE_COUNT; ++i)
        {
            bbox4.minX[i] = std::numeric_limits<float>::max();
            bbox4.minY[i] = std::numeric_limits<float>::max();
            bbox4.minZ[i] = std::numeric_limits<float>::max();

            bbox4.maxX[i] = -std::numeric_limits<float>::max();
            bbox4.maxY[i] = -std::numeric_limits<float>::max();
            bbox4.maxZ[i] = -std::numeric_limits<float>::max();
        }
#endif
    }
};

consteval auto
makeTraversalOrder(const std::array<bool, 3>& isDirNegative)
{
    std::array<uint8_t, CHILD_NODE_COUNT> traversalOrder{};

    for (uint8_t i = 0; i < 2; ++i)
    {
        uint8_t parentOrder = isDirNegative[0] ? i : 1 - i;

        for (uint8_t j = 0; j < 2; ++j)
        {
            bool n = parentOrder == 0 ? isDirNegative[1] : isDirNegative[2];
            uint8_t childOrder = n ? j : 1 - j;

            uint8_t idx = 2 * i + j;
            traversalOrder[idx] = 2 * parentOrder + childOrder;
        }
    }

    return traversalOrder;
}

consteval auto
makeTraversalOrderTable() {
    std::array<std::array<uint8_t, CHILD_NODE_COUNT>, 8> table{};
    std::array<bool, 3> isDirNegative{};

    for (uint8_t i = 0; i < 8; ++i) {
        std::array<bool, 3> isDirNegative = {
            static_cast<bool>((i >> 2) & 1),
            static_cast<bool>((i >> 1) & 1),
            static_cast<bool>((i >> 0) & 1)
        };
        table[i] = makeTraversalOrder(isDirNegative);
    }

    return table;
}

constexpr auto gTraversalOrderTable = makeTraversalOrderTable();

}

class Bvh4::Impl
{
public:
    Impl();
    ~Impl();

    void appendMesh(
        const std::vector<float3>& vertices,
        const std::vector<uint3>& prims,
        const void* ptr);

    void appendSphere(const std::vector<float4>& vertices, const void* ptr);

    void updateScene();

    std::optional<Intersection> intersect(const Ray& ray) const;

    bool intersectAny(const Ray& ray, float tFar) const;

private:
    std::unique_ptr<BuildNode> buildBvh(
        const std::vector<BuildPrimitive>& buildPrimitives,
        std::atomic<uint32_t>& offset,
        std::atomic<uint32_t>& nodeCount);

    std::unique_ptr<BuildNode> createLeaf(
        const std::vector<BuildPrimitive>& buildPrimitives,
        const BoundingBox& bbox,
        std::atomic<uint32_t>& offset);

    uint32_t flatten(const BuildNode* node, uint32_t& offset);

    std::optional<Intersection> traverse(const Ray& ray, float tFar, bool any) const;

private:
    std::vector<Primitive> mPrimitives;
    std::vector<Primitive> mOrderedPrimitives;
    std::vector<LinearNode4> mLinearNodes;
};

Bvh4::Impl::Impl()
{
    mPrimitives.reserve(MAX_PRIMITIVES);
    mOrderedPrimitives.reserve(MAX_PRIMITIVES);
}

Bvh4::Impl::~Impl()
{
    gBvhStats.show();
}

void
Bvh4::Impl::appendMesh(
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
Bvh4::Impl::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{
    printf("ERROR: Sphere intersection has not been implemented yet.");
}

void
Bvh4::Impl::updateScene()
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
    mLinearNodes.resize(linearNodeOffset);

    gBvhStats.countNodes(linearNodeOffset);
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
Bvh4::Impl::buildBvh(
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

    std::vector<BuildPrimitive> subset[2];
    for (uint8_t i = 0; i < 2; ++i)
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
        for (uint8_t i = 0; i < 2; ++i)
        {
            children[i] = std::async([&, i]() {
                return buildBvh(subset[i], offset, nodeCount);
                });
        }

        for (uint8_t i = 0; i < 2; ++i)
        {
            node->children[i] = children[i].get();
        }
    }
    else
    {
        for (uint8_t i = 0; i < 2; ++i)
        {
            node->children[i] = buildBvh(subset[i], offset, nodeCount);
        }
    }

    return node;
}

std::unique_ptr<BuildNode>
Bvh4::Impl::createLeaf(
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
Bvh4::Impl::flatten(const BuildNode* node, uint32_t& offset)
{
    assert(node);

    auto& linearNode = mLinearNodes[offset];
    uint32_t nodeOffset = offset++;

    if (node->isLeaf())
    {
        linearNode.offset[0] = node->offset;
        linearNode.primitiveCount = node->primitiveCount;
    }
    else
    {
        linearNode.dim[0] = node->dim;

        const auto flattenChild = [&](uint32_t idx, const auto* child) {
            uint32_t childOffset = flatten(child, offset);

            if (idx > 0)
            {
                linearNode.offset[idx - 1] = childOffset;
            }
        };

        const auto setBbox = [&](uint32_t idx, const auto& bbox) {
            linearNode.bbox4.minX[idx] = bbox.min.x;
            linearNode.bbox4.minY[idx] = bbox.min.y;
            linearNode.bbox4.minZ[idx] = bbox.min.z;

            linearNode.bbox4.maxX[idx] = bbox.max.x;
            linearNode.bbox4.maxY[idx] = bbox.max.y;
            linearNode.bbox4.maxZ[idx] = bbox.max.z;
        };

        for (uint8_t i = 0; i < node->children.size(); ++i)
        {
            const auto& child = node->children[i];
            assert(child);

            linearNode.dim[i + 1] = child->dim;

            if (child->isLeaf())
            {
                uint8_t idx = 2 * i;
                flattenChild(idx, child.get());
#ifdef USE_BVH_SIMD
                setBbox(idx, child->bbox);
#else
                linearNode.bbox[idx] = child->bbox;
#endif
            }
            else
            {
                for (uint8_t j = 0; j < child->children.size(); ++j)
                {
                    const auto& grandchild = child->children[j];
                    assert(grandchild);

                    uint8_t idx = 2 * i + j;
                    flattenChild(idx, grandchild.get());
#ifdef USE_BVH_SIMD
                    setBbox(idx, grandchild->bbox);
#else
                    linearNode.bbox[idx] = grandchild->bbox;
#endif
                }
            }
        }
    }

    return nodeOffset;
}

std::optional<Intersection>
Bvh4::Impl::intersect(const Ray& ray) const
{
    return traverse(ray, std::numeric_limits<float>::max(), false);
}

bool
Bvh4::Impl::intersectAny(const Ray& ray, float tFar) const
{
    const auto intersection = traverse(ray, tFar, true);
    return intersection.has_value();
}

std::optional<Intersection>
Bvh4::Impl::traverse(const Ray& ray, float tFar, bool any) const
{
    NodeAccessCounter counter(any);

    std::optional<Intersection> closestIsect;
    float tNear = tFar;
    float3 invRayDir = float3(1.0f / ray.dir.x, 1.0f / ray.dir.y, 1.0f / ray.dir.z);

    uint32_t stack[STACK_SIZE];
    uint8_t stackIdx = 0;
    uint32_t currentIdx = 0;

    while (true)
    {
        counter.incrementNode();

        const auto& linearNode = mLinearNodes[currentIdx];
        if (linearNode.isLeaf())
        {
            for (uint16_t i = 0; i < linearNode.primitiveCount; ++i)
            {
                counter.incrementTriangle();

                const auto& prim = mOrderedPrimitives[linearNode.offset[0] + i];
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
        else
        {
            std::array<bool, CHILD_NODE_COUNT> intersects{};
#ifdef USE_BVH_SIMD
            static constexpr float correction = 1.0f + 2.0f * gamma(3); // ensure conservative intersection
            ispc::intersectBoundingBox4(intersects.data(),
                &ray.org[0], &ray.dir[0], &invRayDir[0], tNear, correction, linearNode.bbox4);
#else
            for (uint8_t i = 0; i < CHILD_NODE_COUNT; ++i)
            {
                intersects[i] = linearNode.bbox[i].intersect(ray, tNear, invRayDir);
            }
#endif

            const auto stackChild = [&](uint8_t idx) {
                if (!intersects[idx])
                {
                    return;
                }

                assert(stackIdx < STACK_SIZE - 1);

                if (idx == 0)
                {
                    stack[stackIdx++] = currentIdx + 1;
                }
                else
                {
                    uint32_t offset = linearNode.offset[idx - 1];
                    if (offset != LinearNode4::invalid_offset)
                    {
                        stack[stackIdx++] = offset;
                    }
                }
            };

            std::array<bool, 3> isNeg{};
            for (uint8_t i = 0; i < isNeg.size(); ++i)
            {
                uint8_t dim = linearNode.dim[i];
                isNeg[i] = ray.dir[dim] < 0.0f;
            }

            uint8_t tableIdx = (isNeg[0] << 2) | (isNeg[1] << 1) | (isNeg[2] << 0);
            const auto& traversalOrder = gTraversalOrderTable[tableIdx];
            for (uint8_t idx : traversalOrder)
            {
                stackChild(idx);
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

Bvh4::Bvh4()
    : mPimpl(std::make_unique<Impl>())
{}

Bvh4::~Bvh4() = default;

void
Bvh4::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{
    mPimpl->appendMesh(vertices, prims, ptr);
}

void
Bvh4::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{
    mPimpl->appendSphere(vertices, ptr);
}

void
Bvh4::updateScene()
{
    mPimpl->updateScene();
}

std::optional<Intersection>
Bvh4::intersect(const Ray& ray) const
{
    return mPimpl->intersect(ray);
}

bool
Bvh4::intersectAny(const Ray& ray, float tFar) const
{
    return mPimpl->intersectAny(ray, tFar);
}
}
