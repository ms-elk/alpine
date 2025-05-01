#include "bvh4.h"

#include "alpine_config.h"
#include "ray.h"
#include "utils/bounding_box.h"

#include <ispc/bvh.h>

#include <array>
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
}

struct Primitive4
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

struct BuildPrimitive4
{
    BoundingBox bbox;
    uint32_t index;
};

struct BuildNode4
{
    BoundingBox bbox;
    std::array<std::unique_ptr<BuildNode4>, CHILD_NODE_COUNT> children{};
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
    std::array <uint32_t, CHILD_NODE_COUNT - 1> offset;

    uint16_t primitiveCount = 0;
    uint8_t dim = 0;

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

Bvh4::Bvh4()
{
    mPrimitives.reserve(MAX_PRIMITIVES);
    mOrderedPrimitives.reserve(MAX_PRIMITIVES);
}

Bvh4::~Bvh4()
{
    gBvhStats.show();
}

void
Bvh4::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{
    for (uint32_t primId = 0; primId < prims.size(); ++primId)
    {
        Primitive4 prim;
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
Bvh4::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{
    printf("ERROR: Sphere intersection has not been implemented yet.");
}

void
Bvh4::updateScene()
{
    if (mPrimitives.empty())
    {
        return;
    }

    mOrderedPrimitives.resize(mPrimitives.size());

    std::vector<BuildPrimitive4> buildPrimitives(mPrimitives.size());
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

    gBvhStats.countNodes(nodeCount);
}

namespace {
// k-means++
std::vector<float3>
pickInitialCentroids(
    const float3& seed, const std::vector<BuildPrimitive4>& buildPrimitives, uint8_t clusterCount)
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

std::pair<std::vector<std::vector<BuildPrimitive4>>, float /* cost */>
clusterPrimitives(const std::vector<BuildPrimitive4>& buildPrimitives, uint8_t clusterCount)
{
    if (buildPrimitives.size() <= clusterCount)
    {
        return { std::vector<std::vector<BuildPrimitive4>>(), 0.0f };
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
        return { std::vector<std::vector<BuildPrimitive4>>(), 0.0f };
    }

    auto clusterCentroids = pickInitialCentroids(
        centroidBox.getCenter(), buildPrimitives, clusterCount);

    std::vector<std::vector<BuildPrimitive4>> clusters(clusterCount);
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
sortChildNodesByLongestAxis(std::unique_ptr<BuildNode4>& node)
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
        [dim = node->dim](const auto& a, const auto& b) {
            if (!a || !b)
            {
                return a != nullptr;
            }
            else
            {
                return a->bbox.getCenter()[dim] < b->bbox.getCenter()[dim];
            }
        });
}
}

std::unique_ptr<BuildNode4>
Bvh4::buildBvh(
    const std::vector<BuildPrimitive4>& buildPrimitives,
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

    auto [subset, splitCost] = clusterPrimitives(buildPrimitives, CHILD_NODE_COUNT);
    float leafCost = static_cast<float>(buildPrimitives.size()) * nodeBbox.computeSurfaceArea();

    if (subset.empty() || splitCost >= leafCost)
    {
        return createLeaf(buildPrimitives, nodeBbox, offset);
    }

    auto node = std::make_unique<BuildNode4>();
    node->bbox = nodeBbox;

    if (buildPrimitives.size() >= MIN_PRIMITIVES_FOR_PARALLELIZATION)
    {
        std::future<std::unique_ptr<BuildNode4>> children[CHILD_NODE_COUNT];
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

    sortChildNodesByLongestAxis(node);

    return node;
}

std::unique_ptr<BuildNode4>
Bvh4::createLeaf(
    const std::vector<BuildPrimitive4>& buildPrimitives,
    const BoundingBox& bbox,
    std::atomic<uint32_t>& offset)
{
    auto leaf = std::make_unique<BuildNode4>();

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
Bvh4::flatten(const BuildNode4* node, uint32_t& offset)
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
        for (uint8_t i = 0; i < CHILD_NODE_COUNT; ++i)
        {
            const auto& child = node->children[i];

            if (child)
            {
#ifdef USE_BVH_SIMD
                linearNode.bbox4.minX[i] = child->bbox.min.x;
                linearNode.bbox4.minY[i] = child->bbox.min.y;
                linearNode.bbox4.minZ[i] = child->bbox.min.z;

                linearNode.bbox4.maxX[i] = child->bbox.max.x;
                linearNode.bbox4.maxY[i] = child->bbox.max.y;
                linearNode.bbox4.maxZ[i] = child->bbox.max.z;
#else
                linearNode.bbox[i] = child->bbox;
#endif
            }
        }

        linearNode.dim = node->dim;
        flatten(node->children[0].get(), offset);

        for (uint8_t i = 1; i < CHILD_NODE_COUNT; ++i)
        {
            const auto& child = node->children[i];
            if (child)
            {
                linearNode.offset[i - 1] = flatten(node->children[i].get(), offset);
            }
        }
    }

    return nodeOffset;
}

std::optional<Intersection>
Bvh4::intersect(const Ray& ray) const
{
    return traverse(ray, std::numeric_limits<float>::max(), false);
}

bool
Bvh4::intersectAny(const Ray& ray, float tFar) const
{
    const auto intersection = traverse(ray, tFar, true);
    return intersection.has_value();
}

std::optional<Intersection>
Bvh4::traverse(const Ray& ray, float tFar, bool any) const
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
            std::array<bool, CHILD_NODE_COUNT> intersects;
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

            bool isDirNegative = ray.dir[linearNode.dim] < 0.0f;
            for (uint8_t i = 0; i < CHILD_NODE_COUNT; ++i)
            {
                uint8_t idx = !isDirNegative ? CHILD_NODE_COUNT - 1 - i : i;

                if (intersects[idx])
                {
                    assert(stackIdx < STACK_SIZE - 1);

                    if (idx == 0)
                    {
                        stack[stackIdx++] = currentIdx + 1;
                    }
                    else if (linearNode.offset[idx - 1] != LinearNode4::invalid_offset)
                    {
                        stack[stackIdx++] = linearNode.offset[idx - 1];
                    }
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
