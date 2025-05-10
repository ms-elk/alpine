#include "bvh.h"

#include "bvh_common.h"

#include <utils/bounding_box.h>
#include <intersection.h>
#include <ray.h>

#include <array>
#include <assert.h>

namespace {
static constexpr uint8_t CHILD_NODE_COUNT = 2;
static constexpr uint8_t STACK_SIZE = 64;

alpine::BvhStats gBvhStats;
}

namespace alpine {
class Bvh::Impl
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
    uint32_t flatten(const BuildNode* node, uint32_t& offset);

    std::optional<Intersection> traverse(const Ray& ray, float tFar, bool any) const;

private:
    struct alignas(32) LinearNode
    {
        BoundingBox bbox;
        uint32_t offset;
        uint16_t primitiveCount = 0;
        uint8_t dim = 0;

        bool isLeaf() const { return primitiveCount > 0; }
    };

    std::vector<Primitive> mPrimitives;
    std::vector<Primitive> mOrderedPrimitives;
    std::array<LinearNode, MAX_NODES> mLinearNodes;
    uint32_t mNodeCounts = 0;
};

Bvh::Impl::Impl()
{
    mPrimitives.reserve(MAX_PRIMITIVES);
}

Bvh::Impl::~Impl()
{
    gBvhStats.show();
}

void
Bvh::Impl::appendMesh(
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

        assert(mPrimitives.size() < MAX_PRIMITIVES);
        mPrimitives.push_back(prim);
    }
}

void
Bvh::Impl::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{
    printf("ERROR: Sphere intersection has not been implemented yet.");
}

void
Bvh::Impl::updateScene()
{
    if (mPrimitives.empty())
    {
        return;
    }

    auto bvh = buildBvh(mPrimitives);
    mOrderedPrimitives = std::move(bvh.orderedPrimitives);

    mNodeCounts = 0;
    flatten(bvh.root.get(), mNodeCounts);

    gBvhStats.countNodes(mNodeCounts);
}

uint32_t
Bvh::Impl::flatten(const BuildNode* node, uint32_t& offset)
{
    assert(node);
    assert(offset < MAX_NODES);

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
Bvh::Impl::intersect(const Ray& ray) const
{
    return traverse(ray, std::numeric_limits<float>::max(), false);
}

bool
Bvh::Impl::intersectAny(const Ray& ray, float tFar) const
{
    const auto intersection = traverse(ray, tFar, true);
    return intersection.has_value();
}

std::optional<Intersection>
Bvh::Impl::traverse(const Ray& ray, float tFar, bool any) const
{
    assert(!mLinearNodes.empty());

    NodeAccessCounter counter(gBvhStats, any);

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
                counter.incrementTriangle();

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

Bvh::Bvh()
    : mPimpl(std::make_unique<Impl>())
{}

Bvh::~Bvh() = default;

void
Bvh::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{
    mPimpl->appendMesh(vertices, prims, ptr);
}

void
Bvh::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{
    mPimpl->appendSphere(vertices, ptr);
}

void
Bvh::updateScene()
{
    mPimpl->updateScene();
}

std::optional<Intersection>
Bvh::intersect(const Ray& ray) const
{
    return mPimpl->intersect(ray);
}

bool
Bvh::intersectAny(const Ray& ray, float tFar) const
{
    return mPimpl->intersectAny(ray, tFar);
}
}
