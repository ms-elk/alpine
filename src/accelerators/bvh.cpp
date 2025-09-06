#include "bvh.h"

#include "bvh_common.h"

#include <utils/bounding_box.h>
#include <alpine_config.h>
#include <intersection.h>
#include <ray.h>

#include <array>
#include <assert.h>

namespace {
constexpr uint8_t CHILD_NODE_COUNT = 2;

alpine::BvhStats gBvhStats;
}

namespace alpine {
class Bvh::Impl
{
public:
    Impl(std::span<std::byte> memoryArenaBuffer);
    ~Impl();

    uint32_t appendMesh(
        const std::vector<float3>& vertices,
        const std::vector<uint3>& prims,
        const void* ptr);

    void appendSphere(const std::vector<float4>& vertices, const void* ptr);

    inline void* getVertexBuffer(uint32_t shapeId) { return mShapes[shapeId].vertices.data(); }

    void updateShape(uint32_t shapeId);

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
        uint32_t offset = 0;
        uint16_t primitiveCount = 0;
        uint8_t dim = 0;

        bool isLeaf() const { return primitiveCount > 0; }
    };

    std::vector<Shape> mShapes;
    std::vector<Primitive> mPrimitives;
    std::vector<Primitive> mOrderedPrimitives;
    std::array<LinearNode, MAX_NODES> mLinearNodes;
    uint32_t mNodeCount = 0;

    std::span<std::byte> mMemoryArenaBuffer;
};

Bvh::Impl::Impl(std::span<std::byte> memoryArenaBuffer)
    : mMemoryArenaBuffer(memoryArenaBuffer)
{
    mShapes.reserve(MAX_SHAPES);
    mPrimitives.reserve(MAX_PRIMITIVES);
    mOrderedPrimitives.reserve(MAX_PRIMITIVES);
}

Bvh::Impl::~Impl()
{
    gBvhStats.show();
}

uint32_t
Bvh::Impl::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{
    assert(mShapes.size() < MAX_SHAPES);
    uint32_t shapeId = mShapes.size();
    mShapes.emplace_back(vertices, prims, static_cast<uint32_t>(mPrimitives.size()));
    const auto& shape = mShapes.back();

    for (uint32_t primId = 0; primId < prims.size(); ++primId)
    {
        Primitive prim;
        prim.ptr = ptr;
        prim.primId = primId;
        prim.updateVertices(shape);

        assert(mPrimitives.size() < MAX_PRIMITIVES);
        mPrimitives.push_back(prim);
    }

    return shapeId;
}

void
Bvh::Impl::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{
    printf("ERROR: Sphere intersection has not been implemented yet.");
}

void
Bvh::Impl::updateShape(uint32_t shapeId)
{
    const auto& shape = mShapes[shapeId];

    for (uint32_t primId = 0; primId < shape.prims.size(); ++primId)
    {
        auto& prim = mPrimitives[primId + shape.primOffset];
        prim.updateVertices(shape);
    }
}

void
Bvh::Impl::updateScene()
{
    if (mPrimitives.empty())
    {
        return;
    }

    std::pmr::monotonic_buffer_resource arena(
        mMemoryArenaBuffer.data(), mMemoryArenaBuffer.size(), nullptr);
    auto* bvh = buildBvh(mPrimitives, mOrderedPrimitives, &arena);

    mNodeCount = 0;
    flatten(bvh, mNodeCount);

    gBvhStats.countNodes(mNodeCount);
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
        flatten(node->children[0], offset);
        linearNode.offset = flatten(node->children[1], offset);
        linearNode.primitiveCount = 0;
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

Bvh::Bvh(std::span<std::byte> memoryArenaBuffer)
    : mPimpl(std::make_unique<Impl>(memoryArenaBuffer))
{}

Bvh::~Bvh() = default;

uint32_t
Bvh::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{
    return mPimpl->appendMesh(vertices, prims, ptr);
}

void
Bvh::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{
    mPimpl->appendSphere(vertices, ptr);
}

void*
Bvh::getVertexBuffer(uint32_t shapeId)
{
    return mPimpl->getVertexBuffer(shapeId);
}

void
Bvh::updateShape(uint32_t shapeId)
{
    mPimpl->updateShape(shapeId);
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
