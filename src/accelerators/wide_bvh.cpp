#include "wide_bvh.h"

#include "bvh_common.h"
#include "wide_bvh_builder.h"

#include <ispc/bounding_box.h>
#include <ispc/ispc_config.h>
#include <ispc/triangle.h>
#include <utils/bounding_box.h>
#include <alpine_config.h>
#include <intersection.h>
#include <ray.h>

#include <array>
#include <assert.h>
#include <bit>

#define USE_BVH_SIMD
#define USE_TRIANGLE_SIMD
#define USE_BINARY_TO_WIDE

namespace {
constexpr uint8_t LEAF_THRESHOLD = 2 * SIMD_WIDTH;

alpine::BvhStats gBvhStats;
}

namespace alpine {
class WideBvh::Impl
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
    struct alignas(32) LinearNode
    {
#ifdef USE_BVH_SIMD
        ispc::BoundingBoxN bboxN;
#else
        BoundingBox bbox[SIMD_WIDTH];
#endif

        static constexpr uint32_t INVALID_OFFSET = std::numeric_limits<uint32_t>::max();

        // offset for leaf
        // 0: index of mOrderedPrimitives
        // 1: index of mTriangles
        std::array<uint32_t, SIMD_WIDTH - 1> offset;

        uint16_t primitiveCount = 0;
        uint8_t dim = 0;

        bool isLeaf() const { return primitiveCount > 0; }

        void clearOffset()
        {
            std::fill(offset.begin(), offset.end(), INVALID_OFFSET);
        }

        LinearNode()
        {
            clearOffset();

#ifdef USE_BVH_SIMD
            for (uint8_t i = 0; i < SIMD_WIDTH; ++i)
            {
                bboxN.minX[i] = std::numeric_limits<float>::max();
                bboxN.minY[i] = std::numeric_limits<float>::max();
                bboxN.minZ[i] = std::numeric_limits<float>::max();

                bboxN.maxX[i] = -std::numeric_limits<float>::max();
                bboxN.maxY[i] = -std::numeric_limits<float>::max();
                bboxN.maxZ[i] = -std::numeric_limits<float>::max();
            }
#endif
        }
    };

    uint32_t flattenWide(const BuildWideNode* node, uint32_t& offset);

    uint32_t flatten(const BuildNode* node, uint32_t& offset);

    void createLeaf(const BuildNode* node, LinearNode& linearNode);

    void createWideNode(const BuildNode* node, LinearNode& linearNode, uint32_t& offset);

    std::optional<Intersection> traverse(const Ray& ray, float tFar, bool any) const;

private:
    std::vector<Shape> mShapes;
    std::vector<Primitive> mPrimitives;
    std::vector<Primitive> mOrderedPrimitives;
    std::vector<ispc::TriangleN> mTriangleN;
    std::array<LinearNode, MAX_NODES> mLinearNodes;
    uint32_t mNodeCount = 0;

    std::span<std::byte> mMemoryArenaBuffer;
};

WideBvh::Impl::Impl(std::span<std::byte> memoryArenaBuffer)
    : mMemoryArenaBuffer(memoryArenaBuffer)
{
    mShapes.reserve(MAX_SHAPES);
    mPrimitives.reserve(MAX_PRIMITIVES);
    mOrderedPrimitives.reserve(MAX_PRIMITIVES);
    mTriangleN.reserve(MAX_PRIMITIVES);
}

WideBvh::Impl::~Impl()
{
    gBvhStats.show();
}

uint32_t
WideBvh::Impl::appendMesh(
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
WideBvh::Impl::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{
    printf("ERROR: Sphere intersection has not been implemented yet.");
}

void
WideBvh::Impl::updateShape(uint32_t shapeId)
{
    const auto& shape = mShapes[shapeId];

    for (uint32_t primId = 0; primId < shape.prims.size(); ++primId)
    {
        auto& prim = mPrimitives[primId + shape.primOffset];
        prim.updateVertices(shape);
    }
}

void
WideBvh::Impl::updateScene()
{
    if (mPrimitives.empty())
    {
        return;
    }

    std::pmr::monotonic_buffer_resource arena(
        mMemoryArenaBuffer.data(), mMemoryArenaBuffer.size(), nullptr);

    mTriangleN.clear();
    mNodeCount = 0;

#ifdef USE_BINARY_TO_WIDE
    auto* bvh = buildBvh(mPrimitives, mOrderedPrimitives, LEAF_THRESHOLD, &arena);
    flatten(bvh, mNodeCount);
#else
    auto* bvh = buildWideBvh(mPrimitives, mOrderedPrimitives, LEAF_THRESHOLD, &arena);
    flattenWide(bvh, mNodeCount);
#endif

    gBvhStats.countNodes(mNodeCount);
}

uint32_t
WideBvh::Impl::flattenWide(const BuildWideNode* node, uint32_t& offset)
{
    assert(node);
    assert(offset < MAX_NODES);

    auto& linearNode = mLinearNodes[offset];
    uint32_t nodeOffset = offset++;

    if (node->isLeaf())
    {
        assert(node->isLeaf());

        linearNode.offset[0] = node->offset;
        linearNode.primitiveCount = node->primitiveCount;
#ifdef USE_TRIANGLE_SIMD
        linearNode.offset[1] = mTriangleN.size();

        uint16_t triGroupCount = (node->primitiveCount - 1) / SIMD_WIDTH + 1;
        for (uint16_t i = 0; i < triGroupCount; ++i)
        {
            ispc::TriangleN triN;
            for (uint8_t j = 0; j < SIMD_WIDTH; ++j)
            {
                uint16_t idx = i * SIMD_WIDTH + j;
                if (idx >= node->primitiveCount)
                {
                    break;
                }

                const auto& prim = mOrderedPrimitives[node->offset + idx];

                triN.vx[j] = prim.vertex.x;
                triN.vy[j] = prim.vertex.y;
                triN.vz[j] = prim.vertex.z;

                triN.ex0[j] = prim.edges[0].x;
                triN.ey0[j] = prim.edges[0].y;
                triN.ez0[j] = prim.edges[0].z;

                triN.ex1[j] = prim.edges[1].x;
                triN.ey1[j] = prim.edges[1].y;
                triN.ez1[j] = prim.edges[1].z;
            }

            mTriangleN.emplace_back(triN);
        }
#endif
    }
    else
    {
        linearNode.clearOffset();
        linearNode.dim = node->dim;
        linearNode.primitiveCount = 0;
        for (uint8_t i = 0; i < node->children.size(); ++i)
        {
            const auto* child = node->children[i];
            if (!child)
            {
                continue;
            }

            uint32_t childNodeOffset = flattenWide(child, offset);
            if (i > 0)
            {
                linearNode.offset[i - 1] = childNodeOffset;
            }

#ifdef USE_BVH_SIMD
            linearNode.bboxN.minX[i] = child->bbox.min.x;
            linearNode.bboxN.minY[i] = child->bbox.min.y;
            linearNode.bboxN.minZ[i] = child->bbox.min.z;

            linearNode.bboxN.maxX[i] = child->bbox.max.x;
            linearNode.bboxN.maxY[i] = child->bbox.max.y;
            linearNode.bboxN.maxZ[i] = child->bbox.max.z;
#else
            linearNode.bbox[i] = child->bbox;
#endif
        }

    }

    return nodeOffset;
}

uint32_t
WideBvh::Impl::flatten(const BuildNode* node, uint32_t& offset)
{
    assert(node);
    assert(offset < MAX_NODES);

    auto& linearNode = mLinearNodes[offset];
    uint32_t nodeOffset = offset++;

    if (node->isLeaf())
    {
        createLeaf(node, linearNode);
    }
    else
    {
        createWideNode(node, linearNode, offset);
    }

    return nodeOffset;
}

void
WideBvh::Impl::createLeaf(const BuildNode* node, LinearNode& linearNode)
{
    assert(node->isLeaf());

    linearNode.offset[0] = node->offset;
    linearNode.primitiveCount = node->primitiveCount;
#ifdef USE_TRIANGLE_SIMD
    linearNode.offset[1] = mTriangleN.size();

    uint16_t triGroupCount = (node->primitiveCount - 1) / SIMD_WIDTH + 1;
    for (uint16_t i = 0; i < triGroupCount; ++i)
    {
        ispc::TriangleN triN;
        for (uint8_t j = 0; j < SIMD_WIDTH; ++j)
        {
            uint16_t idx = i * SIMD_WIDTH + j;
            if (idx >= node->primitiveCount)
            {
                break;
            }

            const auto& prim = mOrderedPrimitives[node->offset + idx];

            triN.vx[j] = prim.vertex.x;
            triN.vy[j] = prim.vertex.y;
            triN.vz[j] = prim.vertex.z;

            triN.ex0[j] = prim.edges[0].x;
            triN.ey0[j] = prim.edges[0].y;
            triN.ez0[j] = prim.edges[0].z;

            triN.ex1[j] = prim.edges[1].x;
            triN.ey1[j] = prim.edges[1].y;
            triN.ez1[j] = prim.edges[1].z;
        }

        mTriangleN.emplace_back(triN);
    }
#endif
}

void
WideBvh::Impl::createWideNode(const BuildNode* node, LinearNode& linearNode, uint32_t& offset)
{
    assert(!node->isLeaf());

    static constexpr uint8_t MAX_NODE_DEPTH = std::bit_width(static_cast<uint8_t>(SIMD_WIDTH)) - 2;

    linearNode.clearOffset();
    linearNode.dim = node->dim;
    linearNode.primitiveCount = 0;

    struct NodeInfo {
        const BuildNode* node;
        uint8_t nodeOffset;
        uint8_t depth;
    };

    std::array<NodeInfo, STACK_SIZE> stack;
    uint8_t stackIdx = 0;

    stack[stackIdx++] = { node->children[1], 1, 0 };
    stack[stackIdx++] = { node->children[0], 0, 0 };

    while (stackIdx != 0)
    {
        NodeInfo info = stack[--stackIdx];

        if (info.node->isLeaf() || info.depth == MAX_NODE_DEPTH)
        {
            uint32_t childOffset = flatten(info.node, offset);

            // map index depending on depth
            uint8_t idx = (1 << (MAX_NODE_DEPTH - info.depth)) * info.nodeOffset;
            if (idx > 0)
            {
                linearNode.offset[idx - 1] = childOffset;
            }

#ifdef USE_BVH_SIMD
            linearNode.bboxN.minX[idx] = info.node->bbox.min.x;
            linearNode.bboxN.minY[idx] = info.node->bbox.min.y;
            linearNode.bboxN.minZ[idx] = info.node->bbox.min.z;

            linearNode.bboxN.maxX[idx] = info.node->bbox.max.x;
            linearNode.bboxN.maxY[idx] = info.node->bbox.max.y;
            linearNode.bboxN.maxZ[idx] = info.node->bbox.max.z;
#else
            linearNode.bbox[idx] = info.node->bbox;
#endif
        }
        else
        {
            assert(stackIdx < STACK_SIZE - 2);

            stack[stackIdx++] = {
                info.node->children[1],
                static_cast<uint8_t>(2 * info.nodeOffset + 1),
                static_cast<uint8_t>(info.depth + 1)
            };
            stack[stackIdx++] = {
                info.node->children[0],
                static_cast<uint8_t>(2 * info.nodeOffset),
                static_cast<uint8_t>(info.depth + 1)
            };
        }
    }
}

std::optional<Intersection>
WideBvh::Impl::intersect(const Ray& ray) const
{
    return traverse(ray, std::numeric_limits<float>::max(), false);
}

bool
WideBvh::Impl::intersectAny(const Ray& ray, float tFar) const
{
    const auto intersection = traverse(ray, tFar, true);
    return intersection.has_value();
}

std::optional<Intersection>
WideBvh::Impl::traverse(const Ray& ray, float tFar, bool any) const
{
    NodeAccessCounter counter(gBvhStats, any);

    std::optional<Intersection> closestIsect;
    float tNear = tFar;
    float3 invRayDir = float3(1.0f / ray.dir.x, 1.0f / ray.dir.y, 1.0f / ray.dir.z);

    std::array<uint32_t, STACK_SIZE> stack;
    uint8_t stackIdx = 0;
    uint32_t currentIdx = 0;

    while (true)
    {
        counter.incrementNode();

        assert(currentIdx < mNodeCount);
        const auto& linearNode = mLinearNodes[currentIdx];
        if (linearNode.isLeaf())
        {
#ifdef USE_TRIANGLE_SIMD
            uint16_t triGroupCount = (linearNode.primitiveCount - 1) / SIMD_WIDTH + 1;
            for (uint16_t i = 0; i < triGroupCount; ++i)
            {
                counter.incrementTriangle();

                const auto& triN = mTriangleN[linearNode.offset[1] + i];

                ispc::IntersectionN isectN;
                ispc::intersectTriangleN(isectN, &ray.org[0], &ray.dir[0], triN);

                for (uint8_t j = 0; j < SIMD_WIDTH; ++j)
                {
                    uint16_t idx = i * SIMD_WIDTH + j;
                    if (idx >= linearNode.primitiveCount)
                    {
                        break;
                    }

                    float t = isectN.t[j];
                    if (t == NO_INTERSECT || t >= tNear)
                    {
                        continue;
                    }

                    if (any)
                    {
                        Intersection isect;
                        return isect;
                    }
                    else
                    {
                        const auto& prim = mOrderedPrimitives[linearNode.offset[0] + idx];

                        Intersection isect;
                        isect.shapePtr = prim.ptr;
                        isect.primId = prim.primId;
                        isect.ng = prim.ng;
                        isect.t = t;
                        isect.barycentric = float2(isectN.b0[j], isectN.b1[j]);

                        closestIsect = isect;
                        tNear = t;
                    }
                }
            }

#else
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
#endif
        }
        else
        {
            std::array<bool, SIMD_WIDTH> intersects{};
#ifdef USE_BVH_SIMD
            ispc::intersectBoundingBoxN(intersects.data(),
                &ray.org[0], &ray.dir[0], &invRayDir[0], tNear, linearNode.bboxN);
#else
            for (uint8_t i = 0; i < SIMD_WIDTH; ++i)
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
                    if (offset != LinearNode::INVALID_OFFSET)
                    {
                        stack[stackIdx++] = offset;
                    }
                }
            };

            bool isDirNegative = ray.dir[linearNode.dim] < 0.0f;
            for (uint8_t i = 0; i < SIMD_WIDTH; ++i)
            {
                uint8_t idx = !isDirNegative ? SIMD_WIDTH - 1 - i : i;
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

WideBvh::WideBvh(std::span<std::byte> memoryArenaBuffer)
    : mPimpl(std::make_unique<Impl>(memoryArenaBuffer))
{}

WideBvh::~WideBvh() = default;

uint32_t
WideBvh::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{
    return mPimpl->appendMesh(vertices, prims, ptr);
}

void
WideBvh::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{
    mPimpl->appendSphere(vertices, ptr);
}

void*
WideBvh::getVertexBuffer(uint32_t shapeId)
{
    return mPimpl->getVertexBuffer(shapeId);
}

void
WideBvh::updateShape(uint32_t shapeId)
{
    mPimpl->updateShape(shapeId);
}

void
WideBvh::updateScene()
{
    mPimpl->updateScene();
}

std::optional<Intersection>
WideBvh::intersect(const Ray& ray) const
{
    return mPimpl->intersect(ray);
}

bool
WideBvh::intersectAny(const Ray& ray, float tFar) const
{
    return mPimpl->intersectAny(ray, tFar);
}
}
