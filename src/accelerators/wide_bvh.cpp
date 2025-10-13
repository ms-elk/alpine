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
//#define USE_BREADTH_FIRST

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

        // inner node: child node count
        // leaf: primitive count
        uint16_t childCount = 0;

        uint8_t dim = 0;

        bool isLeaf = false;

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

    void flattenDepthFirst(const BuildNode* node, uint32_t& nodeCount,
        std::pmr::monotonic_buffer_resource* arena);

    void flattenBreadthFirst(const BuildNode* node, uint32_t& nodeCount,
        std::pmr::monotonic_buffer_resource* arena);

    void createLeaf(const BuildNode* node, LinearNode& linearNode);

    std::optional<Intersection> traverse(const Ray& ray, float tFar, bool any) const;

private:
    std::vector<Shape> mShapes;
    std::vector<Primitive> mPrimitives;
    std::vector<Primitive> mOrderedPrimitives;
    std::vector<ispc::TriangleN> mTriangleN;
    std::vector<LinearNode> mLinearNodes;
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

    mLinearNodes.resize(MAX_NODES);
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
#ifdef USE_BREADTH_FIRST
    flattenBreadthFirst(bvh, mNodeCount, &arena);
#else
    flattenDepthFirst(bvh, mNodeCount, &arena);
#endif
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
        linearNode.childCount = node->primitiveCount;
        linearNode.isLeaf = true;
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
        linearNode.isLeaf = false;
        linearNode.childCount = 0;

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
            linearNode.childCount++;
        }

    }

    return nodeOffset;
}

namespace {
std::array<const BuildNode*, SIMD_WIDTH>
collapseNode(const BuildNode* node, uint8_t& childNodeCount)
{
    std::array<const BuildNode*, SIMD_WIDTH> childNodes{};
    childNodeCount = 0;

    childNodes[childNodeCount++] = node->children[0];
    childNodes[childNodeCount++] = node->children[1];

    while (childNodeCount < childNodes.size())
    {
        uint8_t candidateCount = 0;
        float maxSa = 0.0f;
        uint8_t maxNodeIdx = 0;

        for (uint8_t i = 0; i < childNodeCount; ++i)
        {
            const auto* childNode = childNodes[i];

            if (!childNode->isLeaf())
            {
                float sa = childNode->bbox.computeSurfaceArea();
                if (sa > maxSa)
                {
                    maxSa = sa;
                    maxNodeIdx = i;
                }

                candidateCount++;
            }
        }

        if (candidateCount == 0)
        {
            break;
        }

        const auto* collapsedNode = childNodes[maxNodeIdx];
        childNodes[childNodeCount++] = collapsedNode->children[0];
        childNodes[maxNodeIdx] = collapsedNode->children[1];
    }

    return childNodes;
}

template<typename T>
class RingBuffer {
public:
    RingBuffer(std::pmr::monotonic_buffer_resource* arena)
        : mData(arena)
    {
        static constexpr uint32_t RING_BUFFER_SIZE = 128 * 1024;
        mData.resize(RING_BUFFER_SIZE);
    }

    void push_back(const T& value)
    {
        assert(mSize < mData.size());

        uint32_t tail = (mHead + mSize) % mData.size();
        mData[tail] = value;
        mSize++;
    }

    void pop()
    {
        assert(!empty());

        mHead = (mHead + 1) % mData.size();
        mSize--;
    }

    const T& front() const
    {
        assert(!empty());
        return mData[mHead];
    }

    bool empty() const
    {
        return mSize == 0;
    }

private:
    std::pmr::vector<T> mData;
    uint32_t mHead = 0;
    uint32_t mSize = 0;
};
}

void
WideBvh::Impl::flattenDepthFirst(
    const BuildNode* node, uint32_t& nodeCount, std::pmr::monotonic_buffer_resource* arena)
{
    assert(node);

    struct NodeInfo {
        const BuildNode* node;
        uint32_t* offset;
    };

    std::pmr::vector<NodeInfo> stack(arena);
    stack.reserve(STACK_SIZE);
    stack.push_back({ node, nullptr });

    nodeCount = 0;

    while (!stack.empty())
    {
        auto [node, offset] = stack.back();
        stack.pop_back();

        if (offset)
        {
            *offset = nodeCount;
        }

        LinearNode& linearNode = mLinearNodes[nodeCount++];

        if (node->isLeaf())
        {
            createLeaf(node, linearNode);
        }
        else
        {
            linearNode.clearOffset();
            linearNode.dim = node->dim;
            linearNode.isLeaf = false;

            uint8_t childNodeCount = 0;
            auto childNodes = collapseNode(node, childNodeCount);

            linearNode.childCount = childNodeCount;

            for (int8_t i = childNodeCount - 1; i >= 0; i--)
            {
                const auto* childNode = childNodes[i];

#ifdef USE_BVH_SIMD
                linearNode.bboxN.minX[i] = childNode->bbox.min.x;
                linearNode.bboxN.minY[i] = childNode->bbox.min.y;
                linearNode.bboxN.minZ[i] = childNode->bbox.min.z;

                linearNode.bboxN.maxX[i] = childNode->bbox.max.x;
                linearNode.bboxN.maxY[i] = childNode->bbox.max.y;
                linearNode.bboxN.maxZ[i] = childNode->bbox.max.z;
#else
                linearNode.bbox[i] = childNode->bbox;
#endif
                assert(stack.size() < STACK_SIZE);
                uint32_t* offset = i > 0 ? &linearNode.offset[i - 1] : nullptr;
                stack.push_back({ childNode, offset });
            }
        }
    }
}

void
WideBvh::Impl::flattenBreadthFirst(
    const BuildNode* node, uint32_t& nodeCount, std::pmr::monotonic_buffer_resource* arena)
{
    assert(node);

    struct NodeInfo {
        const BuildNode* node;
        uint32_t* offset;
    };

    RingBuffer<NodeInfo> queue(arena);

    queue.push_back({ node, nullptr });

    nodeCount = 0;

    while (!queue.empty())
    {
        auto [node, offset] = queue.front();
        queue.pop();

        if (offset)
        {
            *offset = nodeCount;
        }
        LinearNode& linearNode = mLinearNodes[nodeCount++];

        if (node->isLeaf())
        {
            createLeaf(node, linearNode);
        }
        else
        {
            linearNode.clearOffset();
            linearNode.dim = node->dim;
            linearNode.isLeaf = false;

            uint8_t childNodeCount = 0;
            auto childNodes = collapseNode(node, childNodeCount);

            linearNode.childCount = childNodeCount;

            for (uint8_t i = 0; i < childNodeCount; ++i)
            {
                const auto* childNode = childNodes[i];

#ifdef USE_BVH_SIMD
                linearNode.bboxN.minX[i] = childNode->bbox.min.x;
                linearNode.bboxN.minY[i] = childNode->bbox.min.y;
                linearNode.bboxN.minZ[i] = childNode->bbox.min.z;

                linearNode.bboxN.maxX[i] = childNode->bbox.max.x;
                linearNode.bboxN.maxY[i] = childNode->bbox.max.y;
                linearNode.bboxN.maxZ[i] = childNode->bbox.max.z;
#else
                linearNode.bbox[i] = childNode->bbox;
#endif
                uint32_t* offset = i == 0 ? &linearNode.offset[0] : nullptr;
                queue.push_back({ childNode, offset });
            }
        }
    }
}

void
WideBvh::Impl::createLeaf(const BuildNode* node, LinearNode& linearNode)
{
    assert(node->isLeaf());

    linearNode.offset[0] = node->offset;
    linearNode.childCount = node->primitiveCount;
    linearNode.isLeaf = true;

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

    std::array<uint32_t, STACK_SIZE> stack {};
    uint8_t stackIdx = 0;
    stack[stackIdx++] = 0;

    while (stackIdx != 0)
    {
        counter.incrementNode();

        uint32_t currentIdx = stack[--stackIdx];
        assert(currentIdx < mNodeCount);
        const auto& linearNode = mLinearNodes[currentIdx];

        if (linearNode.isLeaf)
        {
#ifdef USE_TRIANGLE_SIMD
            uint16_t triGroupCount = (linearNode.childCount - 1) / SIMD_WIDTH + 1;
            for (uint16_t i = 0; i < triGroupCount; ++i)
            {
                counter.incrementTriangle();

                const auto& triN = mTriangleN[linearNode.offset[1] + i];

                ispc::IntersectionN isectN;
                ispc::intersectTriangleN(isectN, &ray.org[0], &ray.dir[0], triN);

                for (uint8_t j = 0; j < SIMD_WIDTH; ++j)
                {
                    uint16_t idx = i * SIMD_WIDTH + j;
                    if (idx >= linearNode.childCount)
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
                        return Intersection {};
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
            for (uint16_t i = 0; i < linearNode.childCount; ++i)
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

            for (uint8_t i = 0; i < linearNode.childCount; ++i)
            {
                if (!intersects[i])
                {
                    continue;
                }

                assert(stackIdx < STACK_SIZE - 1);

#ifdef USE_BREADTH_FIRST
                stack[stackIdx] = linearNode.offset[0] + i;
#else
                stack[stackIdx] = i == 0 ? currentIdx + 1 : linearNode.offset[i - 1];
#endif
                assert(stack[stackIdx] != LinearNode::INVALID_OFFSET);

                stackIdx++;
            }
        }
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
