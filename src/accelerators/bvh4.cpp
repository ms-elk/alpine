﻿#include "bvh4.h"

#include "bvh_common.h"

#include <ispc/bounding_box.h>
#include <utils/bounding_box.h>
#include <intersection.h>
#include <ray.h>

#include <array>
#include <assert.h>

// TODO: 
// - triangle intersection tests using SIMD

namespace {
static constexpr uint8_t CHILD_NODE_COUNT = 4;

alpine::BvhStats gBvhStats;

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

namespace alpine {
class Bvh4::Impl
{
public:
    Impl();
    ~Impl();

    uint32_t appendMesh(
        const std::vector<float3>& vertices,
        const std::vector<uint3>& prims,
        const void* ptr);

    void appendSphere(const std::vector<float4>& vertices, const void* ptr);

    void updateScene();

    std::optional<Intersection> intersect(const Ray& ray) const;

    bool intersectAny(const Ray& ray, float tFar) const;

private:
    struct alignas(32) LinearNode
    {
#ifdef USE_BVH_SIMD
        ispc::BoundingBox4 bbox4;
#else
        BoundingBox bbox[4];
#endif

        static constexpr uint32_t INVALID_OFFSET = std::numeric_limits<uint32_t>::max();
        std::array<uint32_t, CHILD_NODE_COUNT - 1> offset;

        uint16_t primitiveCount = 0;
        std::array<uint8_t, 3> dim{};

        bool isLeaf() const { return primitiveCount > 0; }

        LinearNode()
        {
            std::fill(offset.begin(), offset.end(), INVALID_OFFSET);

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

    uint32_t flatten(const BuildNode* node, uint32_t& offset);

    std::optional<Intersection> traverse(const Ray& ray, float tFar, bool any) const;

private:
    std::vector<Primitive> mPrimitives;
    std::vector<Primitive> mOrderedPrimitives;
    std::array<LinearNode, MAX_NODES> mLinearNodes;
    uint32_t mNodeCounts = 0;
};

Bvh4::Impl::Impl()
{
    mPrimitives.reserve(MAX_PRIMITIVES);
}

Bvh4::Impl::~Impl()
{
    gBvhStats.show();
}

uint32_t
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

        assert(mPrimitives.size() < MAX_PRIMITIVES);
        mPrimitives.push_back(prim);
    }

    return 0; // TODO
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

    auto bvh = buildBvh(mPrimitives);
    mOrderedPrimitives = std::move(bvh.orderedPrimitives);

    mNodeCounts = 0;
    flatten(bvh.root.get(), mNodeCounts);

    gBvhStats.countNodes(mNodeCounts);
}

uint32_t
Bvh4::Impl::flatten(const BuildNode* node, uint32_t& offset)
{
    assert(node);
    assert(offset < MAX_NODES);

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
#ifdef USE_BVH_SIMD
            linearNode.bbox4.minX[idx] = bbox.min.x;
            linearNode.bbox4.minY[idx] = bbox.min.y;
            linearNode.bbox4.minZ[idx] = bbox.min.z;

            linearNode.bbox4.maxX[idx] = bbox.max.x;
            linearNode.bbox4.maxY[idx] = bbox.max.y;
            linearNode.bbox4.maxZ[idx] = bbox.max.z;
#else
            linearNode.bbox[idx] = bbox;
#endif
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
                setBbox(idx, child->bbox);
            }
            else
            {
                for (uint8_t j = 0; j < child->children.size(); ++j)
                {
                    const auto& grandchild = child->children[j];
                    assert(grandchild);

                    uint8_t idx = 2 * i + j;
                    flattenChild(idx, grandchild.get());
                    setBbox(idx, grandchild->bbox);
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

        assert(currentIdx < mNodeCounts);
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
            ispc::intersectBoundingBox4(intersects.data(),
                &ray.org[0], &ray.dir[0], &invRayDir[0], tNear, linearNode.bbox4);
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
                    if (offset != LinearNode::INVALID_OFFSET)
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

uint32_t
Bvh4::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{
    return mPimpl->appendMesh(vertices, prims, ptr);
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
