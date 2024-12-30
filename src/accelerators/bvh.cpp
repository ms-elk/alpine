#include "bvh.h"

#include "ray.h"
#include "utils/bounding_box.h"
#include "utils/bvh_util.h"

#include <array>
#include <assert.h>
#include <future>
#include <optional>

namespace alpine {
namespace {
static constexpr uint32_t MAX_PRIMITIVES = 1024 * 1024;
static constexpr uint8_t CHILD_NODE_COUNT = 2;

static constexpr uint8_t SPLITS_PER_DIM = 4;
static constexpr uint8_t BIN_COUNT = SPLITS_PER_DIM + 1;

static constexpr uint32_t MIN_PRIMITIVES_FOR_PARALLELIZATION = 1024;
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

struct Node
{
    BoundingBox bbox;
    std::array<std::unique_ptr<Node>, CHILD_NODE_COUNT> children = { nullptr, nullptr };
    Primitive* leaf = nullptr;

    std::optional<Intersection> intersect(const Ray& ray) const
    {
        if (!bbox.intersect(ray))
        {
            return {};
        }

        if (leaf)
        {
            return leaf->intersect(ray);
        }

        Intersection closestIsect;
        float tNear = std::numeric_limits<float>::max();
        for (const auto& node : children)
        {
            assert(node);

            auto isect = node->intersect(ray);
            if (isect.has_value() && isect.value().t < tNear)
            {
                closestIsect = isect.value();
                tNear = isect.value().t;
            }
        }

        return closestIsect;
    }

    bool occluded(const Ray& ray, float far) const
    {
        if (leaf)
        {
            auto isect = leaf->intersect(ray);
            return isect.has_value();
        }

        for (auto& node : children)
        {
            assert(node);
            auto isect = node->intersect(ray);
            if (isect.has_value() && isect.value().t < far)
            {
                return true;
            }
        }

        return false;
    }
};

namespace {
std::optional<bvh_util::Split>
findSplit(const std::vector<Primitive*>& primitives, const float3& diagonal)
{
    if (primitives.size() <= 2)
    {
        return {};
    }

    auto centroidBox = [&]() {
        BoundingBox cb;
        for (const auto* prim : primitives)
        {
            float3 c = prim->bbox.getCenter();
            cb = merge(cb, BoundingBox{ c, c });
        }

        return cb;
    }();

    float cbDiagonalMax = maxComponent(centroidBox.getDiagonal());
    if (cbDiagonalMax == 0.0f)
    {
        return {};
    }

    bvh_util::Split split;
    float minCost = std::numeric_limits<float>::max();

    for (uint8_t dim = 0; dim < 3; ++dim)
    {
        if (centroidBox.min[dim] == centroidBox.max[dim])
        {
            continue;
        }

        float kr = maxComponent(diagonal) / std::max(diagonal[dim], 0.001f);

        BoundingBox binBoxes[BIN_COUNT];

        // binning
        for (const auto* prim : primitives)
        {
            float c = prim->bbox.getCenter()[dim];
            uint8_t binIdx = bvh_util::getBinIndex(c, centroidBox.min[dim], centroidBox.max[dim], BIN_COUNT);
            binIdx = std::min(binIdx, static_cast<uint8_t>(BIN_COUNT - 1));

            auto& binBox = binBoxes[binIdx];
            binBox = merge(binBox, prim->bbox);
        }

        float costs[SPLITS_PER_DIM] = { 0.0f };

        const auto accumulateCosts = [&](uint8_t binIdx, uint8_t splitIdx, BoundingBox& binAccum) {
            const auto& binBox = binBoxes[binIdx];
            binAccum = merge(binAccum, binBox);

            float sa =
                binAccum.min.x < std::numeric_limits<float>::max() /* is initialized or not */
                ? binAccum.computeSurfaceArea() : 0.0f;
            costs[splitIdx] += sa;
        };

        BoundingBox binBelow;
        for (uint8_t splitIdx = 0; splitIdx < SPLITS_PER_DIM; ++splitIdx)
        {
            uint8_t binIdx = splitIdx;
            accumulateCosts(binIdx, splitIdx, binBelow);
        }

        BoundingBox binAbove;
        for (int8_t splitIdx = SPLITS_PER_DIM - 1; splitIdx >= 0; --splitIdx)
        {
            uint8_t binIdx = splitIdx + 1;
            accumulateCosts(binIdx, splitIdx, binAbove);
        }

        // find the split which has the minimum cost
        for (uint8_t splitIdx = 0; splitIdx < SPLITS_PER_DIM; ++splitIdx)
        {
            auto& cost = costs[splitIdx];
            cost *= kr;
            if (cost < minCost)
            {
                minCost = cost;
                split = { dim, splitIdx, BIN_COUNT, centroidBox.min[dim], centroidBox.max[dim] };
            }
        }
    }

    return split;
}

std::unique_ptr<Node>
buildBvh(const std::vector<Primitive*>& primitives)
{
    auto node = std::make_unique<Node>();
    for (const auto* prim : primitives)
    {
        node->bbox = merge(node->bbox, prim->bbox);
    }

    if (primitives.size() == 1)
    {
        node->leaf = primitives[0];
    }
    else
    {
        auto split = findSplit(primitives, node->bbox.getDiagonal());

        std::vector<Primitive*> subset[CHILD_NODE_COUNT];
        subset[0].reserve(primitives.size());
        subset[1].reserve(primitives.size());

        if (split.has_value())
        {
            const auto& s = split.value();
            for (auto* prim : primitives)
            {
                uint8_t subsetIdx = s.isBelow(prim->bbox.getCenter()) ? 0 : 1;
                subset[subsetIdx].push_back(prim);
            }
        }
        else
        {
            // split by the middle index
            uint32_t midIdx = static_cast<uint32_t>(primitives.size() / 2);
            for (uint32_t i = 0; i < primitives.size(); ++i)
            {
                uint8_t subsetIdx = i < midIdx ? 0 : 1;
                subset[subsetIdx].push_back(primitives[i]);
            }
        }

        assert(!subset[0].empty());
        assert(!subset[1].empty());

        if (primitives.size() >= MIN_PRIMITIVES_FOR_PARALLELIZATION)
        {
            std::future<std::unique_ptr<Node>> children[2];
            for (uint8_t i = 0; i < CHILD_NODE_COUNT; ++i)
            {
                children[i] = std::async([&, i]() {
                    return buildBvh(subset[i]);
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
                node->children[i] = buildBvh(subset[i]);
            }
        }
    }

    return node;
}
}

Bvh::Bvh()
{
    mPrimitives.reserve(MAX_PRIMITIVES);
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
{}

void
Bvh::updateScene()
{
    if (mPrimitives.empty())
    {
        return;
    }

    std::vector<Primitive*> primitives(mPrimitives.size());
    for (uint32_t i = 0; i < mPrimitives.size(); ++i)
    {
        primitives[i] = &mPrimitives[i];
    }

    mBvh = buildBvh(primitives);
}

Intersection
Bvh::intersect(const Ray& ray) const
{
    assert(mBvh);
    auto isect = mBvh->intersect(ray);
    return isect.has_value() ? isect.value() : Intersection();
}

bool
Bvh::occluded(const Ray& ray, float far) const
{
    assert(mBvh);
    return mBvh->occluded(ray, far);
}
}
