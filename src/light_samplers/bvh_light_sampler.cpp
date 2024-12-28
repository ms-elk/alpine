#include "bvh_light_sampler.h"

#include "lights/light.h"
#include "utils/bounding_box.h"
#include "utils/bvh_util.h"
#include "utils/util.h"

#include <assert.h>
#include <future>
#include <optional>

namespace alpine {
namespace {
static constexpr uint8_t SPLITS_PER_DIM = 4;
static constexpr uint32_t MIN_LIGHTS_FOR_PARALLELIZATION = 512;

struct LightNode
{
    float power = 0.0f;
    BoundingBox bbox;
    std::unique_ptr<LightNode> children[2] = { nullptr, nullptr };
    Light* light = nullptr;

    float importance(const float3& p, const float3& n) const
    {
        float3 wiWorld = bbox.getCenter() - p;
        float distance2 = dot(wiWorld, wiWorld);
        float importance = power / std::max(distance2, 0.5f * length(bbox.getDiagonal()));
        
        // importance between normal and direction to bounding box
        float boundRadius = 0.5f * length(bbox.getDiagonal());
        float boundRadius2 = boundRadius * boundRadius;
        if (distance2 > 0.0f && distance2 > boundRadius2)
        {
            float sinB2 = boundRadius2 / distance2;
            float sinB = std::sqrt(sinB2);
            float cosB = std::sqrt(1.0f - sinB2);
        
            wiWorld = normalize(wiWorld);
            float cosI = std::abs(dot(wiWorld, n));
            float sinI = std::sqrt(1 - cosI * cosI);

            const auto cosAsubB = [](float cosA, float sinA, float cosB, float sinB) {
                return cosA <= cosB ? cosA * cosB + sinA * sinB : 1.0f;
            };
        
            importance *= cosAsubB(cosI, sinI, cosB, sinB);
        }

        return importance;
    }
};

std::optional<bvh_util::Split>
splitLightCluster(const std::vector<Light*>& lightCluster, const float3& diagonal)
{
    if (lightCluster.size() <= 2)
    {
        return {};
    }

    auto centroidBox = [&]() {
        BoundingBox cb;
        for (const auto* l : lightCluster)
        {
            float3 c = l->getBoundingBox().getCenter();
            cb = merge(cb, BoundingBox{c, c});
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

    struct Bin
    {
        float power = 0.0f;
        BoundingBox bbox;

        void clear()
        {
            power = 0.0f;
            bbox = BoundingBox();
        }
    };
    static constexpr uint8_t BIN_COUNT = SPLITS_PER_DIM + 1;
    Bin bins[BIN_COUNT];

    for (uint8_t dim = 0; dim < 3; ++dim)
    {
        if (centroidBox.min[dim] == centroidBox.max[dim])
        {
            continue;
        }

        float kr = maxComponent(diagonal) / std::max(diagonal[dim], 0.001f);

        for (auto& bin : bins)
        {
            bin.clear();
        }

        // binning
        for (const auto* l : lightCluster)
        {
            auto bbox = l->getBoundingBox();
            float c = bbox.getCenter()[dim];
            uint8_t binIdx = static_cast<uint8_t>(BIN_COUNT
                * (c - centroidBox.min[dim]) / (centroidBox.max[dim] - centroidBox.min[dim]));
            binIdx = std::min(binIdx, static_cast<uint8_t>(BIN_COUNT - 1));

            auto& bin = bins[binIdx];
            bin.power += length(l->getPower());
            bin.bbox = merge(bin.bbox, bbox);
        }

        float costs[SPLITS_PER_DIM] = { 0.0f };

        const auto accumulateCosts = [&](uint8_t binIdx, uint8_t splitIdx, Bin& binAccum) {
            const auto& bin = bins[binIdx];
            binAccum.power += bin.power;
            binAccum.bbox = merge(binAccum.bbox, bin.bbox);

            float sa =
                binAccum.bbox.min.x < std::numeric_limits<float>::max() /* is initialized or not */
                ? binAccum.bbox.computeSurfaceArea() : 0.0f;
            costs[splitIdx] += binAccum.power * sa;
        };

        Bin binBelow;
        for (uint8_t splitIdx = 0; splitIdx < SPLITS_PER_DIM; ++splitIdx)
        {
            uint8_t binIdx = splitIdx;
            accumulateCosts(binIdx, splitIdx, binBelow);
        }

        Bin binAbove;
        for (int8_t splitIdx = SPLITS_PER_DIM - 1; splitIdx >= 0 ; --splitIdx)
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

std::unique_ptr<LightNode>
createLightNode(const std::vector<Light*>& lightCluster)
{
    auto lightNode = std::make_unique<LightNode>();
    for (const auto* l : lightCluster)
    {
        lightNode->power += length(l->getPower());

        auto bbox = l->getBoundingBox();
        lightNode->bbox = merge(bbox, lightNode->bbox);
    }

    if (lightCluster.size() == 1)
    {
        lightNode->light = lightCluster[0];
    }
    else
    {
        auto split = splitLightCluster(lightCluster, lightNode->bbox.getDiagonal());

        std::vector<Light*> subClusters[2];
        subClusters[0].reserve(lightCluster.size());
        subClusters[1].reserve(lightCluster.size());

        if (split.has_value())
        {
            const auto& s = split.value();
            for (auto* l : lightCluster)
            {
                auto bbox = l->getBoundingBox();
                uint8_t scIdx = s.isBelow(bbox.getCenter()) ? 0 : 1;
                subClusters[scIdx].push_back(l);
            }
        }
        else
        {
            // split the light cluster by the middle index
            uint32_t midIdx = static_cast<uint32_t>(lightCluster.size() / 2);
            for (uint32_t i = 0; i < lightCluster.size(); ++i)
            {
                uint8_t scIdx = i < midIdx ? 0 : 1;
                subClusters[scIdx].push_back(lightCluster[i]);
            }
        }

        assert(!subClusters[0].empty());
        assert(!subClusters[1].empty());

        if (lightCluster.size() >= MIN_LIGHTS_FOR_PARALLELIZATION)
        {
            std::future<std::unique_ptr<LightNode>> children[2];
            for (uint8_t i = 0; i < 2; ++i)
            {
                children[i] = std::async([&, i]() {
                    return createLightNode(subClusters[i]);
                 });
            }

            for (uint8_t i = 0; i < 2; ++i)
            {
                lightNode->children[i] = children[i].get();
            }
        }
        else
        {
            for (uint8_t i = 0; i < 2; ++i)
            {
                lightNode->children[i] = createLightNode(subClusters[i]);
            }
        }
    }

    return lightNode;
}
}

struct BvhLightSampler::LightBvh
{
    std::unique_ptr<LightNode> root = nullptr;
};

BvhLightSampler::BvhLightSampler(const std::vector<std::shared_ptr<Light>>& lights)
    : bvh(std::make_unique<LightBvh>())
{
    if (lights.empty())
    {
        return;
    }

    std::vector<Light*> lightCluster(lights.size());
    for (uint32_t i = 0; i < lights.size(); ++i)
    {
        lightCluster[i] = lights[i].get();
    }

    bvh->root = createLightNode(lightCluster);
}

std::optional<LightSampler::Sample>
BvhLightSampler::sample(float u, const float3& hit, const float3& ns) const
{
    if (!bvh->root)
    {
        return {};
    }

    LightNode* node = bvh->root.get();
    float pdf = 1.0f;

    while (true)
    {
        if (node->children[0] && node->children[1])
        {
            float w[2] = {
                node->children[0]->importance(hit, ns),
                node->children[1]->importance(hit, ns) };

            if (w[0] == 0.0f && w[1] == 0.0f)
            {
                return {};
            }

            float up = (w[0] + w[1]) * u;

            uint8_t idx = up < w[0] ? 0 : 1;
            node = node->children[idx].get();
            pdf *= w[idx] / (w[0] + w[1]);

            // remap u value
            if (idx == 0)
            {
                u = std::min(up / w[0], ONE_MINUS_EPSILON);
            }
            else
            {
                u = std::min((up - w[0]) / w[1], ONE_MINUS_EPSILON);
            }
        }
        else
        {
            if (node->light && node->importance(hit, ns) > 0.0f)
            {
                return Sample{ node->light, pdf };
            }
            else
            {
                return {};
            }
        }
    }
}
}
