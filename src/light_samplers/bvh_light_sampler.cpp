#include "bvh_light_sampler.h"

#include "lights/light.h"
#include "utils/bounding_box.h"
#include "utils/util.h"

#include <assert.h>

namespace alpine {
namespace {
static constexpr uint8_t SPLITS_PER_DIM = 4;

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
        
            importance *= cosI <= cosB ? cosI * cosB + sinI * sinB : 1.0f;
        }

        return importance;
    }
};

struct Split
{
    uint8_t dim = 0;
    float point = 0.0f;
};

Split
splitLightCluster(const std::vector<Light*>& lightCluster, const BoundingBox& bbox)
{
    Split split;
    float minMetric = std::numeric_limits<float>::max();
    float3 bboxDiagonal = bbox.getDiagonal();

    for (uint8_t dim = 0; dim < 3; ++dim)
    {
        float kr = maxComponent(bboxDiagonal) / std::max(bboxDiagonal[dim], 0.001f);

        for (uint8_t splitIdx = 0; splitIdx < SPLITS_PER_DIM; ++splitIdx)
        {
            static constexpr float normalizer = 1.0f / static_cast<float>(SPLITS_PER_DIM + 1);
            float t = static_cast<float>(splitIdx + 1) * normalizer;
            float splitPoint = bbox.min[dim] * (1.0f - t) + bbox.max[dim] * t;

            struct Metric
            {
                float power = 0.0f;
                BoundingBox bbox;

                float evaluate() const {
                    float sa = bbox.min.x < std::numeric_limits<float>::max() /* is initialized or not */
                        ? bbox.computeSurfaceArea() : 0.0f;
                    return power * sa;
                };
            };
            Metric metric[2];

            for (const auto* l : lightCluster)
            {
                auto bbox = l->getBound();
                uint8_t metricIdx = bbox.getCenter()[dim] < splitPoint ? 0 : 1;
                auto& m = metric[metricIdx];

                m.power += length(l->getPower());
                m.bbox = merge(bbox, m.bbox);
            }

            float metricValue = kr * (metric[0].evaluate() + metric[1].evaluate());
            if (metricValue < minMetric)
            {
                minMetric = metricValue;
                split.dim = dim;
                split.point = splitPoint;
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

        auto bbox = l->getBound();
        lightNode->bbox = merge(bbox, lightNode->bbox);
    }

    if (lightCluster.size() == 1)
    {
        lightNode->light = lightCluster[0];
    }
    else
    {
        auto split = splitLightCluster(lightCluster, lightNode->bbox);

        std::vector<Light*> subClusters[2];
        subClusters[0].reserve(lightCluster.size());
        subClusters[1].reserve(lightCluster.size());

        for (auto* l : lightCluster)
        {
            auto bbox = l->getBound();
            uint8_t scIdx = bbox.getCenter()[split.dim] < split.point ? 0 : 1;
            subClusters[scIdx].push_back(l);
        }

        assert(!subClusters[0].empty());
        assert(!subClusters[1].empty());

        lightNode->children[0] = createLightNode(subClusters[0]);
        lightNode->children[1] = createLightNode(subClusters[1]);
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

LightSampler::Sample
BvhLightSampler::sample(float u, const float3& hit, const float3& ns) const
{
    if (!bvh->root)
    {
        return { nullptr, 0.0f };
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
                return { nullptr, 0.0f };
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
                return { node->light, pdf };
            }
            else
            {
                return { nullptr, 0.0f };
            }
        }
    }
}
}
