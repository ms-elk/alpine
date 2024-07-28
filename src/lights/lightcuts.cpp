#include "lightcuts.h"

#include "math/vector.h"
#include "point_light.h"

#include <algorithm>
#include <assert.h>
#include <memory>
#include <vector>

namespace alpine {
static constexpr uint8_t SPLITS_PER_DIM = 4;
struct BoundingBox
{
    float3 min = std::numeric_limits<float>::max();
    float3 max = -std::numeric_limits<float>::max();
};

struct LightNode
{
    float intensity = 0.0f;
    BoundingBox bbox;
    std::shared_ptr<LightNode> children[2] = { nullptr, nullptr };
    PointLight* point = nullptr;
};

struct Split
{
    uint8_t dim = 0;
    float point = 0.0f;
};

Split
splitLightCluster(const std::vector<PointLight*>& lightCluster, const BoundingBox& bbox)
{
    Split split;
    float minMetric = std::numeric_limits<float>::max();

    for (uint8_t dim = 0; dim < 3; ++dim)
    {
        for (uint8_t splitIdx = 0; splitIdx < SPLITS_PER_DIM; ++splitIdx)
        {
            static constexpr float normalizer = 1.0f / static_cast<float>(SPLITS_PER_DIM + 1);
            float t = static_cast<float>(splitIdx + 1) * normalizer;
            float splitPoint = bbox.min[dim] * (1.0f - t) + bbox.max[dim] * t;

            struct Metric
            {
                float intensity = 0.0f;
                BoundingBox bbox;

                float evaluate() const {
                    return intensity + dot(bbox.max - bbox.min, bbox.max - bbox.min); };
            };
            Metric metric[2];

            for (const auto* l : lightCluster)
            {
                float3 p = l->getPosition();
                uint8_t metricIdx = p[dim] < splitPoint ? 0 : 1;
                auto& m = metric[metricIdx];

                // TODO: use power?
                m.intensity += length(l->getPower());
                m.bbox.min = min(p, m.bbox.min);
                m.bbox.max = max(p, m.bbox.max);
            }

            float metricValue = metric[0].evaluate() + metric[1].evaluate();
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

std::shared_ptr<LightNode>
createLightNode(const std::vector<PointLight*>& lightCluster)
{
    auto lightNode = std::make_shared<LightNode>();
    for (const auto* l : lightCluster)
    {
        // TODO: use power?
        lightNode->intensity += length(l->getPower());

        float3 position = l->getPosition();
        lightNode->bbox.min = min(position, lightNode->bbox.min);
        lightNode->bbox.max = max(position, lightNode->bbox.max);
    }

    if (lightCluster.size() == 1)
    {
        lightNode->point = lightCluster[0];
    }
    else
    {
        auto split = splitLightCluster(lightCluster, lightNode->bbox);

        std::vector<PointLight*> subClusters[2];
        subClusters[0].reserve(lightCluster.size());
        subClusters[1].reserve(lightCluster.size());

        for (auto* l : lightCluster)
        {
            float3 p = l->getPosition();
            uint8_t scIdx = p[split.dim] < split.point ? 0 : 1;
            subClusters[scIdx].push_back(l);
        }

        assert(!subClusters[0].empty());
        assert(!subClusters[1].empty());

        lightNode->children[0] = createLightNode(subClusters[0]);
        lightNode->children[1] = createLightNode(subClusters[1]);
    }

    return lightNode;
}

std::shared_ptr<LightNode>
buildLightTree(const std::vector<PointLight*>& lights)
{
    if (lights.empty())
    {
        return nullptr;
    }

    return createLightNode(lights);
}
}
