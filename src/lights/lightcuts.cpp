#include "lightcuts.h"

#include "math/vector.h"
#include "point_light.h"

#include <algorithm>
#include <memory>
#include <vector>

namespace alpine {
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
    uint32_t idx = 0;
    float intensity[2] = { 0.0f, 0.0f };
    BoundingBox bbox[2];
};

Split
splitLightCluster(std::vector<PointLight*>* lightCluster, const BoundingBox& bbox)
{
    uint8_t major = [&]() {
        float3 length = bbox.max - bbox.min;
        if (length.x >= length.y)
        {
            return length.x >= length.z ? 0 : 2;
        }
        else
        {
            return length.y >= length.z ? 1 : 2;
        }
    }();

    std::sort(lightCluster->begin(), lightCluster->end(), [major](const auto* l0, const auto* l1) {
        return l0->getPosition()[major] < l1->getPosition()[major];
     });

    struct Metric
    {
        float intensity = 0.0f;
        BoundingBox bbox;

        float evaluate() const { return intensity + dot(bbox.max-bbox.min, bbox.max - bbox.min); };
    };

    Split s;
    float minMetric = std::numeric_limits<float>::max();
    for (uint32_t idx = 1; idx < lightCluster->size() - 1; ++idx)
    {
        const auto computeMetric = [&](size_t begin, size_t end) {
            Metric m;

            for (size_t i = begin; i < end; ++i)
            {
                const auto* l = (*lightCluster)[i];

                // TODO: use power?
                m.intensity += length(l->getPower());

                float3 position = l->getPosition();
                m.bbox.min = min(position, m.bbox.min);
                m.bbox.max = max(position, m.bbox.max);
            }

            return m;
        };

        Metric m[2] = { computeMetric(0, idx), computeMetric(idx, lightCluster->size()) };
        float metricValue = m[0].evaluate() + m[1].evaluate();

        if (metricValue < minMetric)
        {
            minMetric = metricValue;

            s.idx = idx;
            for (int i = 0; i < 2; ++i)
            {
                s.intensity[i] = m[i].intensity;
                s.bbox[i] = m[i].bbox;
            }
        }
    }

    return s;
}

std::shared_ptr<LightNode>
createLightNode(std::vector<PointLight*>* lightCluster, float intensity, const BoundingBox& bbox)
{
    auto lightNode = std::make_shared<LightNode>();
    lightNode->intensity = intensity;
    lightNode->bbox = bbox;

    if (lightCluster->size() == 1)
    {
        lightNode->point = (*lightCluster)[0];
    }
    else
    {
        auto split = splitLightCluster(lightCluster, bbox);

        std::vector<PointLight*> clusters[2] = {
            std::vector<PointLight*>(lightCluster->begin(), lightCluster->begin() + split.idx),
            std::vector<PointLight*>(lightCluster->begin() + split.idx, lightCluster->end())
        };

        for (uint32_t i = 0; i < 2; ++i)
        {
            lightNode->children[i] = createLightNode(
                &clusters[i], split.intensity[i], split.bbox[i]);
        }
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

    std::vector<PointLight*> lightCluster = lights;

    float intensity = 0;
    BoundingBox bbox;
    for (const auto* l : lightCluster)
    {
        // TODO: use power?
        intensity += length(l->getPower());

        float3 position = l->getPosition();
        bbox.min = min(position, bbox.min);
        bbox.max = max(position, bbox.max);
    }

    return createLightNode(&lightCluster, intensity, bbox);
}
}
