#pragma once

#include <alpine_config.h>

#include <memory>
#include <vector>

namespace alpine {
class Animation;
class EnvironmentMap;
class Light;
class Shape;

struct Scene
{
    std::vector<std::shared_ptr<Shape>> shapes;
    std::vector<std::shared_ptr<Light>> lights;
    std::vector<std::shared_ptr<Animation>> animations;
    std::shared_ptr<EnvironmentMap> environmentMap;

    Scene()
    {
        shapes.reserve(MAX_SHAPES);
    }

    void reset()
    {
        shapes.clear();
        lights.clear();
        animations.clear();
    }
};
}
