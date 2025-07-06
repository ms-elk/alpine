#pragma once

#include <alpine_config.h>

#include <memory>
#include <vector>

namespace alpine {
class Animation;
class Light;
class Shape;

struct Scene
{
    std::vector<std::shared_ptr<Shape>> shapes;
    std::vector<std::shared_ptr<Light>> lights;
    std::vector<std::shared_ptr<Animation>> animations;

    Scene()
    {
        shapes.reserve(MAX_SHAPES);
    }
};
}
