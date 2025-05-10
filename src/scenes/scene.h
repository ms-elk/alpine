#pragma once

#include <alpine_config.h>

#include <memory>
#include <vector>

namespace alpine {
class Shape;
class Light;

struct Scene
{
    std::vector<std::shared_ptr<Shape>> shapes;
    std::vector <std::shared_ptr<Light>> lights;

    Scene()
    {
        shapes.reserve(MAX_SHAPES);
    }
};
}
