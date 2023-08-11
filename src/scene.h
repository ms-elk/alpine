#pragma once

#include <memory>
#include <vector>

namespace alpine {
class Shape;
class Light;

struct Scene
{
    std::vector<std::shared_ptr<Shape>> shapes;
    std::shared_ptr<Light> light;
};
}