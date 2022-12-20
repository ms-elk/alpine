#pragma once

namespace alpine {
namespace kernel{
struct Intersection;
}
struct Ray;
class Material;

struct IntersectionAttributes
{
    Material* material = nullptr;
};

class Shape
{
public:
    Shape(){}
    virtual ~Shape() {}

    virtual IntersectionAttributes getIntersectionAttributes(
        const Ray& ray, const kernel::Intersection& isect) const = 0;
};
}