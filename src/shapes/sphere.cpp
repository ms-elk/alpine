#include "sphere.h"

#include "accelerators/accelerator.h"
#include "ray.h"

namespace alpine {
Sphere::Sphere(const std::shared_ptr<Data>& data)
    : mData(data)
{
    accelerator::createSphere(mData->vertices, this);
}

IntersectionAttributes
Sphere::getIntersectionAttributes(const accelerator::Intersection& isect) const
{
    IntersectionAttributes isectAttr;
    isectAttr.ns = isect.ng;
    std::tie(isectAttr.ss, isectAttr.ts) = getBasis(isectAttr.ns);

    if (isect.primId < mData->materials.size())
    {
        isectAttr.material = mData->materials[isect.primId].get();
    }

    return isectAttr;
}
}