#include "sphere.h"

#include <accelerators/accelerator.h>
#include <intersection.h>
#include <ray.h>

namespace alpine {
void
Sphere::appendTo(Accelerator* accelerator)
{
    accelerator->appendSphere(mData->vertices, this);
}

void
Sphere::update(
    Accelerator* accelerator,
    const std::vector<float>& weights0,
    const std::vector<float>& weights1,
    float t)
{
    printf("ERROR: Sphere update has not been implemented yet.");
}

IntersectionAttributes
Sphere::getIntersectionAttributes(const Intersection& isect) const
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