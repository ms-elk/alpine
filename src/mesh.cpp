#include "mesh.h"

#include "kernel.h"
#include "ray.h"

namespace alpine {
Mesh::Mesh(const std::shared_ptr<Data>& data)
    : mData(data)
{
    kernel::createMesh(mData->vertices, mData->prims, this);
}

IntersectionAttributes
Mesh::getIntersectionAttributes(const Ray& ray, const kernel::Intersection& isect) const
{
    IntersectionAttributes isectAttr;
    if (isect.primId < mData->materials.size())
    {
        isectAttr.material = mData->materials[isect.primId].get();
    }

    return isectAttr;
}
}