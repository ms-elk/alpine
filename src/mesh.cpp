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
Mesh::getIntersectionAttributes(const kernel::Intersection& isect) const
{
    IntersectionAttributes isectAttr;

    isectAttr.ns = isect.ng; // TODO

    if (!mData->uvs.empty())
    {
        const uint3& idx = mData->uvPrims[isect.primId];
        float a = isect.barycentric.x;
        float b = isect.barycentric.y;
        float c = 1.0f - a - b;
        isectAttr.uv = mData->uvs[idx[0]] * c + mData->uvs[idx[1]] * a + mData->uvs[idx[2]] * b;
    }

    if (isect.primId < mData->materials.size())
    {
        isectAttr.material = mData->materials[isect.primId].get();
    }

    return isectAttr;
}
}