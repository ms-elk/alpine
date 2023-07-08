#include "mesh.h"

#include "kernel.h"
#include "ray.h"

namespace alpine {
Mesh::Mesh(Data&& data)
    : mData(std::move(data))
{
    kernel::createMesh(mData.vertices, mData.prims, this);
}

IntersectionAttributes
Mesh::getIntersectionAttributes(const kernel::Intersection& isect) const
{
    IntersectionAttributes isectAttr;
    float b1 = isect.barycentric.x;
    float b2 = isect.barycentric.y;
    float b0 = 1.0f - b1 - b2;

    const auto interpolate = [&](const auto& values, const std::vector<uint3>& prims) {
        const uint3& idx = prims[isect.primId];
        return values[idx[0]] * b0 + values[idx[1]] * b1 + values[idx[2]] * b2;
    };

    if (!mData.normals.empty())
    {
        isectAttr.ns = interpolate(mData.normals, mData.normalPrims);
    }
    else
    {
        isectAttr.ns = isect.ng;
    }
    std::tie(isectAttr.ss, isectAttr.ts) = getBasis(isectAttr.ns);

    if (!mData.uvs.empty())
    {
        isectAttr.uv = interpolate(mData.uvs, mData.uvPrims);
    }

    if (isect.primId < mData.materials.size())
    {
        isectAttr.material = mData.materials[isect.primId].get();
    }

    return isectAttr;
}
}