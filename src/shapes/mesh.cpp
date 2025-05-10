#include "mesh.h"

#include "accelerators/accelerator.h"
#include "materials/material.h"

#include "intersection.h"
#include "ray.h"
#include "texture.h"

#include <assert.h>

namespace alpine {
void
Mesh::appendTo(Accelerator* accelerator) const
{
    accelerator->appendMesh(mData.vertices, mData.prims, this);
}

IntersectionAttributes
Mesh::getIntersectionAttributes(const Intersection& isect) const
{
    IntersectionAttributes isectAttr;
    float b1 = isect.barycentric.x;
    float b2 = isect.barycentric.y;
    float b0 = 1.0f - b1 - b2;

    const auto interpolate = [&](const auto& values, const std::vector<uint3>& prims) {
        const uint3& idx = prims[isect.primId];
        return values[idx[0]] * b0 + values[idx[1]] * b1 + values[idx[2]] * b2;
    };

    if (!mData.uvs.empty())
    {
        isectAttr.uv = interpolate(
            mData.uvs, !mData.uvPrims.empty() ? mData.uvPrims : mData.prims);
    }

    if (isect.primId < mData.materials.size())
    {
        isectAttr.material = mData.materials[isect.primId].get();
    }

    if (!mData.normals.empty())
    {
        isectAttr.ns = interpolate(
            mData.normals, !mData.normalPrims.empty() ? mData.normalPrims : mData.prims);

        if (isectAttr.material && !mData.tangents.empty())
        {
            assert(!mData.bitangents.empty());
            float3 tan = interpolate(
                mData.tangents, !mData.normalPrims.empty() ? mData.normalPrims : mData.prims);
            float3 bitan = interpolate(
                mData.bitangents, !mData.normalPrims.empty() ? mData.normalPrims : mData.prims);

            float3 v = isectAttr.material->getNormal(isectAttr.uv);
            isectAttr.ns = normalize(bitan * v.x + tan * v.y + isectAttr.ns * v.z);
        }
    }
    else
    {
        isectAttr.ns = isect.ng;
    }
    std::tie(isectAttr.ss, isectAttr.ts) = getBasis(isectAttr.ns);

    return isectAttr;
}
}
