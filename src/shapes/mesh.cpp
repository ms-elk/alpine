#include "mesh.h"

#include <accelerators/accelerator.h>
#include <materials/material.h>

#include <intersection.h>
#include <ray.h>
#include <texture.h>

#include <assert.h>

namespace alpine {
Mesh::Mesh(Data&& data)
    : mData(std::move(data))
    , mShapeId(Accelerator::INVALID_SHAPE_ID)
{}

void
Mesh::appendTo(Accelerator* accelerator)
{
    mShapeId = accelerator->appendMesh(mData.vertices, mData.prims, this);
    mVertexBuffer = static_cast<float3*>(accelerator->getVertexBuffer(mShapeId));
}

void
Mesh::update(Accelerator* accelerator, float weight)
{
    assert(!mData.targets.empty());

    mWeight = weight;

    for (uint32_t i = 0; i < mData.vertices.size(); ++i) {
        mVertexBuffer[i] = lerp(mData.vertices[i], mData.targets[0].vertices[i], mWeight);
    }

    accelerator->updateShape(mShapeId);
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

    const auto morph = [&](
        const auto& value, const auto& target, const std::vector<uint3>& prims) {
            auto t = interpolate(target, prims);
            return lerp(value, t, mWeight);
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
        const auto& normalPrims = !mData.normalPrims.empty() ? mData.normalPrims : mData.prims;

        isectAttr.ns = interpolate(mData.normals, normalPrims);
        if (!mData.targets.empty())
        {
            isectAttr.ns = morph(isectAttr.ns, mData.targets[0].normals, normalPrims);
        }

        if (isectAttr.material && !mData.tangents.empty())
        {
            assert(!mData.bitangents.empty());
            float3 tan = interpolate(mData.tangents, normalPrims);
            float3 bitan = interpolate(mData.bitangents, normalPrims);
            if (!mData.targets.empty())
            {
                tan = morph(tan, mData.targets[0].tangents, normalPrims);
                bitan = morph(bitan, mData.targets[0].bitangents, normalPrims);
            }

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
