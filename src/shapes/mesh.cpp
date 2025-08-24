#include "mesh.h"

#include <accelerators/accelerator.h>
#include <materials/material.h>

#include <intersection.h>
#include <ray.h>
#include <texture.h>

#include <array>
#include <assert.h>

namespace alpine {
Mesh::Mesh(Data&& data, uint32_t weightCount)
    : mData(std::move(data))
    , mShapeId(Accelerator::INVALID_SHAPE_ID)
    , mWeights(weightCount)
{
    std::fill(mWeights.begin(), mWeights.end(), 0.0f);
}

void
Mesh::appendTo(Accelerator* accelerator)
{
    mShapeId = accelerator->appendMesh(mData.vertices, mData.prims, this);
    mVertexBuffer = static_cast<float3*>(accelerator->getVertexBuffer(mShapeId));
}

void
Mesh::update(
    Accelerator* accelerator,
    const std::vector<float>& weights0,
    const std::vector<float>& weights1,
    float t)
{
    assert(!mData.targets.empty());
    assert(mWeights.size() == weights0.size());
    assert(mWeights.size() == weights1.size());

    for (uint32_t i = 0; i < mWeights.size(); ++i)
    {
        mWeights[i] = std::lerp(weights0[i], weights1[i], t);
    }

    for (uint32_t i = 0; i < mData.vertices.size(); ++i)
    {
        mVertexBuffer[i] = mData.vertices[i];
        for (uint32_t j = 0; j < mData.targets.size(); ++j)
        {
            mVertexBuffer[i] += mData.targets[j].vertices[i] * mWeights[j];
        }
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

    const auto interpolate = [&](const auto& values, const uint3& prim) {
        return values[prim[0]] * b0 + values[prim[1]] * b1 + values[prim[2]] * b2;
    };

    if (!mData.uvs.empty())
    {
        isectAttr.uv = interpolate(
            mData.uvs,
            !mData.uvPrims.empty() ? mData.uvPrims[isect.primId] : mData.prims[isect.primId]);
    }

    if (isect.primId < mData.materials.size())
    {
        isectAttr.material = mData.materials[isect.primId].get();
    }

    if (!mData.normals.empty())
    {
        const auto& normalPrim = !mData.normalPrims.empty()
            ? mData.normalPrims[isect.primId]
            : mData.prims[isect.primId];

        isectAttr.ns = [&](){
            float3 ns = interpolate(mData.normals, normalPrim);

            // morphing
            if (!mData.targets.empty())
            {
                for (uint32_t i = 0; i < mData.targets.size(); ++i)
                {
                    ns += interpolate(mData.targets[i].normals, normalPrim) * mWeights[i];
                }

                ns = normalize(ns);
            }

            return ns;
        }();

        // tangent space
        if (isectAttr.material && !mData.tangents.empty())
        {
            const auto [tangent, bitangent] = [&]() {
                std::array<float3, 3> tangents = {
                mData.tangents[normalPrim[0]].xyz(),
                mData.tangents[normalPrim[1]].xyz(),
                mData.tangents[normalPrim[2]].xyz() };

                // morphing
                if (!mData.targets.empty())
                {
                    for (uint32_t i = 0; i < tangents.size(); ++i)
                    {
                        for (uint32_t j = 0; j < mData.targets.size(); ++j)
                        {
                            tangents[i] += mData.targets[j].tangents[normalPrim[i]] * mWeights[j];
                        }

                        tangents[i] = normalize(tangents[i]);
                    }
                }

                std::array<float3, 3> bitangents;
                for (uint32_t i = 0; i < 3; ++i)
                {
                    const float3& normal = mData.normals[normalPrim[i]];
                    float sign = mData.tangents[normalPrim[i]].w;
                    bitangents[i] = cross(normal, tangents[i]) * sign;
                    bitangents[i] = normalize(bitangents[i]);
                }

                float3 tan = interpolate(tangents, uint3(0, 1, 2));
                float3 bitan = interpolate(bitangents, uint3(0, 1, 2));

                return std::pair<float3, float3>(tan, bitan);
                }();

            float3 v = isectAttr.material->getNormal(isectAttr.uv);
            isectAttr.ns = normalize(bitangent * v.x + tangent * v.y + isectAttr.ns * v.z);
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
