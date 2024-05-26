#pragma once

#include "light.h"
#include "math/vector.h"

namespace alpine {
struct IntersectionAttributes;

class DiskLight : public Light
{
public:
    DiskLight(const float3& emission, const float3& position, const float3& normal, float radius);

    Sample sample(const float2& u, const float3& hit) const override;

    std::pair<float /* pdf */, float /* distance */>
        computePdf(const float3& hit, const float3& wiWorld) const override;

    float3 getEmission() const override { return mEmission * mScale; }

    float3 getPower() const override;

private:
    float3 mEmission;
    float3 mNormal;
    float mRadius;

    float3 mBinormal;
    float3 mTangent;
    float mArea;
};
}
