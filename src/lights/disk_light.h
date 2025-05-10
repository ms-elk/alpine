#pragma once

#include "light.h"

#include <math/vector.h>

namespace alpine {
struct IntersectionAttributes;

class DiskLight final : public Light
{
public:
    DiskLight(
        float power, const float3& color, const float3& position, const float3& normal, float radius);

    std::optional<Sample> sample(const float2& u, const float3& hit) const override;

    std::pair<float /* pdf */, float /* distance */>
        computePdf(const float3& hit, const float3& wiWorld) const override;

    float3 getEmittedRadiance() const override { return mEmittedRadiance * mScale; }

    float3 getPower() const override { return mPower * mScale; }

    bool isDelta() const override { return false; }

    BoundingBox getBoundingBox() const override { return mBbox; }

private:
    float3 mEmittedRadiance;
    float3 mPower;
    float3 mNormal;
    float mRadius;

    float3 mBinormal;
    float3 mTangent;
    float mArea;
    BoundingBox mBbox;
};
}
