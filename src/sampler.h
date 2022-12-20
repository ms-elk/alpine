#pragma once

#include "vector.h"

namespace alpine {
Vector2f get2D();
Vector2f sampleConcentricDisk(const Vector2f& u);
Vector3f sampleCosineWeightedHemisphere(float& pdf, const Vector2f& u);
}