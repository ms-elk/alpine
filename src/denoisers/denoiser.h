#pragma once

#include <math/vector.h>

namespace alpine {
namespace denoiser {
struct RenderTarget
{
    float3 color;
    float3 albedo;
    float3 normal;
};

void
initialize();

void
denoise(RenderTarget* renderTarget, uint32_t width, uint32_t height);
}
}
