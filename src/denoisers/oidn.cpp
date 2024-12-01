#include "denoiser.h"

#include <OpenImageDenoise/oidn.hpp>

namespace alpine::denoiser {
oidn::DeviceRef gDenoiser;

void
initialize()
{
    gDenoiser = oidn::newDevice();
    gDenoiser.commit();
}

void
denoise(RenderTarget* renderTarget, uint32_t width, uint32_t height)
{
    oidn::FilterRef filter = gDenoiser.newFilter("RT");
    filter.setImage("color", renderTarget, oidn::Format::Float3, width, height, 0, sizeof(RenderTarget));
    filter.setImage("albedo", renderTarget, oidn::Format::Float3, width, height, sizeof(float3), sizeof(RenderTarget));
    filter.setImage("normal", renderTarget, oidn::Format::Float3, width, height, 2 * sizeof(float3), sizeof(RenderTarget));
    filter.setImage("output", renderTarget, oidn::Format::Float3, width, height, 0, sizeof(RenderTarget));
    filter.set("hdr", true);
    filter.commit();
    filter.execute();
}
}
