#include "file_loader.h"

#include <math/vector.h>
#include <scenes/scene.h>
#include <environment_map.h>

#include <stb/stb_image.h>

namespace alpine {
bool
loadHdr(Scene* scene, std::string_view filename)
{
    int w, h, ch;
    float* src = stbi_loadf(filename.data(), &w, &h, &ch, 3);
    if (!src)
    {
        return false;
    }

    int pixelCount = w * h;
    std::vector<float3> hdrData(pixelCount);
    memcpy(hdrData.data(), src, pixelCount * sizeof(float3));

    scene->environmentMap = std::make_shared<EnvironmentMap>(std::move(hdrData), w, h);

    return true;
}
}
