#include "image.h"

#include <fstream>
#include <algorithm>

namespace alpine {
bool
writePPM(std::string_view filename, uint32_t width, uint32_t height, const byte3* image)
{
    std::ofstream ppm(filename.data());
    if (!ppm)
    {
        return false;
    }

    ppm << "P3" << std::endl;
    ppm << width << ' ' << height << std::endl;
    ppm << "255" << std::endl;

    for (uint32_t y = 0; y < height; ++y)
    {
        for (uint32_t x = 0; x < width; ++x)
        {
            const byte3& pixel = image[y * width + x];
            auto r = static_cast<uint32_t>(pixel.x);
            auto g = static_cast<uint32_t>(pixel.y);
            auto b = static_cast<uint32_t>(pixel.z);
            ppm << r << ' ' << g << ' ' << b << std::endl;
        }
    }

    return true;
};
}