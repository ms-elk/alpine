#pragma once

#include <fstream>
#include <algorithm>

namespace alpine {
template <typename T>
bool
writePPM(const char* filename, int width, int height, const T* image)
{
    std::ofstream ppm(filename);
    if (!ppm)
    {
        return false;
    }

    ppm << "P3" << std::endl;
    ppm << width << ' ' << height << std::endl;
    ppm << "255" << std::endl;

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const auto& pixel = image[y * width + x];
            int r = static_cast<int>(std::clamp(pixel.x, 0.0f, 1.0f) * 255.0f + 0.5f);
            int g = static_cast<int>(std::clamp(pixel.y, 0.0f, 1.0f) * 255.0f + 0.5f);
            int b = static_cast<int>(std::clamp(pixel.z, 0.0f, 1.0f) * 255.0f + 0.5f);
            ppm << r << ' ' << g << ' ' << b << std::endl;
        }
    }

    return true;
};
}