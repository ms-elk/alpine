#include "image.h"

#include <fstream>
#include <algorithm>

namespace alpine {
bool
writePPM(const char* filename, int width, int height, const byte3* image)
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
            const byte3& pixel = image[y * width + x];
            unsigned int r = static_cast<unsigned int>(pixel.x);
            unsigned int g = static_cast<unsigned int>(pixel.y);
            unsigned int b = static_cast<unsigned int>(pixel.z);
            ppm << r << ' ' << g << ' ' << b << std::endl;
        }
    }

    return true;
};
}