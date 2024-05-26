#pragma once

#include "math/vector.h"

#include <string_view>

namespace alpine {
bool writePPM(std::string_view filename, uint32_t width, uint32_t height, const byte3* image);
}
