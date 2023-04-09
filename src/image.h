#pragma once

#include "vector.h"

namespace alpine {
bool writePPM(const char* filename, uint32_t width, uint32_t height, const byte3* image);
}