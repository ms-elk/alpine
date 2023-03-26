#pragma once

#include "vector.h"

namespace alpine {
bool writePPM(const char* filename, int width, int height, const byte3* image);
}