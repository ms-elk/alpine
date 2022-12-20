#include <alpine.h>

#include <string>

#define _USE_MATH_DEFINES
#include <math.h>

int
main(int argc, char* argv[])
{
    if (argc != 4)
    {
        printf("ERROR: invalid input parameters\n");
        return 1;
    }

    // input parameters
    int spp = std::atoi(argv[1]);
    const char* objFilename = argv[2];
    const char* imageFilename = argv[3];

    // constant values
    const int width = 256;
    const int height = 256;
    const int maxDepth = 8;
    const float eye[] = { 278.0f, 273.0f, -600.0f };
    const float at[] = { 278.0f, 273.0f, 0.0f };
    const float up[] = { 0.0f, 1.0f, 0.0f };
    const float fovy = float(M_PI) / 2.0f;
    float aspect = float(width) / float(height);

    alpine::initialize(width, height, maxDepth);

    bool loaded = alpine::loadObj(objFilename);
    if (!loaded)
    {
        printf("ERROR: failed to load %s\n", objFilename);
        return 1;
    }

    alpine::setCamera(eye, at, up, fovy, aspect);

    alpine::render(spp);

    alpine::saveImage(imageFilename);

    return 0;
}