#include <alpine.h>

#include <string>

static constexpr float PI = 3.14159265358979323846f;

int
main(int argc, char* argv[])
{
    if (argc != 4)
    {
        printf("ERROR: invalid input parameters\n");
        return 1;
    }

    // input parameters
    uint32_t spp = std::atoi(argv[1]);
    const char* objFilename = argv[2];
    const char* imageFilename = argv[3];

    // constant values
    const uint32_t width = 256;
    const uint32_t height = 256;
    const uint32_t maxDepth = 8;
    const float emisssion[] = { 1.0f, 1.0f, 1.0f };
    const float lightPos[] = { 278.0f, 548.7f, 227.0f };
    const float lightRadius = 200.0f;
    const float eye[] = { 278.0f, 273.0f, -600.0f };
    const float target[] = { 278.0f, 273.0f, 0.0f };
    const float up[] = { 0.0f, 1.0f, 0.0f };
    const float fovy = PI / 2.0f;
    float aspect = float(width) / float(height);

    alpine::initialize(width, height, maxDepth);

    alpine::addDiskLight(emisssion, lightPos, lightRadius);

    bool loaded = alpine::load(objFilename, alpine::FileType::OBJ);
    if (!loaded)
    {
        printf("ERROR: failed to load %s\n", objFilename);
        return 1;
    }

    auto* camera = alpine::getCamera();
    camera->setLookAt(eye, target, up, fovy, aspect);

    alpine::render(spp);

    alpine::saveImage(imageFilename);

    return 0;
}