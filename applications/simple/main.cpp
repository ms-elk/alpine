#include <alpine/alpine.h>

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
    const float emisssion[] = { 5.0f, 5.0f, 5.0f };
    const float lightPos[] = { 0.0f, 10.0f, 0.0f };
    const float lightRadius = 5.0f;
    const float eye[] = { 0.0f, 0.0f, -3.0f };
    const float target[] = { 0.0f, 0.0f, 0.0f };
    const float up[] = { 0.0f, 1.0f, 0.0f };
    const float fovy = PI / 2.0f;
    float aspect = float(width) / float(height);

    alpine::initialize(width, height, maxDepth);
    alpine::setBackgroundColor(0.0f, 0.0f, 0.0f);
    alpine::addDiskLight(emisssion, lightPos, lightRadius);

    bool loaded = alpine::load(objFilename, alpine::FileType::Obj);
    if (!loaded)
    {
        printf("ERROR: failed to load %s\n", objFilename);
        return 1;
    }

    alpine::buildLightSampler(alpine::LightSamplerType::Uniform);

    auto* camera = alpine::getCamera();
    camera->setLookAt(eye, target, up, fovy, aspect);

    alpine::render(spp);
    alpine::resolve(false);

    alpine::saveImage(imageFilename);

    return 0;
}