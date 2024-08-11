#include <alpine/alpine.h>

#include <chrono>
#include <string>

static constexpr float PI = 3.14159265358979323846f;

class Timer
{
public:
    Timer(const char* name)
        : name(name)
    {
        start = clock::now();
    }

    ~Timer()
    {
        clock::time_point end = clock::now();
        clock::duration time = end - start;
        printf("%s: %.3f [ms]\n", name,
            std::chrono::duration_cast<std::chrono::microseconds>(time).count() * 1e-3f);
    }

private:
    using clock = std::chrono::high_resolution_clock;
    const char* name;
    clock::time_point start;
};

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
    const float lightPower = 100.0f;
    const float lightColor[] = { 1.0f, 1.0f, 1.0f };
    const float lightPos[] = { 0.0f, 10.0f, 0.0f };
    const float lightRadius = 5.0f;
    const float eye[] = { 0.0f, 0.0f, -3.0f };
    const float target[] = { 0.0f, 0.0f, 0.0f };
    const float up[] = { 0.0f, 1.0f, 0.0f };
    const float fovy = PI / 2.0f;
    float aspect = float(width) / float(height);

    alpine::initialize(width, height, maxDepth);
    alpine::setBackgroundColor(0.0f, 0.0f, 0.0f);
    alpine::addDiskLight(lightPower, lightColor, lightPos, lightRadius);

    bool loaded = alpine::load(objFilename, alpine::FileType::Obj);
    if (!loaded)
    {
        printf("ERROR: failed to load %s\n", objFilename);
        return 1;
    }

    {
        Timer timer("Light Sampler Build");
        alpine::buildLightSampler(alpine::LightSamplerType::Bvh);
    }

    auto* camera = alpine::getCamera();
    camera->setLookAt(eye, target, up, fovy, aspect);

    {
        Timer timer("Render");
        alpine::render(spp);
    }

    alpine::resolve(false);

    alpine::saveImage(imageFilename);

    return 0;
}