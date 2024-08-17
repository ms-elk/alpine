#include <alpine/alpine.h>

#include <chrono>
#include <filesystem>
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
    if (argc != 5)
    {
        printf("ERROR: invalid input parameters\n");
        return 1;
    }

    // input parameters
    uint32_t spp = std::atoi(argv[1]);
    std::string lightSamplerType = argv[2];
    std::filesystem::path filePath = argv[3];
    const char* imageFilename = argv[4];

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

    std::string extension = filePath.extension().string();
    alpine::FileType fileType;
    if (extension == ".obj")
    {
        fileType = alpine::FileType::Obj;
    }
    else if (extension == ".glb")
    {
        fileType = alpine::FileType::Gltf;
    }
    else
    {
        printf("ERROR: invalid file format: %s\n", filePath.filename().string().c_str());
        return 1;
    }

    bool loaded = alpine::load(filePath.string(), fileType);
    if (!loaded)
    {
        printf("ERROR: failed to load %s\n", filePath.string().c_str());
        return 1;
    }

    alpine::LightSamplerType lsType;
    if (lightSamplerType == "uniform")
    {
        lsType = alpine::LightSamplerType::Uniform;
    }
    else if (lightSamplerType == "power")
    {
        lsType = alpine::LightSamplerType::Power;
    }
    else if (lightSamplerType == "bvh")
    {
        lsType = alpine::LightSamplerType::Bvh;
    }
    else
    {
        printf("ERROR: invalid light sampler type: %s\n", lightSamplerType.c_str());
        return 1;
    }

    {
        Timer timer("Light Sampler Build");
        alpine::buildLightSampler(lsType);
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
