#include <alpine/alpine.h>

#include <chrono>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <numbers>
#include <string>

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

const std::unordered_map<std::string, alpine::AcceleratorType> gAcceleratorTypeTable = {
    {"bvh", alpine::AcceleratorType::Bvh},
    {"bvh4", alpine::AcceleratorType::Bvh4},
    {"embree", alpine::AcceleratorType::Embree}
};

const std::unordered_map<std::string, alpine::LightSamplerType> gLightSamplerTypeTable = {
    {"uniform", alpine::LightSamplerType::Uniform},
    {"power", alpine::LightSamplerType::Power},
    {"bvh", alpine::LightSamplerType::Bvh}
};

int
main(int argc, char* argv[])
{
    // input parameters
    std::filesystem::path inputPath;
    alpine::FileType fileType;
    std::filesystem::path outputPath;
    uint32_t spp;
    alpine::AcceleratorType acceleratorType;
    alpine::LightSamplerType lightSamplerType;
    bool denoise = false;

    CLI::App app;
    app.add_option("-i,--input", inputPath, "Input file (obj|glb)")
        ->required()
        ->check([&fileType](const std::filesystem::path& filePath) {
            std::string extension = filePath.extension().string();
            if (extension == ".obj")
            {
                fileType = alpine::FileType::Obj;
                return "";
            }
            else if (extension == ".glb")
            {
                fileType = alpine::FileType::Gltf;
                return "";
            }
            else
            {
                return "File extension must be .obj or .glb.";
            }
        });
    app.add_option("-o,--output", outputPath, "Output image file (ppm)")
        ->required()
        ->check([](const std::filesystem::path& filePath) {
            std::string extension = filePath.extension().string();
            if (extension == ".ppm")
            {
                return "";
            }
            else
            {
                return "File extension must be .ppm.";
            }
        });
    app.add_option("--spp", spp, "Number of samples per pixel")
        ->required();
    app.add_option(
        "--accelerator", acceleratorType, "Accelerator type: \"bvh\", \"bvh4\" or \"embree\"")
        ->required()
        ->transform(CLI::CheckedTransformer(gAcceleratorTypeTable));
    app.add_option(
        "--lightSampler", lightSamplerType, "Light sampler type: \"uniform\", \"power\", or \"bvh\"")
        ->required()
        ->transform(CLI::CheckedTransformer(gLightSamplerTypeTable));
    app.add_flag("--denoiser", denoise, "Enable denoiser");

    CLI11_PARSE(app, argc, argv);

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
    const float fovy = std::numbers::pi_v<float> / 2.0f;
    const float aspect = float(width) / float(height);

    alpine::initialize(width, height, maxDepth, acceleratorType);
    alpine::addDiskLight(lightPower, lightColor, lightPos, lightRadius);

    bool loaded = alpine::load(inputPath.string(), fileType);
    if (!loaded)
    {
        printf("ERROR: failed to load %s\n", inputPath.string().c_str());
        return 1;
    }

    {
        Timer timer("Accelerator Build");
        alpine::buildAccelerator();
    }

    {
        Timer timer("Light Sampler Build");
        alpine::buildLightSampler(lightSamplerType);
    }

    auto* camera = alpine::getCamera();
    camera->setLookAt(eye, target, up, fovy, aspect);

    {
        Timer timer("Render");
        alpine::render(spp);
    }

    alpine::resolve(denoise);

    alpine::saveImage(outputPath.string());

    return 0;
}
