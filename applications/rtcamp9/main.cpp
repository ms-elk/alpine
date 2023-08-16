#include <alpine.h>

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <vector>
#include <string_view>
#include <chrono>
#include <thread>

// https://github.com/richgel999/fpng
#include <fpng.h>

static constexpr float PI = 3.14159265358979323846f;

int32_t main(int32_t argc, const char* argv[])
{
    // レンダラー起動時間を取得。
    using clock = std::chrono::high_resolution_clock;
    const clock::time_point appStartTp = clock::now();

    uint32_t startFrameIndex = 0;
    uint32_t endFrameIndex = 0;
    std::string_view gltfName = "";
    for (int argIdx = 1; argIdx < argc; ++argIdx)
    {
        std::string_view arg = argv[argIdx];
        if (arg == "--frame-range")
        {
            if (argIdx + 2 >= argc)
            {
                printf("--frame-range requires a frame number pair to start and end.\n");
                return -1;
            }
            startFrameIndex = static_cast<uint32_t>(atoi(argv[argIdx + 1]));
            endFrameIndex = static_cast<uint32_t>(atoi(argv[argIdx + 2]));
            argIdx += 2;
        }
        else if (arg == "--gltf")
        {
            if (argIdx + 1 >= argc)
            {
                return -1;
            }
            gltfName = argv[argIdx + 1];
            argIdx++;
        }
        else
        {
            printf("Unknown argument %s.\n", argv[argIdx]);
            return -1;
        }
    }

    if (endFrameIndex < startFrameIndex)
    {
        printf("Invalid frame range.\n");
        return -1;
    }

    // constant values
    const uint32_t width = 256;
    const uint32_t height = 256;
    const uint32_t maxDepth = 8;

    const float emisssion[] = { 5.0f, 5.0f, 5.0f };
    const float lightPos[] = { 0.0f, 2.5f, 0.0f };
    const float lightRadius = 1.0f;

    const float eye[] = { 5.0f, 0.0f, 0.0f };
    const float target[] = { 0.0f, 0.0f, 0.0f };
    const float up[] = { 0.0f, 1.0f, 0.0f };

    const float fovy = PI / 2.0f;
    float aspect = float(width) / float(height);

    alpine::initialize(width, height, maxDepth);
    alpine::addDiskLight(emisssion, lightPos, lightRadius);

    auto* camera = alpine::getCamera();
    camera->setLookAt(eye, target, up, fovy, aspect);

    bool loaded = alpine::load(gltfName, alpine::FileType::GLTF);
    if (!loaded)
    {
        printf("ERROR: failed to load %s\n", gltfName.data());
        return 1;
    }

    fpng::fpng_init();

    for (uint32_t frameIndex = startFrameIndex; frameIndex <= endFrameIndex; ++frameIndex)
    {
        const clock::time_point frameStartTp = clock::now();
        printf("Frame %u ... ", frameIndex);

        alpine::render(1);

        // 起動からの時刻とフレーム時間を計算。
        const clock::time_point now = clock::now();
        const clock::duration frameTime = now - frameStartTp;
        const clock::duration totalTime = now - appStartTp;
        printf(
            "Done: %.3f [ms] (total: %.3f [s])\n",
            std::chrono::duration_cast<std::chrono::microseconds>(frameTime).count() * 1e-3f,
            std::chrono::duration_cast<std::chrono::milliseconds>(totalTime).count() * 1e-3f);

        // 3桁連番で画像出力。
        char filename[256];
        sprintf_s(filename, "%03u.png", frameIndex);
        const void* pixels = alpine::getFrameBuffer();
        fpng::fpng_encode_image_to_file(filename, pixels, width, height, 3, 0);
    }

    return 0;
}
