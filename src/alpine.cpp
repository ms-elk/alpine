#include <alpine.h>

#include "camera.h"
#include "debug_scene.h"
#include "image.h"
#include "kernel.h"
#include "material.h"
#include "mesh.h"
#include "obj_converter.h"
#include "ray.h"
#include "sampler.h"
#include "sphere.h"
#include "util.h"

#include <algorithm>
#include <future>
#include <memory>
#include <vector>

namespace alpine {
static constexpr uint32_t MAX_SHAPES = 10;
static constexpr uint32_t TILE_SIZE = 64;

class Alpine
{
public:
    Alpine(const Alpine&) = delete;
    Alpine& operator=(const Alpine&) = delete;

    static Alpine& getInstance()
    {
        static Alpine inst;
        return inst;
    }

    inline void setBackgroundColor(float r, float g, float b)
    {
        mBackgroundColor = float3(r, g, b);
    }

    inline ICamera* getCamera() { return &mCamera; }

    inline const void* getFrameBuffer() const { return mFrameBuffer.data(); }

    void initialize(uint32_t width, uint32_t height, uint32_t maxDepth);

    bool loadObj(const char* filename);

    void resetAccumulation();

    void render(uint32_t spp);

    void saveImage(const char* filename) const;

    void addDebugScene();

private:
    Alpine()
    {
        kernel::initialize();
    }
    ~Alpine() { kernel::finalize(); }

private:
    uint32_t mWidth = 0;
    uint32_t mHeight = 0;
    uint32_t mMaxDepth = 0;

    float3 mBackgroundColor = float3(1.0f);

    std::vector<Sampler> mSamplers;
    std::vector<float3> mAccumBuffer;
    std::vector<byte3> mFrameBuffer;
    uint32_t mTotalSamples = 0;

    uint32_t mTileWidth = 0;
    uint32_t mTileHeight = 0;
    std::vector<std::future<void>> mTiles;

    std::vector<std::shared_ptr<Shape>> mScene;

    Camera mCamera;
};

void
Alpine::initialize(uint32_t width, uint32_t height, uint32_t maxDepth)
{
    mWidth = width;
    mHeight = height;
    mMaxDepth = maxDepth;

    uint32_t pixelCount = mWidth * mHeight;
    mSamplers.resize(pixelCount);
    mAccumBuffer.resize(pixelCount);
    mFrameBuffer.resize(pixelCount);

    mTileWidth = width / TILE_SIZE;
    mTileHeight = height / TILE_SIZE;
    uint32_t tileCount = mTileWidth * mTileHeight;
    mTiles.resize(tileCount);

    mScene.reserve(MAX_SHAPES);

    resetAccumulation();
}

bool
Alpine::loadObj(const char* filename)
{
    mScene.push_back(createMesh(filename));
    if (!mScene.back())
    {
        mScene.pop_back();
        return false;
    }

    kernel::updateScene();

    return true;
}

void
Alpine::resetAccumulation()
{
    for (uint32_t p = 0; p < mSamplers.size(); ++p)
    {
        mSamplers[p].reset(p);
    }
    std::fill(mAccumBuffer.begin(), mAccumBuffer.end(), float3(0.0f));
    mTotalSamples = 0;
}

void
Alpine::render(uint32_t spp)
{
    for (uint32_t sampleId = 0; sampleId < spp; ++sampleId)
    {
        for (uint32_t tileId = 0; tileId < mTiles.size(); ++tileId)
        {
            mTiles[tileId] = std::async([&, tileId]() {
                uint32_t xBegin = tileId % mTileWidth * TILE_SIZE;
                uint32_t yBegin = tileId / mTileWidth * TILE_SIZE;

                for (uint32_t y = yBegin; y < yBegin + TILE_SIZE; ++y)
                {
                    for (uint32_t x = xBegin; x < xBegin + TILE_SIZE; ++x)
                    {
                        uint32_t index = y * mWidth + x;
                        auto& sampler = mSamplers[index];
                        float2 jitter = sampler.get2D();
                        auto ray = mCamera.generateRay(
                            (x + jitter.x) / float(mWidth), (y + jitter.y) / float(mHeight));

                        float3 throughput(1.0f, 1.0f, 1.0f);
                        float3 radiance(0.0f, 0.0f, 0.0f);
                        for (uint32_t depth = 0; depth < mMaxDepth; ++depth)
                        {
                            auto isect = kernel::intersect(ray);

                            if (!isect.shapePtr)
                            {
                                radiance += throughput * mBackgroundColor;
                                break;
                            }

                            const auto* shape = static_cast<Shape*>(isect.shapePtr);
                            auto isectAttr
                                = shape->getIntersectionAttributes(ray, isect);

                            float3 wi;
                            float pdf;
                            float3 bsdf = isectAttr.material->sample(
                                ray.dir, isect.ng, sampler.get2D(), wi, pdf);
                            if (pdf == 0.0f)
                            {
                                break;
                            }

                            float cosTerm = std::abs(dot(wi, isect.ng));
                            throughput = throughput * bsdf * cosTerm / pdf;

                            float3 rayOffset = isect.ng * 0.001f;
                            ray.org = ray.org + ray.dir * isect.t + rayOffset;
                            ray.dir = wi;
                        }

                        mAccumBuffer[index] += radiance;
                    }
                }
                });
        }

        for (auto& tile : mTiles)
        {
            tile.get();
        }
    }
    mTotalSamples += spp;

    for (uint32_t i = 0; i < mFrameBuffer.size(); ++i)
    {
        auto pixel = mAccumBuffer[i];
        pixel /= float(mTotalSamples);

        auto& fb = mFrameBuffer[i];
        fb.x = static_cast<uint8_t>(std::clamp(pixel.x, 0.0f, 1.0f) * 255.0f + 0.5f);
        fb.y = static_cast<uint8_t>(std::clamp(pixel.y, 0.0f, 1.0f) * 255.0f + 0.5f);
        fb.z = static_cast<uint8_t>(std::clamp(pixel.z, 0.0f, 1.0f) * 255.0f + 0.5f);
    }
}

void
Alpine::saveImage(const char* filename) const
{
    writePPM(filename, mWidth, mHeight, mFrameBuffer.data());
}

void
Alpine::addDebugScene()
{
    mScene.push_back(createDebugTriangle());
    mScene.push_back(createDebugSphere());
    kernel::updateScene();
}

///////////////////////////////////////////////////////////////
void
initialize(uint32_t width, uint32_t height, uint32_t maxDepth)
{
    Alpine::getInstance().initialize(width, height, maxDepth);
}

bool
loadObj(const char* filename)
{
    return Alpine::getInstance().loadObj(filename);
}

void
setBackgroundColor(float r, float g, float b)
{
    Alpine::getInstance().setBackgroundColor(r, g, b);
}

ICamera*
getCamera()
{
    return Alpine::getInstance().getCamera();
}

void
resetAccumulation()
{
    Alpine::getInstance().resetAccumulation();
}

void
render(uint32_t spp)
{
    Alpine::getInstance().render(spp);
}

const void*
getFrameBuffer()
{
    return Alpine::getInstance().getFrameBuffer();
}

void
saveImage(const char* filename)
{
    Alpine::getInstance().saveImage(filename);
}

void
addDebugScene()
{
    Alpine::getInstance().addDebugScene();
}
}