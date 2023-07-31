﻿#include <alpine.h>

#include "camera.h"
#include "debug_scene.h"
#include "disk_light.h"
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
static constexpr uint32_t MAX_SHAPES = 16;
static constexpr uint32_t TILE_SIZE = 64;
static constexpr float RAY_OFFSET = 0.001f;

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

    void setLight(const float emission[3], const float position[3], float radius);

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

    float3 estimateDirectIllumination(Sampler& sampler, const float3& hit,
        const float3& wo, const IntersectionAttributes& isectAttr) const;

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

    struct Scene
    {
        std::vector<std::shared_ptr<Shape>> shapes;
        std::shared_ptr<Light> light;
    };
    Scene mScene;

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

    mScene.shapes.reserve(MAX_SHAPES);

    resetAccumulation();
}

bool
Alpine::loadObj(const char* filename)
{
    if (mScene.shapes.size() >= MAX_SHAPES)
    {
        return false;
    }

    mScene.shapes.push_back(createMesh(filename));
    if (!mScene.shapes.back())
    {
        mScene.shapes.pop_back();
        return false;
    }

    kernel::updateScene();

    return true;
}

void
Alpine::setLight(const float emission[3], const float position[3], float radius)
{
    mScene.light = std::make_shared<DiskLight>(
        float3(emission), float3(position), normalize(float3(0.0, -1.0f, 0.0f)), radius);
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

                            float3 hit = ray.org + ray.dir * isect.t;
                            const auto* shape = static_cast<Shape*>(isect.shapePtr);
                            auto isectAttr
                                = shape->getIntersectionAttributes(isect);

                            float3 wo = toLocal(- ray.dir, isectAttr.ss, isectAttr.ts, isectAttr.ns);
                            radiance += throughput * estimateDirectIllumination(sampler, hit, wo, isectAttr);

                            auto ms =
                                isectAttr.material->sample(wo, sampler.get2D(), isectAttr);
                            float3 wiWorld = toWorld(ms.wi, isectAttr.ss, isectAttr.ts, isectAttr.ns);

                            bool isReflect = dot(wiWorld, isect.ng) > 0.0f;
                            if (ms.pdf == 0.0f || !isReflect)
                            {
                                break;
                            }

                            throughput = throughput * ms.estimator;

                            float3 rayOffset = isect.ng * RAY_OFFSET;
                            ray.org = hit + rayOffset;
                            ray.dir = wiWorld;
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

float3
Alpine::estimateDirectIllumination(
    Sampler& sampler, const float3& hit, const float3& wo, const IntersectionAttributes& isectAttr) const
{
    float3 radiance(0.0f);

    if (!mScene.light)
    {
        return radiance;
    }

    const auto isOccluded = [](
        const float3& position, const float3& normal, const float3& dir, float dist) {
        float3 rayOffset = normal * RAY_OFFSET;
        Ray shadowRay{ position + rayOffset, dir };
        bool occluded = kernel::occluded(shadowRay, dist);
        return occluded;
    };

    // Light sampling
    auto ls = mScene.light->sample(sampler.get2D(), hit);
    if (ls.pdf > 0.0f)
    {
        float3 wi = toLocal(ls.wiWorld, isectAttr.ss, isectAttr.ts, isectAttr.ns);
        float bsdfPdf = isectAttr.material->computePdf(wo, wi);
        if (bsdfPdf > 0.0f && !isOccluded(hit, isectAttr.ns, ls.wiWorld, ls.distance))
        {
            float3 bsdf = isectAttr.material->computeBsdf(wo, wi, isectAttr);
            float cosTerm = std::max(0.0f, dot(ls.wiWorld, isectAttr.ns));
            float misWeight = powerHeuristic(1, ls.pdf, 1, bsdfPdf);
            radiance += ls.emission * misWeight * bsdf * cosTerm / ls.pdf;
        }
    }

    // BSDF sampling
    auto ms = isectAttr.material->sample(wo, sampler.get2D(), isectAttr);
    if (ms.pdf > 0.0f)
    {
        float3 lightDir = toWorld(ms.wi, isectAttr.ss, isectAttr.ts, isectAttr.ns);
        auto [lightPdf, lightDist] = mScene.light->computePdf(hit, lightDir);
        if (lightPdf > 0.0f && !isOccluded(hit, isectAttr.ns, lightDir, lightDist))
        {
            float3 emission = mScene.light->getEmission();
            float misWeight = powerHeuristic(1, ms.pdf, 1, lightPdf);
            radiance += emission * misWeight * ms.estimator;
        }
    }

    return radiance;
}

void
Alpine::saveImage(const char* filename) const
{
    writePPM(filename, mWidth, mHeight, mFrameBuffer.data());
}

void
Alpine::addDebugScene()
{
    mScene.shapes.push_back(createDebugTriangle());
    mScene.shapes.push_back(createDebugSphere());
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
setLight(const float emission[3], const float position[3], float radius)
{
    Alpine::getInstance().setLight(emission, position, radius);
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