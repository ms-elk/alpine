#include <alpine/alpine.h>

#include "camera.h"
#include "image.h"
#include "accelerators/accelerator.h"
#include "denoisers/denoiser.h"
#include "light_samplers/light_sampler.h"
#include "light_samplers/bvh_light_sampler.h"
#include "light_samplers/power_light_sampler.h"
#include "light_samplers/uniform_light_sampler.h"
#include "lights/disk_light.h"
#include "lights/point_light.h"
#include "loaders/file_loader.h"
#include "materials/material.h"
#include "ray.h"
#include "sampler.h"
#include "scenes/debug_scene.h"
#include "scenes/scene.h"
#include "shapes/mesh.h"
#include "shapes/sphere.h"
#include "utils/util.h"

#include <algorithm>
#include <assert.h>
#include <future>
#include <memory>
#include <vector>

namespace alpine {
static constexpr uint32_t TILE_SIZE = 64;
static constexpr float RAY_OFFSET = 0.001f;

class Alpine
{
public:
    Alpine(uint32_t width, uint32_t height, uint32_t maxDepth);
    ~Alpine() { accelerator::finalize(); }

    inline void setBackgroundColor(float r, float g, float b)
    {
        mBackgroundColor = float3(r, g, b);
    }

    inline api::Camera* getCamera() { return &mCamera; }

    inline const void* getFrameBuffer() const { return mFrameBuffer.data(); }

    bool load(std::string_view filename, FileType fileType);

    api::Light* addPointLight(float power, const float color[3], const float position[3]);

    api::Light* addDiskLight(float power, const float color[3], const float position[3], float radius);

    void buildLightSampler(LightSamplerType lightSamplerType);

    void resetAccumulation();

    void render(uint32_t spp);

    void resolve(bool denoise);

    void saveImage(std::string_view filename) const;

    void addDebugScene();

private:
    float3 estimateDirectIllumination(Sampler& sampler, const float3& hit,
        const float3& wo, const IntersectionAttributes& isectAttr) const;

private:
    uint32_t mWidth = 0;
    uint32_t mHeight = 0;
    uint32_t mMaxDepth = 0;

    float3 mBackgroundColor = float3(1.0f);

    std::vector<Sampler> mSamplers;
    std::vector<denoiser::RenderTarget> mAccumBuffer;
    std::vector<denoiser::RenderTarget> mResolvedBuffer;
    std::vector<byte3> mFrameBuffer;
    uint32_t mTotalSamples = 0;

    uint32_t mTileWidth = 0;
    uint32_t mTileHeight = 0;
    std::vector<std::future<void>> mTiles;

    Scene mScene;

    Camera mCamera;

    std::unique_ptr<LightSampler> mLightSampler;
};

Alpine::Alpine(uint32_t width, uint32_t height, uint32_t maxDepth)
{
    accelerator::initialize();
    denoiser::initialize();

    mWidth = width;
    mHeight = height;
    mMaxDepth = maxDepth;

    uint32_t pixelCount = mWidth * mHeight;
    mSamplers.resize(pixelCount);
    mAccumBuffer.resize(pixelCount);
    mResolvedBuffer.resize(pixelCount);
    mFrameBuffer.resize(pixelCount);

    mTileWidth = (width - 1) / TILE_SIZE + 1;
    mTileHeight = (height - 1) / TILE_SIZE + 1;
    uint32_t tileCount = mTileWidth * mTileHeight;
    mTiles.resize(tileCount);

    resetAccumulation();
}

bool
Alpine::load(std::string_view filename, FileType fileType)
{
    const auto load = [&]() {
        switch (fileType)
        {
        case FileType::Gltf:
            return loadGltf(&mScene, filename);
        case FileType::Obj:
            return loadObj(&mScene, filename);
        default:
            return false;
        }
    };

    if (!load())
    {
        return false;
    }

    accelerator::updateScene();

    return true;
}

api::Light*
Alpine::addPointLight(float power, const float color[3], const float position[3])
{
    mScene.lights.push_back(std::make_shared<PointLight>(power, float3(color), float3(position)));
    return mScene.lights.back().get();
}

api::Light*
Alpine::addDiskLight(float power, const float color[3], const float position[3], float radius)
{
    mScene.lights.push_back(std::make_shared<DiskLight>(
        power, float3(color), float3(position), normalize(float3(0.0, -1.0f, 0.0f)), radius));
    return mScene.lights.back().get();
}

void
Alpine::buildLightSampler(LightSamplerType lightSamplerType)
{
    switch (lightSamplerType)
    {
    case LightSamplerType::Power:
        mLightSampler = std::make_unique<PowerLightSampler>(mScene.lights);
        break;
    case LightSamplerType::Bvh:
        mLightSampler = std::make_unique<BvhLightSampler>(mScene.lights);
        break;
    case LightSamplerType::Uniform:
    default:
        mLightSampler = std::make_unique<UniformLightSampler>(mScene.lights);
        break;
    }
}

void
Alpine::resetAccumulation()
{
    for (uint32_t p = 0; p < mSamplers.size(); ++p)
    {
        mSamplers[p].reset(p);
    }
    std::fill(mAccumBuffer.begin(), mAccumBuffer.end(),
        denoiser::RenderTarget{float3(0.0f), float3(0.0f), float3(0.0f) });
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
                    if (y >= mHeight)
                    {
                        continue;
                    }

                    for (uint32_t x = xBegin; x < xBegin + TILE_SIZE; ++x)
                    {
                        if (x >= mWidth)
                        {
                            continue;
                        }

                        uint32_t index = y * mWidth + x;
                        auto& sampler = mSamplers[index];
                        float2 jitter = sampler.get2D();
                        auto ray = mCamera.generateRay(
                            (x + jitter.x) / float(mWidth), (y + jitter.y) / float(mHeight));

                        float3 throughput(1.0f);
                        float3 radiance(0.0f);
                        for (uint32_t depth = 0; depth < mMaxDepth; ++depth)
                        {
                            auto isect = accelerator::intersect(ray);

                            if (!isect.shapePtr)
                            {
                                radiance += throughput * mBackgroundColor;
                                break;
                            }

                            float3 hit = ray.org + ray.dir * isect.t;
                            const auto* shape = static_cast<Shape*>(isect.shapePtr);
                            auto isectAttr
                                = shape->getIntersectionAttributes(isect);

                            if (depth == 0)
                            {
                                mAccumBuffer[index].albedo += isectAttr.material->getBaseColor(isectAttr.uv);
                                mAccumBuffer[index].normal += isectAttr.ns;
                            }

                            float3 wo = toLocal(- ray.dir, isectAttr.ss, isectAttr.ts, isectAttr.ns);
                            radiance += throughput * estimateDirectIllumination(sampler, hit, wo, isectAttr);

                            auto oms =
                                isectAttr.material->sample(wo, sampler.get2D(), isectAttr);
                            if (!oms.has_value())
                            {
                                break;
                            }

                            const auto& ms = oms.value();
                            float3 wiWorld = toWorld(ms.wi, isectAttr.ss, isectAttr.ts, isectAttr.ns);
                            bool isReflect = dot(wiWorld, isect.ng) > 0.0f;
                            if (!isReflect)
                            {
                                break;
                            }

                            throughput = throughput * ms.estimator;

                            float3 rayOffset = isect.ng * RAY_OFFSET;
                            ray.org = hit + rayOffset;
                            ray.dir = wiWorld;
                        }

                        mAccumBuffer[index].color += radiance;
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
}

float3
Alpine::estimateDirectIllumination(
    Sampler& sampler, const float3& hit, const float3& wo, const IntersectionAttributes& isectAttr) const
{
    float3 radiance(0.0f);

    if (!mLightSampler)
    {
        return radiance;
    }

    auto olss = mLightSampler->sample(sampler.get1D(), hit, isectAttr.ns);
    if (!olss.has_value())
    {
        return radiance;
    }

    const auto& lss = olss.value();

    const auto isOccluded = [](
        const float3& position, const float3& normal, const float3& dir, float dist) {
        float3 rayOffset = normal * RAY_OFFSET;
        Ray shadowRay{ position + rayOffset, dir };
        bool occluded = accelerator::occluded(shadowRay, dist);
        return occluded;
    };

    // Light sampling
    auto ols = lss.light->sample(sampler.get2D(), hit);
    if (ols.has_value())
    {
        const auto& ls = ols.value();
        float3 wi = toLocal(ls.wiWorld, isectAttr.ss, isectAttr.ts, isectAttr.ns);
        float bsdfPdf = isectAttr.material->computePdf(wo, wi);
        if (bsdfPdf > 0.0f && !isOccluded(hit, isectAttr.ns, ls.wiWorld, ls.distance))
        {
            float3 bsdf = isectAttr.material->computeBsdf(wo, wi, isectAttr);
            float cosTerm = std::max(0.0f, dot(ls.wiWorld, isectAttr.ns));
            float misWeight = !lss.light->isDelta() ? powerHeuristic(1, ls.pdf, 1, bsdfPdf) : 1.0f;
            radiance += ls.emittedRadiance * misWeight * bsdf * cosTerm / ls.pdf;
        }
    }

    // BSDF sampling
    if (!lss.light->isDelta())
    {
        auto oms = isectAttr.material->sample(wo, sampler.get2D(), isectAttr);
        if (oms.has_value())
        {
            const auto& ms = oms.value();
            float3 lightDir = toWorld(ms.wi, isectAttr.ss, isectAttr.ts, isectAttr.ns);
            auto [lightPdf, lightDist] = lss.light->computePdf(hit, lightDir);
            if (lightPdf > 0.0f && !isOccluded(hit, isectAttr.ns, lightDir, lightDist))
            {
                float3 emission = lss.light->getEmittedRadiance();
                float misWeight = powerHeuristic(1, ms.pdf, 1, lightPdf);
                radiance += emission * misWeight * ms.estimator;
            }
        }
    }

    return radiance / lss.pdf;
}

void
Alpine::resolve(bool denoise)
{
    for (uint32_t i = 0; i < mResolvedBuffer.size(); ++i)
    {
        auto pixel = mAccumBuffer[i];
        mResolvedBuffer[i].color = pixel.color / float(mTotalSamples);
        mResolvedBuffer[i].albedo = pixel.albedo / float(mTotalSamples);
        mResolvedBuffer[i].normal = pixel.normal / float(mTotalSamples);
    }

    if (denoise)
    {
        denoiser::denoise(mResolvedBuffer.data(), mWidth, mHeight);
    }

    for (uint32_t i = 0; i < mFrameBuffer.size(); ++i)
    {
        auto pixel = mResolvedBuffer[i].color;

        auto& fb = mFrameBuffer[i];
        fb.x = static_cast<uint8_t>(std::clamp(pixel.x, 0.0f, 1.0f) * 255.0f + 0.5f);
        fb.y = static_cast<uint8_t>(std::clamp(pixel.y, 0.0f, 1.0f) * 255.0f + 0.5f);
        fb.z = static_cast<uint8_t>(std::clamp(pixel.z, 0.0f, 1.0f) * 255.0f + 0.5f);
    }
}

void
Alpine::saveImage(std::string_view filename) const
{
    writePPM(filename, mWidth, mHeight, mFrameBuffer.data());
}

void
Alpine::addDebugScene()
{
    mScene.shapes.push_back(createDebugTriangle());
    mScene.shapes.push_back(createDebugSphere());
    accelerator::updateScene();
}

///////////////////////////////////////////////////////////////
std::unique_ptr<Alpine> gAlpine;

bool
initialize(uint32_t width, uint32_t height, uint32_t maxDepth)
{
    if (!gAlpine)
    {
        gAlpine = std::make_unique<Alpine>(width, height, maxDepth);
        return true;
    }
    else
    {
        return false;
    }
}

bool
load(std::string_view filename, FileType fileType)
{
    assert(gAlpine);
    return gAlpine->load(filename, fileType);
}

api::Light*
addPointLight(float power, const float color[3], const float position[3])
{
    assert(gAlpine);
    return gAlpine->addPointLight(power, color, position);
}

api::Light*
addDiskLight(float power, const float color[3], const float position[3], float radius)
{
    assert(gAlpine);
    return gAlpine->addDiskLight(power, color, position, radius);
}

void
buildLightSampler(LightSamplerType lightSamplerType)
{
    assert(gAlpine);
    gAlpine->buildLightSampler(lightSamplerType);
}

void
setBackgroundColor(float r, float g, float b)
{
    assert(gAlpine);
    gAlpine->setBackgroundColor(r, g, b);
}

api::Camera*
getCamera()
{
    assert(gAlpine);
    return gAlpine->getCamera();
}

void
resetAccumulation()
{
    assert(gAlpine);
    gAlpine->resetAccumulation();
}

void
render(uint32_t spp)
{
    assert(gAlpine);
    gAlpine->render(spp);
}

void
resolve(bool denoise)
{
    assert(gAlpine);
    gAlpine->resolve(denoise);
}

const void*
getFrameBuffer()
{
    assert(gAlpine);
    return gAlpine->getFrameBuffer();
}

void
saveImage(std::string_view filename)
{
    assert(gAlpine);
    gAlpine->saveImage(filename);
}

void
addDebugScene()
{
    assert(gAlpine);
    gAlpine->addDebugScene();
}
}
