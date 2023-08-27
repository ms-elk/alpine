#include <alpine.h>

#include "camera.h"
#include "debug_scene.h"
#include "disk_light.h"
#include "file_loader.h"
#include "image.h"
#include "kernel.h"
#include "material.h"
#include "mesh.h"
#include "point_light.h"
#include "ray.h"
#include "scene.h"
#include "sampler.h"
#include "sphere.h"
#include "util.h"

#include <OpenImageDenoise/oidn.hpp>

#include <algorithm>
#include <future>
#include <memory>
#include <vector>

namespace alpine {
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

    bool load(std::string_view filename, FileType fileType);

    ILight* addPointLight(const float intensity[3], const float position[3]);

    ILight* addDiskLight(const float emission[3], const float position[3], float radius);

    void resetAccumulation();

    void render(uint32_t spp);

    void resolve(bool denoise);

    void saveImage(std::string_view filename) const;

    void addDebugScene();

private:
    Alpine()
    {
        kernel::initialize();
    }
    ~Alpine() { kernel::finalize(); }

    float3 estimateDirectIllumination(Sampler& sampler, const float3& hit,
        const float3& wo, const IntersectionAttributes& isectAttr) const;

    const std::pair<Light*, std::size_t /* count */> selectLight(float u) const;

private:
    uint32_t mWidth = 0;
    uint32_t mHeight = 0;
    uint32_t mMaxDepth = 0;

    float3 mBackgroundColor = float3(1.0f);

    struct RenderTarget
    {
        float3 color;
        float3 albedo;
        float3 normal;
    };

    std::vector<Sampler> mSamplers;
    std::vector<RenderTarget> mAccumBuffer;
    std::vector<RenderTarget> mResolvedBuffer;
    std::vector<float3> mDenoisedBuffer;
    std::vector<byte3> mFrameBuffer;
    uint32_t mTotalSamples = 0;

    uint32_t mTileWidth = 0;
    uint32_t mTileHeight = 0;
    std::vector<std::future<void>> mTiles;

    Scene mScene;

    Camera mCamera;

    oidn::DeviceRef mDenoiser;
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
    mResolvedBuffer.resize(pixelCount);
    mDenoisedBuffer.resize(pixelCount);
    mFrameBuffer.resize(pixelCount);

    mTileWidth = (width - 1) / TILE_SIZE + 1;
    mTileHeight = (height - 1) / TILE_SIZE + 1;
    uint32_t tileCount = mTileWidth * mTileHeight;
    mTiles.resize(tileCount);

    mDenoiser = oidn::newDevice();
    mDenoiser.commit();

    resetAccumulation();
}

bool
Alpine::load(std::string_view filename, FileType fileType)
{
    const auto load = [&]() {
        switch (fileType)
        {
        case FileType::GLTF:
            return loadGltf(&mScene, filename);
        case FileType::OBJ:
            return loadObj(&mScene, filename);
        default:
            return false;
        }
    };

    if (!load())
    {
        return false;
    }

    kernel::updateScene();

    return true;
}

ILight*
Alpine::addPointLight(const float intensity[3], const float position[3])
{
    mScene.lights.push_back(std::make_shared<PointLight>(float3(intensity), float3(position)));
    return mScene.lights.back().get();
}

ILight*
Alpine::addDiskLight(const float emission[3], const float position[3], float radius)
{
    mScene.lights.push_back(std::make_shared<DiskLight>(
        float3(emission), float3(position), normalize(float3(0.0, -1.0f, 0.0f)), radius));
    return mScene.lights.back().get();
}

void
Alpine::resetAccumulation()
{
    for (uint32_t p = 0; p < mSamplers.size(); ++p)
    {
        mSamplers[p].reset(p);
    }
    std::fill(mAccumBuffer.begin(), mAccumBuffer.end(), RenderTarget{float3(0.0f), float3(0.0f), float3(0.0f) });
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

                            if (depth == 0)
                            {
                                mAccumBuffer[index].albedo += isectAttr.material->getBaseColor(isectAttr.uv);
                                mAccumBuffer[index].normal += isectAttr.ns;
                            }

                            //radiance = float3(abs(isectAttr.ns.x), abs(isectAttr.ns.y), abs(isectAttr.ns.z));
                            //radiance = isectAttr.ns;
                            //radiance = float3(isectAttr.uv.x, isectAttr.uv.y, 0.0f);
                            //break;

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

    const auto [light, lightCount] = selectLight(sampler.get1D());

    if (lightCount == 0)
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
    auto ls = light->sample(sampler.get2D(), hit);
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
        auto [lightPdf, lightDist] = light->computePdf(hit, lightDir);
        if (lightPdf > 0.0f && !isOccluded(hit, isectAttr.ns, lightDir, lightDist))
        {
            float3 emission = light->getEmission();
            float misWeight = powerHeuristic(1, ms.pdf, 1, lightPdf);
            radiance += emission * misWeight * ms.estimator;
        }
    }

    return radiance * static_cast<float>(lightCount);
}

const std::pair<Light*, std::size_t /* count */>
Alpine::selectLight(float u) const
{
    std::size_t lightCount = std::count_if(mScene.lights.begin(), mScene.lights.end(),
        [](const auto& l) { return l->isEnabled(); });
    if (lightCount == 0)
    {
        return { nullptr, 0 };
    }

    std::size_t lightIdx = static_cast<std::size_t>(u * lightCount);
    lightIdx = std::min(lightIdx, lightCount - 1);

    Light* light = nullptr;
    std::size_t currentIdx = 0;
    for (auto& l : mScene.lights)
    {
        if (l->isEnabled())
        {
            if (currentIdx == lightIdx)
            {
                light = l.get();
            }
            currentIdx++;
        }
    }

    assert(light);
    return { light, lightCount };
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
        oidn::FilterRef filter = mDenoiser.newFilter("RT");
        filter.setImage("color", mResolvedBuffer.data(), oidn::Format::Float3, mWidth, mHeight, 0, sizeof(RenderTarget));
        filter.setImage("albedo", mResolvedBuffer.data(), oidn::Format::Float3, mWidth, mHeight, sizeof(float3), sizeof(RenderTarget));
        filter.setImage("normal", mResolvedBuffer.data(), oidn::Format::Float3, mWidth, mHeight, 2 * sizeof(float3), sizeof(RenderTarget));
        filter.setImage("output", mDenoisedBuffer.data(), oidn::Format::Float3, mWidth, mHeight, 0, sizeof(float3));
        filter.set("hdr", true);
        filter.commit();
        filter.execute();
    }

    for (uint32_t i = 0; i < mFrameBuffer.size(); ++i)
    {
        auto pixel = denoise ? mDenoisedBuffer[i] : mResolvedBuffer[i].color;

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
    kernel::updateScene();
}

///////////////////////////////////////////////////////////////
void
initialize(uint32_t width, uint32_t height, uint32_t maxDepth)
{
    Alpine::getInstance().initialize(width, height, maxDepth);
}

bool
load(std::string_view filename, FileType fileType)
{
    return Alpine::getInstance().load(filename, fileType);
}

ILight*
addPointLight(const float intensity[3], const float position[3])
{
    return Alpine::getInstance().addPointLight(intensity, position);
}

ILight*
addDiskLight(const float emission[3], const float position[3], float radius)
{
    return Alpine::getInstance().addDiskLight(emission, position, radius);
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

void
resolve(bool denoise)
{
    Alpine::getInstance().resolve(denoise);
}

const void*
getFrameBuffer()
{
    return Alpine::getInstance().getFrameBuffer();
}

void
saveImage(std::string_view filename)
{
    Alpine::getInstance().saveImage(filename);
}

void
addDebugScene()
{
    Alpine::getInstance().addDebugScene();
}
}