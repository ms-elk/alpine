#include <alpine.h>

#include "camera.h"
#include "debug_scene.h"
#include "image.h"
#include "kernel.h"
#include "light.h"
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

    float3 evaluateNextEventEstimation(Sampler& sampler, const float3& hit, const float3& wo, const IntersectionAttributes& isectAttr) const;

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
    mScene.light = std::make_shared<Light>(
        float3(10.0f), float3(278.0f, 548.7f, 227.0f), float3(0.0f, 1.0f, 0.0f), 100.0f); // TODO

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
                            radiance += throughput * evaluateNextEventEstimation(sampler, hit, wo, isectAttr);

                            auto ms = isectAttr.material->sample(
                                wo, sampler.get2D(), isectAttr);
                            float3 wi = toWorld(ms.wi, isectAttr.ss, isectAttr.ts, isectAttr.ns);

                            bool isReflect = dot(wi, isect.ng) > 0.0f;
                            if (ms.pdf == 0.0f || !isReflect)
                            {
                                break;
                            }

                            throughput = throughput * ms.estimator;

                            float3 rayOffset = isect.ng * 0.001f;
                            ray.org = hit + rayOffset;
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

namespace {
float powerHeuristic(float a, float b)
{
    float a2 = a * a;
    return a2 / (a2 + b * b);
}

float3
estimateDirectIllumination(float pdfA, float pdfB, const float3& position, const float3& normal,
    const float3& emission, const float3& lightDir, float lightDist, const float3& bsdf)
{
    if (pdfA <= 0.0f || pdfB <= 0.0f)
    {
        return float3(0.0f);
    }

    Ray shadowRay{ position, lightDir };
    bool occluded = kernel::occluded(shadowRay, lightDist);
    if (occluded)
    {
        return float3(0.0f);
    }

    float cosTerm = std::max(0.0f, dot(lightDir, normal));
    float misWeight = powerHeuristic(pdfA, pdfB);
    return emission * misWeight * bsdf * cosTerm / pdfA;
}
};

float3
Alpine::evaluateNextEventEstimation(
    Sampler& sampler, const float3& hit, const float3& wo, const IntersectionAttributes& isectAttr) const
{
    float3 radiance(0.0f);

    if (!mScene.light)
    {
        return radiance;
    }

    {
        auto ls = mScene.light->sample(sampler.get2D(), hit);

        float3 wi = toLocal(ls.wi, isectAttr.ss, isectAttr.ts, isectAttr.ns); // TODO: name
        float bsdfPdf = isectAttr.material->computePdf(wo, wi);
        float3 bsdf = isectAttr.material->evaluate(wo, wi, isectAttr);

        radiance += estimateDirectIllumination(
            ls.pdf, bsdfPdf, hit, isectAttr.ns, ls.emission, ls.wi, ls.distance, bsdf);
    }

    {
        auto ms = isectAttr.material->sample(wo, sampler.get2D(), isectAttr);
        float3 bsdf = isectAttr.material->evaluate(wo, ms.wi, isectAttr); // TODO

        Vector3 lightDir = toWorld(ms.wi, isectAttr.ss, isectAttr.ts, isectAttr.ns); // TODO: name
        auto [lightPdf, lightDist] = mScene.light->computePdf(hit, lightDir);
        auto emission = mScene.light->getEmission();

        radiance += estimateDirectIllumination(ms.pdf, lightPdf, hit, isectAttr.ns, emission, lightDir, lightDist, bsdf);
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