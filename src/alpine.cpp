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
#include <memory>
#include <vector>

namespace alpine {
static constexpr unsigned int MAX_SHAPES = 10;

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

    inline void setBackgroundColor(float r, float g, float b) { mBackgroundColor = float3(r, g, b); }
    inline const void* getFrameBuffer() const { return mFrameBuffer.data(); }

    void initialize(int width, int height, int maxDepth);

    bool loadObj(const char* filename);

    void setCamera(
        const float eye[3],
        const float at[3],
        const float up[3],
        float fovy,
        float aspect);

    void rotateCamera(float theta, float phi);

    void resetAccumulation();

    void render(int spp);

    void saveImage(const char* filename) const;

    void addDebugScene();

private:
    Alpine()
    {
        kernel::initialize();
    }
    ~Alpine() { kernel::finalize(); }

private:
    int mWidth = 0;
    int mHeight = 0;
    int mMaxDepth = 0;

    float3 mBackgroundColor = float3(1.0f);

    std::vector<float3> mAccumBuffer;
    std::vector<byte3> mFrameBuffer;
    int mTotalSamples = 0;

    std::vector<std::shared_ptr<Shape>> mScene;

    Camera mCamera;
};

void
Alpine::initialize(int width, int height, int maxDepth)
{
    mWidth = width;
    mHeight = height;
    mMaxDepth = maxDepth;
    int pixelCount = mWidth * mHeight;
    mAccumBuffer.resize(pixelCount);
    mFrameBuffer.resize(pixelCount);
    mScene.reserve(MAX_SHAPES);
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
Alpine::setCamera(
    const float eye[3],
    const float at[3],
    const float up[3],
    float fovy,
    float aspect)
{
    mCamera.set(float3(eye), float3(at), float3(up), fovy, aspect);
}

void
Alpine::rotateCamera(float theta, float phi)
{
    mCamera.rotate(theta, phi);
}

void
Alpine::resetAccumulation()
{
    mTotalSamples = 0;
    std::fill(mAccumBuffer.begin(), mAccumBuffer.end(), float3(0.0f));
}

void
Alpine::render(int spp)
{
    for (int sampleId = 0; sampleId < spp; ++sampleId)
    {
        for (int y = 0; y < mHeight; ++y)
        {
            for (int x = 0; x < mWidth; ++x)
            {
                float2 jitter = get2D();
                auto ray = mCamera.generateRay((x + jitter.x) / float(mWidth), (y + jitter.y) / float(mHeight));

                float3 throughput(1.0f, 1.0f, 1.0f);
                float3 radiance(0.0f, 0.0f, 0.0f);
                for (int depth = 0; depth < mMaxDepth; ++depth)
                {
                    auto isect = kernel::intersect(ray);

                    if (!isect.shapePtr)
                    {
                        radiance += throughput * mBackgroundColor;
                        break;
                    }

                    const auto* shape = static_cast<Shape*>(isect.shapePtr);
                    auto isectAttr = shape->getIntersectionAttributes(ray, isect);

                    float3 wi;
                    float pdf;
                    float3 bsdf = isectAttr.material->sample(ray.dir, isect.ng, get2D(), wi, pdf);
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

                int index = y * mWidth + x;
                mAccumBuffer[index] += radiance;
            }
        }
    }
    mTotalSamples += spp;

    for (int i = 0; i < mFrameBuffer.size(); ++i)
    {
        auto pixel = mAccumBuffer[i];
        pixel /= float(mTotalSamples);

        auto& fb = mFrameBuffer[i];
        fb.x = static_cast<char>(std::clamp(pixel.x, 0.0f, 1.0f) * 255.0f + 0.5f);
        fb.y = static_cast<char>(std::clamp(pixel.y, 0.0f, 1.0f) * 255.0f + 0.5f);
        fb.z = static_cast<char>(std::clamp(pixel.z, 0.0f, 1.0f) * 255.0f + 0.5f);
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
initialize(int width, int height, int maxDepth)
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

void
setCamera(
    const float eye[3],
    const float at[3],
    const float up[3],
    float fovy,
    float aspect)
{
    Alpine::getInstance().setCamera(eye, at, up, fovy, aspect);
}

void
rotateCamera(float theta, float phi)
{
    Alpine::getInstance().rotateCamera(theta, phi);
}

void
resetAccumulation()
{
    Alpine::getInstance().resetAccumulation();
}

void
render(int spp)
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