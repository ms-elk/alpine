#include <alpine.h>

#include "camera.h"
#include "debug_scene.h"
#include "kernel.h"
#include "material.h"
#include "mesh.h"
#include "obj_converter.h"
#include "ray.h"
#include "sampler.h"
#include "util.h"
#include "ppm.h"

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

    void initialize(int width, int height, int maxDepth);

    bool loadObj(const char* filename);

    void setCamera(
        const float eye[3],
        const float at[3],
        const float up[3],
        float fovy,
        float aspect);

    void render(int spp);

    void saveImage(const char* filename);

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

    std::vector<Vector3f> mFrameBuffer;
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
    mCamera.set(Vector3f(eye), Vector3f(at), Vector3f(up), fovy, aspect);
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
                Vector2f jitter = get2D();
                auto ray = mCamera.generateRay((x + jitter.x) / float(mWidth), (y + jitter.y) / float(mHeight));

                Vector3f throughput(1.0f, 1.0f, 1.0f);
                Vector3f radiance(0.0f, 0.0f, 0.0f);
                for (int depth = 0; depth < mMaxDepth; ++depth)
                {
                    auto isect = kernel::intersect(ray);

                    if (!isect.shapePtr)
                    {
                        const Vector3f backGroundColor(1.0f);
                        radiance += throughput * backGroundColor;
                        break;
                    }

                    const auto* shape = static_cast<Shape*>(isect.shapePtr);
                    auto isectAttr = shape->getIntersectionAttributes(ray, isect);

                    Vector3f wi;
                    float pdf;
                    Vector3 bsdf = isectAttr.material->sample(ray.dir, isect.ng, get2D(), wi, pdf);
                    if (pdf == 0.0f)
                    {
                        break;
                    }

                    float cosTerm = std::abs(dot(wi, isect.ng));
                    throughput = throughput * bsdf * cosTerm / pdf;

                    Vector3f rayOffset = isect.ng * 0.001f;
                    ray.org = ray.org + ray.dir * isect.t + rayOffset;
                    ray.dir = wi;
                }

                int index = y * mWidth + x;
                mFrameBuffer[index] += radiance;
            }
        }
    }

    for (auto& pixel : mFrameBuffer)
    {
        pixel /= float(spp);
    }
}

void
Alpine::saveImage(const char* filename)
{
    writePPM(filename, mWidth, mHeight, mFrameBuffer.data());
}

void
Alpine::addDebugScene()
{
    mScene.push_back(createDebugTriangle());
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
render(int spp)
{
    Alpine::getInstance().render(spp);
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