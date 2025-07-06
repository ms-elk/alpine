#include "embree.h"

#include <alpine_config.h>
#include <intersection.h>
#include <ray.h>

#include <assert.h>
#include <embree4/rtcore.h>
#include <stdio.h>

namespace alpine {
namespace {
void
errorFunction(void* userPtr, enum RTCError error, const char* str)
{
    printf("error %d: %s\n", error, str);
}
}

class Embree::Impl
{
public:
    Impl();
    ~Impl();

    uint32_t appendMesh(
        const std::vector<float3>& vertices,
        const std::vector<uint3>& prims,
        const void* ptr);

    void appendSphere(const std::vector<float4>& vertices, const void* ptr);

    void* getVertexBuffer(uint32_t shapeId);

    void updateShape(uint32_t shapeId);

    void updateScene();

    std::optional<Intersection> intersect(const Ray& ray) const;

    bool intersectAny(const Ray& ray, float tFar) const;

private:
    RTCDevice mDevice = nullptr;
    RTCScene mScene = nullptr;
    std::vector<const void*> mShapeTable;
};

Embree::Impl::Impl()
{
    mDevice = rtcNewDevice(nullptr);

    if (!mDevice)
    {
        printf("error %d: cannot create device\n", rtcGetDeviceError(nullptr));
        return;
    }

    rtcSetDeviceErrorFunction(mDevice, errorFunction, nullptr);

    mScene = rtcNewScene(mDevice);

    mShapeTable.reserve(MAX_SHAPES);
}

Embree::Impl::~Impl()
{
    if (mScene)
    {
        rtcReleaseScene(mScene);
        mScene = nullptr;
    }

    if (mDevice)
    {
        rtcReleaseDevice(mDevice);
        mDevice = nullptr;
    }
}

uint32_t
Embree::Impl::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{
    assert(mDevice && mScene);

    auto geom = rtcNewGeometry(mDevice, RTC_GEOMETRY_TYPE_TRIANGLE);

    float3* vertexBuffer = static_cast<float3*>(rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(float3), vertices.size()));
    memcpy(vertexBuffer, vertices.data(), sizeof(float3) * vertices.size());

    unsigned int* indexBuffer = static_cast<unsigned int*>(rtcSetNewGeometryBuffer(
        geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(uint3), prims.size()));
    memcpy(indexBuffer, prims.data(), sizeof(uint3) * prims.size());

    rtcCommitGeometry(geom);
    uint32_t shapeId = rtcAttachGeometry(mScene, geom);

    mShapeTable.push_back(ptr);

    return shapeId;
}

void
Embree::Impl::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{
    assert(mDevice && mScene);

    auto sphere = rtcNewGeometry(mDevice, RTC_GEOMETRY_TYPE_SPHERE_POINT);

    float4* vertexBuffer = static_cast<float4*>(rtcSetNewGeometryBuffer(
        sphere, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, sizeof(float4), vertices.size()));
    memcpy(vertexBuffer, vertices.data(), sizeof(float4) * vertices.size());

    rtcCommitGeometry(sphere);
    rtcAttachGeometry(mScene, sphere);

    mShapeTable.push_back(ptr);
}

void*
Embree::Impl::getVertexBuffer(uint32_t shapeId)
{
    auto geom = rtcGetGeometry(mScene, shapeId);
    void* vertexBuffer = rtcGetGeometryBufferData(geom, RTC_BUFFER_TYPE_VERTEX, 0);

    return vertexBuffer;
}

void
Embree::Impl::updateShape(uint32_t shapeId)
{
    auto geom = rtcGetGeometry(mScene, shapeId);
    rtcUpdateGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0);
    rtcCommitGeometry(geom);
}

void
Embree::Impl::updateScene()
{
    rtcCommitScene(mScene);
}

std::optional<Intersection>
Embree::Impl::intersect(const Ray& ray) const
{
    RTCRayHit rtcRayhit;
    rtcRayhit.ray.org_x = ray.org.x;
    rtcRayhit.ray.org_y = ray.org.y;
    rtcRayhit.ray.org_z = ray.org.z;
    rtcRayhit.ray.dir_x = ray.dir.x;
    rtcRayhit.ray.dir_y = ray.dir.y;
    rtcRayhit.ray.dir_z = ray.dir.z;
    rtcRayhit.ray.tnear = 0;
    rtcRayhit.ray.tfar = std::numeric_limits<float>::infinity();
    rtcRayhit.ray.mask = -1;
    rtcRayhit.ray.flags = 0;
    rtcRayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rtcRayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(mScene, &rtcRayhit);

    if (rtcRayhit.hit.geomID == RTC_INVALID_GEOMETRY_ID)
    {
        return {};
    }

    Intersection isect;
    isect.shapePtr = mShapeTable[rtcRayhit.hit.geomID];
    isect.primId = rtcRayhit.hit.primID;
    isect.ng = normalize(float3(rtcRayhit.hit.Ng_x, rtcRayhit.hit.Ng_y, rtcRayhit.hit.Ng_z));
    isect.barycentric.x = rtcRayhit.hit.u;
    isect.barycentric.y = rtcRayhit.hit.v;
    isect.t = rtcRayhit.ray.tfar;

    return isect;
}

bool
Embree::Impl::intersectAny(const Ray& ray, float tFar) const
{
    RTCRay rtcRay;
    rtcRay.org_x = ray.org.x;
    rtcRay.org_y = ray.org.y;
    rtcRay.org_z = ray.org.z;
    rtcRay.dir_x = ray.dir.x;
    rtcRay.dir_y = ray.dir.y;
    rtcRay.dir_z = ray.dir.z;
    rtcRay.tnear = 0;
    rtcRay.tfar = tFar;
    rtcRay.mask = -1;
    rtcRay.flags = 0;

    rtcOccluded1(mScene, &rtcRay);

    return rtcRay.tfar < tFar;
}

Embree::Embree()
    : mPimpl(std::make_unique<Impl>())
{}

Embree::~Embree() = default;

uint32_t
Embree::appendMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    const void* ptr)
{
    return mPimpl->appendMesh(vertices, prims, ptr);
}

void
Embree::appendSphere(const std::vector<float4>& vertices, const void* ptr)
{
    mPimpl->appendSphere(vertices, ptr);
}

void*
Embree::getVertexBuffer(uint32_t shapeId)
{
    return mPimpl->getVertexBuffer(shapeId);
}

void
Embree::updateShape(uint32_t shapeId)
{
    mPimpl->updateShape(shapeId);
}

void
Embree::updateScene()
{
    mPimpl->updateScene();
}

std::optional<Intersection>
Embree::intersect(const Ray& ray) const
{
    return mPimpl->intersect(ray);
}

bool
Embree::intersectAny(const Ray& ray, float tFar) const
{
    return mPimpl->intersectAny(ray, tFar);
}
}
