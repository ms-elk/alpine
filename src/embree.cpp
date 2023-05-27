#include "kernel.h"

#include "ray.h"
#include <embree3/rtcore.h>
#include <stdio.h>

namespace alpine::kernel {
RTCDevice gDevice = nullptr;
RTCScene gScene = nullptr;

void
errorFunction(void* userPtr, enum RTCError error, const char* str)
{
    printf("error %d: %s\n", error, str);
}

bool
initialize()
{
    gDevice = rtcNewDevice(nullptr);

    if (!gDevice)
    {
        printf("error %d: cannot create device\n", rtcGetDeviceError(nullptr));
        return false;
    }

    rtcSetDeviceErrorFunction(gDevice, errorFunction, nullptr);

    gScene = rtcNewScene(gDevice);

    return true;
}

void
finalize()
{
    if (gScene)
    {
        rtcReleaseScene(gScene);
        gScene = nullptr;
    }

    if (gDevice)
    {
        rtcReleaseDevice(gDevice);
        gDevice = nullptr;
    }
}

bool
createMesh(
    const std::vector<float3>& vertices,
    const std::vector<uint3>& prims,
    void* ptr)
{
    if (!gDevice || !gScene)
    {
        return false;
    }

    auto mesh = rtcNewGeometry(gDevice, RTC_GEOMETRY_TYPE_TRIANGLE);

    // register the pointer of the mesh
    rtcSetGeometryUserData(mesh, ptr);

    float3* vertexBuffer = (float3*)rtcSetNewGeometryBuffer(
        mesh, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(float3), vertices.size());
    memcpy(vertexBuffer, vertices.data(), sizeof(float3) * vertices.size());

    unsigned int* indexBuffer = (unsigned int*)rtcSetNewGeometryBuffer(
        mesh, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(uint3), prims.size());
    memcpy(indexBuffer, prims.data(), sizeof(uint3) * prims.size());

    rtcCommitGeometry(mesh);
    rtcAttachGeometry(gScene, mesh);
    rtcReleaseGeometry(mesh);

    return true;
}

bool
createSphere(const std::vector<float4>& vertices, void* ptr)
{
    if (!gDevice || !gScene)
    {
        return false;
    }

    auto sphere = rtcNewGeometry(gDevice, RTC_GEOMETRY_TYPE_SPHERE_POINT);

    // register the pointer of the mesh
    rtcSetGeometryUserData(sphere, ptr);

    float4* vertexBuffer = (float4*)rtcSetNewGeometryBuffer(
        sphere, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT4, sizeof(float4), vertices.size());
    memcpy(vertexBuffer, vertices.data(), sizeof(float4) * vertices.size());

    rtcCommitGeometry(sphere);
    rtcAttachGeometry(gScene, sphere);
    rtcReleaseGeometry(sphere);

    return true;
}

void
updateScene()
{
    rtcCommitScene(gScene);
}

Intersection
intersect(const Ray& ray)
{
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    RTCRayHit rayhit;
    rayhit.ray.org_x = ray.org.x;
    rayhit.ray.org_y = ray.org.y;
    rayhit.ray.org_z = ray.org.z;
    rayhit.ray.dir_x = ray.dir.x;
    rayhit.ray.dir_y = ray.dir.y;
    rayhit.ray.dir_z = ray.dir.z;
    rayhit.ray.tnear = 0;
    rayhit.ray.tfar = std::numeric_limits<float>::infinity();
    rayhit.ray.mask = -1;
    rayhit.ray.flags = 0;
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;

    rtcIntersect1(gScene, &context, &rayhit);

    Intersection isect;
    if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
    {
        auto geom = rtcGetGeometry(gScene, rayhit.hit.geomID);

        // retrieve the pointer of the intersected shape
        isect.shapePtr = rtcGetGeometryUserData(geom);
    }
    else
    {
        isect.shapePtr = nullptr;
    }
    isect.primId = rayhit.hit.primID;
    isect.ng = normalize(float3(rayhit.hit.Ng_x, rayhit.hit.Ng_y, rayhit.hit.Ng_z));
    isect.barycentric.x = rayhit.hit.u;
    isect.barycentric.y = rayhit.hit.v;
    isect.t = rayhit.ray.tfar;

    return isect;
}
}