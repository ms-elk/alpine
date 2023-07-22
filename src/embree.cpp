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

    rtcIntersect1(gScene, &context, &rtcRayhit);

    Intersection isect;
    if (rtcRayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID)
    {
        auto geom = rtcGetGeometry(gScene, rtcRayhit.hit.geomID);

        // retrieve the pointer of the intersected shape
        isect.shapePtr = rtcGetGeometryUserData(geom);
    }
    else
    {
        isect.shapePtr = nullptr;
    }
    isect.primId = rtcRayhit.hit.primID;
    isect.ng = normalize(float3(rtcRayhit.hit.Ng_x, rtcRayhit.hit.Ng_y, rtcRayhit.hit.Ng_z));
    isect.barycentric.x = rtcRayhit.hit.u;
    isect.barycentric.y = rtcRayhit.hit.v;
    isect.t = rtcRayhit.ray.tfar;

    return isect;
}

bool
occluded(const Ray& ray, float far)
{
    RTCIntersectContext context;
    rtcInitIntersectContext(&context);

    RTCRay rtcRay;
    rtcRay.org_x = ray.org.x;
    rtcRay.org_y = ray.org.y;
    rtcRay.org_z = ray.org.z;
    rtcRay.dir_x = ray.dir.x;
    rtcRay.dir_y = ray.dir.y;
    rtcRay.dir_z = ray.dir.z;
    rtcRay.tnear = 0;
    rtcRay.tfar = far;
    rtcRay.mask = -1;
    rtcRay.flags = 0;

    rtcOccluded1(gScene, &context, &rtcRay);

    return rtcRay.tfar < far;
}
}
