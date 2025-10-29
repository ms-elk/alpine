#include "debug_scene.h"

#include <materials/conductor.h>
#include <materials/dielectric.h>
#include <shapes/mesh.h>
#include <shapes/sphere.h>

namespace alpine {
std::shared_ptr<Mesh>
createDebugTriangle()
{
    Mesh::Data triData;

    triData.vertices.push_back(float3(-1.0f, 0.0f, 3.0f));
    triData.vertices.push_back(float3(0.0f, 1.0f, 3.0f));
    triData.vertices.push_back(float3(1.0f, 0.0f, 3.0f));
    triData.prims.push_back(uint3(0, 1, 2));
    triData.materials.push_back(std::make_shared<Dielectric>(
        float2(0.5f), float3(1.0f, 0.0f, 0.0f), nullptr, nullptr));

    return std::make_shared<Mesh>(std::move(triData), 0);
}

std::shared_ptr<Sphere>
createDebugSphere()
{
    auto sphereData = std::make_shared<Sphere::Data>();

    sphereData->vertices.push_back(float4(-0.75f, -0.75f, 0.0f, 0.5f));
    sphereData->materials.push_back(std::make_shared<Conductor>(
        float2(0.2f), float3(0.0f, 0.0f, 1.0f), nullptr, nullptr));

    sphereData->vertices.push_back(float4(0.75f, 0.75f, 0.0f, 0.3f));
    sphereData->materials.push_back(std::make_shared<Conductor>(
        float2(0.5f), float3(0.0f, 1.0f, 0.0f), nullptr, nullptr));

    return std::make_shared<Sphere>(sphereData);
}
}