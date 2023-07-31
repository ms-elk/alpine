﻿#include "debug_scene.h"

#include "matte.h"
#include "mesh.h"
#include "metal.h"
#include "sphere.h"

namespace alpine {
std::shared_ptr<Mesh>
createDebugTriangle()
{
    Mesh::Data triData;

    triData.vertices.push_back(float3(-1.0f, 0.0f, 3.0f));
    triData.vertices.push_back(float3(0.0f, 1.0f, 3.0f));
    triData.vertices.push_back(float3(1.0f, 0.0f, 3.0f));
    triData.prims.push_back(uint3(0, 1, 2));
    triData.materials.push_back(std::make_shared<Matte>(float3(1.0f, 0.0f, 0.0f), nullptr));

    return std::make_shared<Mesh>(std::move(triData));
}

std::shared_ptr<Sphere>
createDebugSphere()
{
    auto sphereData = std::make_shared<Sphere::Data>();

    sphereData->vertices.push_back(float4(-0.75f, -0.75f, 0.0f, 0.5f));
    sphereData->materials.push_back(std::make_shared<Matte>(float3(0.0f, 0.0f, 1.0f), nullptr));

    sphereData->vertices.push_back(float4(0.75f, 0.75f, 0.0f, 0.3f));
    sphereData->materials.push_back(std::make_shared<Matte>(float3(0.0f, 1.0f, 0.0f), nullptr));

    return std::make_shared<Sphere>(sphereData);
}
}