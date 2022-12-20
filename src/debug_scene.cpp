#include "debug_scene.h"

#include "lambertian.h"
#include "mesh.h"

namespace alpine {
std::shared_ptr<Mesh>
createDebugTriangle()
{
    auto triData = std::make_shared<Mesh::Data>();
    triData->vertices.push_back(Vector3f(-1.0f, 0.0f, 3.0f));
    triData->vertices.push_back(Vector3f(0.0f, 1.0f, 3.0f));
    triData->vertices.push_back(Vector3f(1.0f, 0.0f, 3.0f));
    triData->prims.push_back(Vector3ui(0, 1, 2));
    triData->materials.push_back(std::make_shared<Lambertian>(Vector3f(1.0f, 0.0f, 0.0f)));

    return std::make_shared<Mesh>(triData);
}
}