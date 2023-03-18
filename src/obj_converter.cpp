#include "obj_converter.h"

#include "lambertian.h"
#include "mesh.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include <tinyobjloader/tiny_obj_loader.h>

namespace alpine {
std::shared_ptr<Mesh>
createMesh(const char* filename)
{
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> objShapes;
    std::vector<tinyobj::material_t> objMaterials;
    std::string warn;
    std::string err;
    bool loaded = tinyobj::LoadObj(&attrib, &objShapes, &objMaterials, &warn, &err, filename, nullptr, true);
    if (!loaded)
    {
        return nullptr;
    }

    std::vector<std::shared_ptr<Material>> materials;
    materials.reserve(objMaterials.size());
    for (const auto& om : objMaterials)
    {
        const auto& d = om.diffuse;
        materials.push_back(std::make_shared<Lambertian>(float3(d[0], d[1], d[2])));
    }

    auto meshData = std::make_shared<Mesh::Data>();

    size_t vertexCount = attrib.vertices.size() / 3;
    meshData->vertices.resize(vertexCount);
    memcpy(meshData->vertices.data(), attrib.vertices.data(), sizeof(float3) * vertexCount);

    for (const auto& shape : objShapes)
    {
        const auto& sm = shape.mesh;
        size_t primCount = sm.indices.size() / 3;

        for (int i = 0; i < primCount; ++i)
        {
            const auto* index = &sm.indices[3*i];
            meshData->prims.push_back(uint3(index[0].vertex_index, index[1].vertex_index, index[2].vertex_index));

            meshData->materials.push_back(materials[sm.material_ids[i]]);
        }
    }

    return std::make_shared<Mesh>(meshData);
}
}