#include "obj_converter.h"

#include "lambertian.h"
#include "mesh.h"
#include "microfacet.h"
#include "texture.h"

#include <filesystem>

#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>

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
    std::filesystem::path filepath(filename);
    filepath.remove_filename();
    bool loaded = tinyobj::LoadObj(&attrib, &objShapes, &objMaterials, &warn, &err,
        filename, filepath.string().c_str(), true);

    if (!err.empty())
    {
        printf("%s", err.c_str());
        return nullptr;
    }

    if (!warn.empty())
    {
        printf("%s", warn.c_str());
    }

    if (!loaded)
    {
        return nullptr;
    }

    // extract materials from Obj
    std::vector<std::shared_ptr<Material>> materials;
    materials.reserve(objMaterials.size());
    for (const auto& om : objMaterials)
    {
        if (!om.diffuse_texname.empty())
        {
            int32_t w, h, channels;
            std::string diffuseTexName = filepath.string() + om.diffuse_texname;
            stbi_uc* data = stbi_load(diffuseTexName.c_str(), &w, &h, &channels, STBI_rgb_alpha);

            std::shared_ptr<Texture<float4>> diffuseTex = nullptr;
            if (channels == 3 || channels == 4)
            {
                std::vector<float4> texData(w * h);
                for (uint32_t i = 0; i < texData.size(); ++i)
                {
                    const uint8_t* d = &data[4 * i];
                    texData[i] = float4(d[0], d[1], d[2], d[3]) / 255.0f;
                }
                diffuseTex = std::make_shared<Texture<float4>>(w, h, std::move(texData));
            }
            else
            {
                printf("The channel count of %s is not valid\n", om.diffuse_texname.c_str());
            }

            const auto& d = om.diffuse;
            materials.push_back(std::make_shared<Lambertian>(float3(d[0], d[1], d[2]), diffuseTex));
        }
        //else if (!om.specular_texname.empty())
        //{
        //}
        else
        {
            const auto& d = om.diffuse;
            materials.push_back(std::make_shared<Microfacet>(float2(0.5f, 0.5f), float3(d[0], d[1], d[2]), nullptr));
            //materials.push_back(std::make_shared<Lambertian>(float3(d[0], d[1], d[2]), nullptr));
        }
    }

    Mesh::Data meshData;

    // copy vertices from Obj
    size_t vertexCount = attrib.vertices.size() / 3;
    meshData.vertices.resize(vertexCount);
    memcpy(meshData.vertices.data(), attrib.vertices.data(), sizeof(float3) * vertexCount);

    size_t normalCount = attrib.normals.size() / 3;
    meshData.normals.resize(normalCount);
    memcpy(meshData.normals.data(), attrib.normals.data(), sizeof(float3) * normalCount);

    size_t uvCount = attrib.texcoords.size() / 2;
    meshData.uvs.resize(uvCount);
    memcpy(meshData.uvs.data(), attrib.texcoords.data(), sizeof(float2) * uvCount);

    // assign prim indices and material IDs
    for (const auto& shape : objShapes)
    {
        const auto& sm = shape.mesh;
        size_t primCount = sm.indices.size() / 3;

        for (uint32_t i = 0; i < primCount; ++i)
        {
            const auto* index = &sm.indices[3 * i];
            meshData.prims.push_back(
                uint3(index[0].vertex_index, index[1].vertex_index, index[2].vertex_index));
            meshData.normalPrims.push_back(
                uint3(index[0].normal_index, index[1].normal_index, index[2].normal_index));
            meshData.uvPrims.push_back(
                uint3(index[0].texcoord_index, index[1].texcoord_index, index[2].texcoord_index));

            meshData.materials.push_back(materials[sm.material_ids[i]]);
        }
    }

    return std::make_shared<Mesh>(std::move(meshData));
}
}