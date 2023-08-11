#include "obj_converter.h"

#include "matrix.h"
#include "matte.h"
#include "mesh.h"
#include "metal.h"
#include "scene.h"
#include "shape.h"
#include "texture.h"

#include <filesystem>

//#define STB_IMAGE_IMPLEMENTATION
//#include <stb/stb_image.h>

//#define TINYOBJLOADER_IMPLEMENTATION
//#include <tinyobjloader/tiny_obj_loader.h>

#pragma warning(push)
#pragma warning(disable:4018)
#pragma warning(disable:4267)
#define __STDC_LIB_EXT1__
#define TINYGLTF_IMPLEMENTATION
#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <tinygltf/tiny_gltf.h>
#pragma warning(pop)

namespace alpine {
#if 0
static std::shared_ptr<Mesh>
convertObj(const char* filename)
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
        std::shared_ptr<Texture<float4>> diffuseTex = nullptr;
        if (!om.diffuse_texname.empty())
        {
            int32_t w, h, channels;
            std::string diffuseTexName = filepath.string() + om.diffuse_texname;
            stbi_uc* data = stbi_load(diffuseTexName.c_str(), &w, &h, &channels, STBI_rgb_alpha);

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
        }

        const auto& d = om.diffuse;
        //materials.push_back(std::make_shared<Metal>(float2(0.5f, 0.5f), float3(d[0], d[1], d[2]), diffuseTex));
        materials.push_back(std::make_shared<Matte>(float3(d[0], d[1], d[2]), diffuseTex));
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
#endif

template <typename T>
static void
appendBuffer(std::vector<T>& dst, uint32_t attrIdx, const tinygltf::Model& model)
{
    const auto& acc = model.accessors[attrIdx];
    const auto& bufView = model.bufferViews[acc.bufferView];
    const auto& buf = model.buffers[bufView.buffer];
    const auto* values = reinterpret_cast<const T*>(&buf.data[bufView.byteOffset + acc.byteOffset]);

    for (uint32_t i = 0; i < acc.count; ++i)
    {
        dst.emplace_back(values[i]);
    }
}

static std::size_t
appendIndexBuffer(std::vector<uint3>& dst, uint32_t attrIdx, const tinygltf::Model& model)
{
    const auto& acc = model.accessors[attrIdx];
    const auto& bufView = model.bufferViews[acc.bufferView];
    const auto& buf = model.buffers[bufView.buffer];
    const auto* values = reinterpret_cast<const uint16_t*>(&buf.data[bufView.byteOffset + acc.byteOffset]);

    for (uint32_t i = 0; i < acc.count; i += 3)
    {
        const auto* v = &values[i];
        dst.emplace_back(uint3(v[0], v[1], v[2]));
    }

    return acc.count / 3;
}

class GltfLoader
{
public:
    GltfLoader() {}

    bool load(Scene& scene, const char* filename);

private:
    void traverse(float4x4 matrix, uint32_t nodeIdx);

private:
    Scene* mScene;
    std::vector<std::shared_ptr<Material>> mMaterials;

    tinygltf::Model mSrcModel;
};

bool
GltfLoader::load(Scene& scene, const char* filename)
{
    mScene = &scene;

    tinygltf::TinyGLTF loader;
    std::string warn;
    std::string err;
    bool loaded = loader.LoadBinaryFromFile(&mSrcModel, &err, &warn, filename);

    if (!err.empty())
    {
        printf("%s", err.c_str());
        return false;
    }

    if (!warn.empty())
    {
        printf("%s", warn.c_str());
    }

    if (!loaded)
    {
        return false;
    }

    // extract materials from Obj
    mMaterials.reserve(mSrcModel.materials.size());
    for (const auto& srcMat : mSrcModel.materials)
    {
        std::shared_ptr<Texture<float4>> diffuseTex = nullptr;
        if (int32_t texIdx = srcMat.pbrMetallicRoughness.baseColorTexture.index; texIdx >= 0)
        {
            uint32_t imgIdx = mSrcModel.textures[texIdx].source;
            const auto& image = mSrcModel.images[imgIdx];

            if (image.component == 3 || image.component == 4)
            {
                std::vector<float4> texData(image.width * image.height);
                for (uint32_t i = 0; i < texData.size(); ++i)
                {
                    const uint8_t* d = &image.image[4 * i];
                    texData[i] = float4(d[0], d[1], d[2], d[3]) / 255.0f;
                }
                diffuseTex = std::make_shared<Texture<float4>>(image.width, image.height, std::move(texData));
            }
            else
            {
                printf("The channel count of %s is not valid\n", image.name.c_str());
            }
        }

        const auto& d = srcMat.pbrMetallicRoughness.baseColorFactor;
        float3 baseColor(static_cast<float>(d[0]), static_cast<float>(d[1]), static_cast<float>(d[2]));
        //materials.push_back(std::make_shared<Metal>(float2(0.5f, 0.5f), float3(d[0], d[1], d[2]), diffuseTex));
        mMaterials.push_back(std::make_shared<Matte>(baseColor, diffuseTex));
    }

    for (const auto& srcScene : mSrcModel.scenes)
    {
        for (uint32_t nodeIdx : srcScene.nodes)
        {
            traverse(float4x4(), nodeIdx);
        }
    }

    return true;
}

float4x4
getMatrix(const tinygltf::Node& node)
{
    float4x4 matrix;
    if (!node.matrix.empty())
    {
        const double* m = node.matrix.data();
        for (uint32_t i = 0; i < 16; ++i)
        {
            matrix[i] = static_cast<float>(m[i]);
        }
    }
    else
    {
        float4 qr = !node.rotation.empty()
            ? float4(
                static_cast<float>(node.rotation[0]),
                static_cast<float>(node.rotation[1]),
                static_cast<float>(node.rotation[2]),
                static_cast<float>(node.rotation[3]))
            : float4(0.0f, 0.0f, 0.0f, 1.0f);

        float3 s = !node.scale.empty()
            ? float3(
                static_cast<float>(node.scale[0]),
                static_cast<float>(node.scale[1]),
                static_cast<float>(node.scale[2]))
            : float3(1.0f);

        float3 t = !node.translation.empty()
            ? float3(
                static_cast<float>(node.translation[0]),
                static_cast<float>(node.translation[1]),
                static_cast<float>(node.translation[2]))
            : float3(0.0f);

        matrix[0] = (1.0f - 2.0f * qr.y * qr.y - 2.0f * qr.z * qr.z) * s.x;
        matrix[1] = (2.0f * qr.x * qr.y + 2.0f * qr.z * qr.w) * s.x;
        matrix[2] = (2.0f * qr.x * qr.z - 2.0f * qr.y * qr.w) * s.x;
        matrix[3] = 0.0f;

        matrix[4] = (2.0f * qr.x * qr.y - 2.0f * qr.z * qr.w) * s.y;
        matrix[5] = (1.0f - 2.0f * qr.x * qr.x - 2.0f * qr.z * qr.z) * s.y;
        matrix[6] = (2.0f * qr.y * qr.z + 2.0f * qr.x * qr.w) * s.y;
        matrix[7] = 0.0f;

        matrix[8] = (2.0f * qr.x * qr.z + 2.0f * qr.y * qr.w) * s.z;
        matrix[9] = (2.0f * qr.y * qr.z - 2.0f * qr.x * qr.w) * s.z;
        matrix[10] = (1.0f - 2.0f * qr.x * qr.x - 2.0f * qr.y * qr.y) * s.z;
        matrix[11] = 0.0f;

        matrix[12] = t.x;
        matrix[13] = t.y;
        matrix[14] = t.z;
        matrix[15] = 1.0f;
    }

    return matrix;
}

void
GltfLoader::traverse(float4x4 matrix, uint32_t nodeIdx)
{
    const auto& srcNode = mSrcModel.nodes[nodeIdx];

    matrix = mul(matrix, getMatrix(srcNode));

    if (srcNode.mesh >= 0)
    {
        Mesh::Data meshData;
        auto& srcMesh = mSrcModel.meshes[srcNode.mesh];

        for (auto& srcPrim : srcMesh.primitives)
        {
            assert(srcPrim.mode == TINYGLTF_MODE_TRIANGLES);

            appendBuffer(meshData.vertices, srcPrim.attributes["POSITION"], mSrcModel);
            appendBuffer(meshData.normals, srcPrim.attributes["NORMAL"], mSrcModel);
            appendBuffer(meshData.uvs, srcPrim.attributes["TEXCOORD_0"], mSrcModel);

            std::size_t primCount = appendIndexBuffer(meshData.prims, srcPrim.indices, mSrcModel);

            for (uint32_t i = 0; i < primCount; ++i)
            {
                meshData.materials.push_back(mMaterials[srcPrim.material]);
            }
        }

        assert(meshData.vertices.size() == meshData.normals.size());
        for (uint32_t i = 0; i < meshData.vertices.size(); ++i)
        {
            auto& v = meshData.vertices[i];
            v = mul(matrix, float4(v.x, v.y, v.z, 1.0f)).xyz();

            auto& n = meshData.normals[i];
            n = mul(matrix, float4(n.x, n.y, n.z, 0.0f)).xyz();
        }

        mScene->shapes.push_back(std::make_shared<Mesh>(std::move(meshData)));
    }

    for (uint32_t childIdx : srcNode.children)
    {
        traverse(matrix, childIdx);
    }
}

std::shared_ptr<Mesh>
createMesh(const char* filename)
{
    return nullptr;// convertGltf(filename);
}

bool
createMeshes(Scene& scene, const char* filename)
{
    GltfLoader loader;
    return loader.load(scene, filename);
}
}