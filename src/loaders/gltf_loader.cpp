#include "file_loader.h"

#include "lights/point_light.h"
#include "materials/matte.h"
#include "materials/metal.h"
#include "math/matrix.h"
#include "scenes/scene.h"
#include "shapes/mesh.h"
#include "texture.h"

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
namespace {
class GltfLoader
{
public:
    GltfLoader(Scene* scene) : mScene(scene) {}

    bool load(std::string_view filename);

private:
    void traverse(const float4x4& parent, uint32_t nodeIdx);

    std::shared_ptr<Mesh> createMesh(const float4x4& matrix, uint32_t meshIdx) const;

    template <typename T>
    std::size_t appendVertexBuffer(
        std::vector<T>& dst, const std::string& name, const tinygltf::Primitive& srcPrim) const;

    std::size_t appendTangentBuffer(
        std::vector<float3>& tangents,
        std::vector<float3>& bitangents,
        const std::vector<float3>& normals,
        const tinygltf::Primitive& srcPrim) const;

    std::size_t appendIndexBuffer(std::vector<uint3>& dst, const tinygltf::Primitive& srcPrim) const;

    std::shared_ptr<Material> createMaterial(uint32_t matIdx) const;

    std::shared_ptr<Texture4f> createTexture(uint32_t texIdx, bool isSigned) const;

    std::shared_ptr<Light> createLight(const float4x4& matrix, uint32_t lightIdx) const;

private:
    Scene* mScene = nullptr;
    std::vector<std::shared_ptr<Material>> mMaterials;

    tinygltf::Model mSrcModel;
};

bool
GltfLoader::load(std::string_view filename)
{
    tinygltf::TinyGLTF loader;
    std::string warn;
    std::string err;
    bool loaded = loader.LoadBinaryFromFile(&mSrcModel, &err, &warn, filename.data());

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

    // extract materials
    mMaterials.reserve(mSrcModel.materials.size());
    for (uint32_t matIdx = 0; matIdx < mSrcModel.materials.size(); ++matIdx)
    {
        mMaterials.push_back(createMaterial(matIdx));
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
GltfLoader::traverse(const float4x4& parent, uint32_t nodeIdx)
{
    const auto& srcNode = mSrcModel.nodes[nodeIdx];

    float4x4 matrix = mul(parent, getMatrix(srcNode));

    if (srcNode.mesh >= 0)
    {
        mScene->shapes.push_back(createMesh(matrix, srcNode.mesh));
    }

    if (srcNode.light >= 0)
    {
        mScene->lights.push_back(createLight(matrix, srcNode.light));
    }

    for (uint32_t childIdx : srcNode.children)
    {
        traverse(matrix, childIdx);
    }
}

std::shared_ptr<Mesh>
GltfLoader::createMesh(const float4x4& matrix, uint32_t meshIdx) const
{
    Mesh::Data meshData;
    const auto& srcMesh = mSrcModel.meshes[meshIdx];

    for (const auto& srcPrim : srcMesh.primitives)
    {
        assert(srcPrim.mode == TINYGLTF_MODE_TRIANGLES);
        assert(srcPrim.material >= 0);

        appendVertexBuffer(meshData.vertices, "POSITION", srcPrim);
        appendVertexBuffer(meshData.normals, "NORMAL", srcPrim);
        appendVertexBuffer(meshData.uvs, "TEXCOORD_0", srcPrim);

        appendTangentBuffer(
            meshData.tangents, meshData.bitangents, meshData.normals, srcPrim);

        std::size_t primCount = appendIndexBuffer(meshData.prims, srcPrim);
        for (uint32_t i = 0; i < primCount; ++i)
        {
            meshData.materials.push_back(mMaterials[srcPrim.material]);
        }
    }

    // transform
    // TODO: fix matrix for normals
    assert(meshData.vertices.size() == meshData.normals.size());
    for (uint32_t i = 0; i < meshData.vertices.size(); ++i)
    {
        auto& v = meshData.vertices[i];
        v = mul(matrix, float4(v.x, v.y, v.z, 1.0f)).xyz();

        auto& n = meshData.normals[i];
        n = mul(matrix, float4(n.x, n.y, n.z, 0.0f)).xyz();

        if (!meshData.tangents.empty())
        {
            auto& t = meshData.tangents[i];
            t = mul(matrix, float4(t.x, t.y, t.z, 0.0f)).xyz();

            assert(!meshData.bitangents.empty());
            auto& b = meshData.bitangents[i];
            b = mul(matrix, float4(b.x, b.y, b.z, 0.0f)).xyz();
        }
    }

    return std::make_shared<Mesh>(std::move(meshData));
}

template <typename T>
std::size_t
GltfLoader::appendVertexBuffer(
    std::vector<T>& dst, const std::string& name, const tinygltf::Primitive& srcPrim) const
{
    const auto itr = srcPrim.attributes.find(name);
    if (itr == srcPrim.attributes.end())
    {
        return 0;
    }

    const auto& acc = mSrcModel.accessors[itr->second];
    const auto& bufView = mSrcModel.bufferViews[acc.bufferView];
    const auto& buf = mSrcModel.buffers[bufView.buffer];
    const auto* values = reinterpret_cast<const T*>(&buf.data[bufView.byteOffset + acc.byteOffset]);

    assert(acc.componentType == TINYGLTF_COMPONENT_TYPE_FLOAT);

    for (uint32_t i = 0; i < acc.count; ++i)
    {
        dst.emplace_back(values[i]);
    }

    return acc.count;
}

std::size_t
GltfLoader::appendTangentBuffer(
    std::vector<float3>& tangents,
    std::vector<float3>& bitangents,
    const std::vector<float3>& normals,
    const tinygltf::Primitive& srcPrim) const
{
    const auto itr = srcPrim.attributes.find("TANGENT");
    if (itr == srcPrim.attributes.end())
    {
        return 0;
    }

    assert(!normals.empty());

    const auto& acc = mSrcModel.accessors[itr->second];
    const auto& bufView = mSrcModel.bufferViews[acc.bufferView];
    const auto& buf = mSrcModel.buffers[bufView.buffer];
    const auto* values = reinterpret_cast<const float4*>(&buf.data[bufView.byteOffset + acc.byteOffset]);

    for (uint32_t i = 0; i < acc.count; ++i)
    {
        const float3& normal = normals[i];
        float4 tangent = values[i];
        float3 bitangent = cross(normal, tangent.xyz()) * tangent.w;

        tangents.push_back(tangent.xyz());
        bitangents.push_back(bitangent);
    }

    return acc.count;
}

std::size_t
GltfLoader::appendIndexBuffer(std::vector<uint3>& dst, const tinygltf::Primitive& srcPrim) const
{
    const auto append = [&](const auto& values, std::size_t primCount) {
        for (uint32_t i = 0; i < primCount; ++i)
        {
            const auto* v = &values[3 * i];
            dst.emplace_back(uint3(v[0], v[1], v[2]));
        }
    };

    const auto& acc = mSrcModel.accessors[srcPrim.indices];
    const auto& bufView = mSrcModel.bufferViews[acc.bufferView];
    const auto& buf = mSrcModel.buffers[bufView.buffer];

    std::size_t primCount = acc.count / 3;

    if (acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT)
    {
        const auto* values =
            reinterpret_cast<const uint32_t*>(&buf.data[bufView.byteOffset + acc.byteOffset]);
        append(values, primCount);
    }
    else
    {
        assert(acc.componentType == TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT);
        const auto* values =
            reinterpret_cast<const uint16_t*>(&buf.data[bufView.byteOffset + acc.byteOffset]);
        append(values, primCount);
    }

    return primCount;
}

std::shared_ptr<Material>
GltfLoader::createMaterial(uint32_t matIdx) const
{
    const auto& srcMat = mSrcModel.materials[matIdx];

    std::shared_ptr<Texture4f> baseColorTex = nullptr;
    if (int32_t texIdx = srcMat.pbrMetallicRoughness.baseColorTexture.index; texIdx >= 0)
    {
        baseColorTex = createTexture(texIdx, false);
    }

    std::shared_ptr<Texture4f> normalTex = nullptr;
    if (int32_t texIdx = srcMat.normalTexture.index; texIdx >= 0)
    {
        normalTex = createTexture(texIdx, true);
    }

    const auto& d = srcMat.pbrMetallicRoughness.baseColorFactor;
    float3 baseColor(static_cast<float>(d[0]), static_cast<float>(d[1]), static_cast<float>(d[2]));

    if (srcMat.pbrMetallicRoughness.metallicFactor > 0.0)
    {
        float roughness = static_cast<float>(srcMat.pbrMetallicRoughness.roughnessFactor);
        return std::make_shared<Metal>(
            float2(roughness), baseColor, baseColorTex, normalTex);
    }
    else
    {
        return std::make_shared<Matte>(baseColor, baseColorTex, normalTex);
    }
}

std::shared_ptr<Texture4f>
GltfLoader::createTexture(uint32_t texIdx, bool isSigned) const
{
    uint32_t imgIdx = mSrcModel.textures[texIdx].source;
    const auto& srcImage = mSrcModel.images[imgIdx];

    if (srcImage.component == 3 || srcImage.component == 4)
    {
        std::vector<float4> texData(srcImage.width * srcImage.height);
        for (uint32_t i = 0; i < texData.size(); ++i)
        {
            if (srcImage.component == 3)
            {
                const uint8_t* d = &srcImage.image[3 * i];
                texData[i] = float4(d[0], d[1], d[2], 255) / 255.0f;
            }
            else
            {
                const uint8_t* d = &srcImage.image[4 * i];
                texData[i] = float4(d[0], d[1], d[2], d[3]) / 255.0f;
            }

            if (isSigned)
            {
                texData[i] = texData[i] * 2.0f - 1.0f;
            }
        }

        return std::make_shared<Texture4f>(srcImage.width, srcImage.height, std::move(texData));
    }
    else
    {
        printf("The channel count of %s is not valid\n", srcImage.name.c_str());
        return nullptr;
    }
}

std::shared_ptr<Light>
GltfLoader::createLight(const float4x4& matrix, uint32_t lightIdx) const
{
    const auto& srcLight = mSrcModel.lights[lightIdx];
    const auto& c = srcLight.color;
    float3 intensity = float3(
        static_cast<float>(c[0]),
        static_cast<float>(c[1]),
        static_cast<float>(c[2])) * static_cast<float>(srcLight.intensity);

    float3 position = mul(matrix, float4(0.0f, 0.0f, 0.0f, 1.0f)).xyz();

    return std::make_shared<PointLight>(intensity, position);
}
}

bool
loadGltf(Scene* scene, std::string_view filename)
{
    GltfLoader loader(scene);
    return loader.load(filename);
}
}
