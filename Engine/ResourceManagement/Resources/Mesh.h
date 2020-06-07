#ifndef _MESH_H_
#define _MESH_H_

#include "Resource.h"
#include "ResourceManagement/Manager/MeshManager.h"

#include <GL/glew.h>
#include <MathGeoLib.h>
#include <vector>

class Metafile;

static const size_t MAX_JOINTS = 4;
static const size_t MAX_MORPH_TARGETS = 20;
class Mesh : public Resource
{
public:
	struct Vertex 
	{
		float3 position;
		float3 normals;
		float3 tangent;
		float3 bitangent;
		float2 tex_coords;
		uint32_t joints[MAX_JOINTS] = {0,0,0,0};
		float weights[MAX_JOINTS] = {0,0,0,0};
		uint32_t index = 0;
	};
	struct MorphVertex
	{
		float3 position;
		float3 normals;
		float3 tangent;
	};
	struct MorphTarget
	{
		float3 morph_targets[MAX_MORPH_TARGETS];
		//float3 morph_targets[MAX_MORPH_TARGETS];
		//std::vector<MorphVertex> morph_target;
	};

	Mesh(uint32_t uuid, std::vector<Vertex> && vertices, std::vector<uint32_t> && indices, std::vector<MorphTarget> && morph_targets_vector);
	~Mesh();

	GLuint GetVAO() const;
	void ChangeTiling();
	int GetNumTriangles() const;
	int GetNumVerts() const;
	std::vector<Triangle> GetTriangles() const;

private:
	void LoadInMemory();



public:
	std::vector<Vertex> vertices;
	std::vector<uint32_t> indices;
	std::vector<MorphTarget> morph_targets_vector;

private:
	GLuint vao = 0;
	GLuint vbo = 0;
	GLuint ssbo = 0;
	GLuint ebo = 0;
};

namespace ResourceManagement
{
	template<>
	static std::shared_ptr<Mesh> Load(uint32_t uuid, const FileData& resource_data)
	{
		return MeshManager::Load(uuid, resource_data);
	}
}

#endif // !_MESH_H_

