#include "MeshManager.h"

#include "Main/Application.h"
#include "Module/ModuleFileSystem.h"
#include "ResourceManagement/Metafile/Metafile.h"
#include "ResourceManagement/Resources/Mesh.h"

#include <Brofiler/Brofiler.h>
#include <vector>

std::shared_ptr<Mesh> MeshManager::Load(uint32_t uuid, const FileData& resource_data)
{
	char * data = (char*)resource_data.buffer;
	char* cursor = data;

	uint32_t ranges[3];
	//Get ranges
	size_t bytes = sizeof(ranges); // First store ranges
	memcpy(ranges, cursor, bytes);

	std::vector<uint32_t> indices;
	std::vector<Mesh::Vertex> vertices;
	std::vector<Mesh::MorphTarget> morph_targets;

	indices.resize(ranges[0]);

	cursor += bytes; // Get indices
	bytes = sizeof(uint32_t) * ranges[0];
	memcpy(&indices.front(), cursor, bytes);

	vertices.resize(ranges[1]);

	cursor += bytes; // Get vertices
	bytes = sizeof(Mesh::Vertex) * ranges[1];
	memcpy(&vertices.front(), cursor, bytes);

	morph_targets.resize(ranges[2]);

	if (ranges[2] > 0)
	{
		cursor += bytes; // Get morph targets
		bytes = sizeof(Mesh::MorphTarget) * ranges[2];
		memcpy(&morph_targets.front(), cursor, bytes);
	}

	std::shared_ptr<Mesh> new_mesh = std::make_shared<Mesh>(uuid, std::move(vertices), std::move(indices), std::move(morph_targets));

	return new_mesh;
}