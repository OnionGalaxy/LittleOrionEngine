#include "Mesh.h"

#include "ResourceManagement/Metafile/Metafile.h"

Mesh::Mesh(uint32_t uuid, std::vector<Vertex> && vertices, std::vector<uint32_t> && indices, std::vector<MorphVertex> && morph_targets_vector)
	: vertices(vertices)
	, indices(indices)
	, morph_targets_vector(morph_targets_vector)
	, Resource(uuid)
{
	LoadInMemory();
}

Mesh::~Mesh() 
{
	if (vbo != 0)
	{
		glDeleteBuffers(1, &vbo);
		glDeleteBuffers(1, &ebo);
		glDeleteBuffers(1, &ssbo);
		glDeleteVertexArrays(1, &vao);
	}
}

GLuint Mesh::GetVAO() const
{
	return vao;
}

int Mesh::GetNumTriangles() const
{
	return indices.size() / 3;
}

int Mesh::GetNumVerts() const
{
	return vertices.size();
}

void Mesh::LoadInMemory()
{

	glGenVertexArrays(1, &vao);
	glGenBuffers(1, &vbo);
	glGenBuffers(1, &ssbo);
	glGenBuffers(1, &ebo);

	glBindVertexArray(vao);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Mesh::Vertex), vertices.data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(uint32_t), indices.data(), GL_STATIC_DRAW);

	// VERTEX POSITION
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Mesh::Vertex), (void*)0);

	// VERTEX TEXTURE COORDS
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Mesh::Vertex), (void*)offsetof(Mesh::Vertex, tex_coords));

	// VERTEX NORMALS
	glEnableVertexAttribArray(2);
	glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(Mesh::Vertex), (void*)offsetof(Mesh::Vertex, normals));

	// VERTEX TANGENT
	glEnableVertexAttribArray(3);
	glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Mesh::Vertex), (void*)offsetof(Mesh::Vertex, tangent));

	// VERTEX BITANGENT
	glEnableVertexAttribArray(4);
	glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Mesh::Vertex), (void*)offsetof(Mesh::Vertex, bitangent));
	
	// VERTEX JOINTS
	glEnableVertexAttribArray(5);
	glVertexAttribIPointer(4, 4, GL_UNSIGNED_INT, sizeof(Mesh::Vertex), (void*)offsetof(Mesh::Vertex, joints));

	// VERTEX WEIGHTS
	glEnableVertexAttribArray(6);
	glVertexAttribPointer(5, 4, GL_FLOAT, GL_FALSE, sizeof(Mesh::Vertex), (void*)offsetof(Mesh::Vertex, weights));

	glEnableVertexAttribArray(9);
	glVertexAttribIPointer(9, 1, GL_UNSIGNED_INT, sizeof(Mesh::Vertex), (void*)offsetof(Mesh::Vertex, index));

	if (morph_targets_vector.size() > 0)
	{
		glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo);
		glBufferData(GL_SHADER_STORAGE_BUFFER, sizeof(MorphVertex) * morph_targets_vector.size(), morph_targets_vector.data(), GL_STATIC_DRAW);
		glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 10, ssbo);
	}
	glBindVertexArray(0);
}

