#include "Mesh.h"

#include "ResourceManagement/Metafile/Metafile.h"

Mesh::Mesh(uint32_t uuid, std::vector<Vertex> && vertices, std::vector<uint32_t> && indices, std::vector<MorphTarget> && morph_targets_vector)
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
	glGenBuffers(1, &vmo);
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

	glEnableVertexAttribArray(7);
	glVertexAttribPointer(7, MAX_MORPH_TARGETS, GL_FLOAT, GL_FALSE, sizeof(Mesh::Vertex), (void*)offsetof(Mesh::Vertex, morph_targets));

	glBindVertexArray(0);
}

