#pragma once

#include "Vertex.h"

class Mesh {
public:
	Mesh();
	Mesh(int, int, int);
	Mesh(int, int, int, int);
	Mesh(Vertex, Vertex, Vertex);
	Mesh(Vertex, Vertex, Vertex, Vertex);

	Mesh(int, int, int, Vertex, Vertex, Vertex);
	~Mesh();

	int index1, index2, index3, index4;
	Vertex v1, v2, v3, v4;
	Vertex center;
	Vertex normal;
	float d;

	bool has(Vertex);
	bool has(int);
	void updateMesh();
};
