#pragma once

#include <vector>
#include "Vertex.h"

class Layer {
public:
	Layer();
	Layer(vector<Vertex>&);
	~Layer();

	bool operator==(Layer&);
	
	void setLayer(vector<Vertex>&);
	Vertex getCenter();
	vector<Vertex>& getVerts();
	int getSize();

	void moveCenter(float, float, float);

private:
	Vertex maxX, maxY, maxZ;
	Vertex minX, minY, minZ;

	Vertex center;
	vector<Vertex> vertices;
};
