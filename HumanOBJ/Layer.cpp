#include "stdafx.h"
#include "Layer.h"

Layer::Layer() {
	maxX = Vertex();
	maxY = Vertex();
	maxZ = Vertex();
	minX = Vertex();
	minY = Vertex();
	minZ = Vertex();

	center = Vertex();
}

Layer::~Layer() {

}

Layer::Layer(vector<Vertex>& v) {
	maxX = v[0]; 
	maxY = v[0]; 
	maxZ = v[0]; 
	minX = v[0]; 
	minY = v[0]; 
	minZ = v[0];
	for (int i = 0; i < v.size(); i++) {
		float x = v[i].x;
		float y = v[i].y;
		float z = v[i].z;

		if (x > maxX.x) maxX = v[i];
		if (x < minX.x) minX = v[i];
		if (y > maxY.y) maxY = v[i];
		if (y < minY.y) minY = v[i];
		if (z > maxZ.z) maxZ = v[i];
		if (z < minZ.z) minZ = v[i];
	}

	vertices = v;
	center = Vertex(((maxX + minX)/2).x, ((maxY + minY)/2).y, ((maxZ + minZ)/2).z);

}

bool Layer::operator==(Layer& l) {
	if (vertices.size() != l.getSize())
		return false;

	vector<Vertex> tmpV = l.getVerts();

	for (int i = 0; i < l.getSize(); i++) {
		if (vertices[i] != tmpV[i])
			return false;
	}
	return true;
}

void Layer::setLayer(vector<Vertex>& v) {
	vertices = v;
}

Vertex Layer::getCenter() {
	return center;
}

vector<Vertex>& Layer::getVerts() {
	return vertices;
}

int Layer::getSize() {
	return vertices.size();
}

void Layer::moveCenter(float mx, float my, float mz) {
	center.x -= mx;
	center.y -= my;
	center.z -= mz;
}
