#include "stdafx.h"
#include "Mesh.h"

Mesh::Mesh() {

}

Mesh::Mesh(int i1, int i2, int i3) {
	index1 = i1;
	index2 = i2;
	index3 = i3;

	v1 = Vertex();
	v2 = Vertex();
	v3 = Vertex();

	center = v1.add(v2).add(v3).divide(4);
}

Mesh::Mesh(int i1, int i2, int i3, int i4) {
	index1 = i1;
	index2 = i2;
	index3 = i3;
	index4 = i4;

	v1 = Vertex();
	v2 = Vertex();
	v3 = Vertex();
	v4 = Vertex();

	center = v1.add(v2).add(v3).add(v4).divide(4);
}

Mesh::Mesh(Vertex _v1, Vertex _v2, Vertex _v3) {
	v1 = _v1;
	v2 = _v2;
	v3 = _v3;

	index1 = v1.idx;
	index2 = v2.idx;
	index3 = v3.idx;

	center = v1.add(v2).add(v3).divide(3);

	Vertex v = (v3 - v1);
	Vertex vv = (v2 - v1);
	normal = vv.cross(v).normalize();
	d = -(normal.x*v1.x + normal.y*v1.y + normal.z*v1.z);
}

Mesh::Mesh(Vertex _v1, Vertex _v2, Vertex _v3, Vertex _v4) {
	v1 = _v1;
	v2 = _v2;
	v3 = _v3;
	v4 = _v4;

	index1 = v1.idx;
	index2 = v2.idx;
	index3 = v3.idx;
	index4 = v4.idx;

	center = v1.add(v2).add(v3).add(v4).divide(4);

	Vertex v = (v3 - v1);
	Vertex vv = (v2 - v1);
	normal = vv.cross(v).normalize();
	d = -(normal.x*v1.x + normal.y*v1.y + normal.z*v1.z);
}

Mesh::Mesh(int _i1, int _i2, int _i3, Vertex _v1, Vertex _v2, Vertex _v3) {
	v1 = _v1;
	v2 = _v2;
	v3 = _v3;

	index1 = _i1;
	index2 = _i2;
	index3 = _i3;

	center = v1.add(v2).add(v3).divide(3);
	Vertex v = (v3 - v1);
	Vertex vv = (v2 - v1);
	normal = vv.cross(v).normalize();
	d = -(normal.x*v1.x + normal.y*v1.y + normal.z*v1.z);
}

Mesh::~Mesh() {

}

bool Mesh::has(Vertex v) {
	if (v1 == v || v2 == v || v3 == v || v4 == v)
		return true; 

	return false;
}

bool Mesh::has(int i) {
	if (index1 == i || index2 == i || index3 == i || index4 == i)
		return true;

	return false;
}

void Mesh::updateMesh() {

}
