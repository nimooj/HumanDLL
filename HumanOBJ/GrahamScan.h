#pragma once


#include "Vertex.h"
#include <vector>
using namespace std;

class GrahamScan {
public:
	GrahamScan();
	GrahamScan(vector<Vertex>);
	~GrahamScan();

	vector<Vertex> GenerateConvexHull();


private:
	Vertex P;
	vector<Vertex> verts;
	vector<Vertex> newVerts;

	vector<Vertex> mergeSort(vector<Vertex>, Vertex);
	vector<Vertex> mSort(vector<Vertex>, Vertex);
	vector<Vertex> merge(vector<Vertex>, vector<Vertex>, Vertex);

	float slope(Vertex, Vertex);
	bool isLeftTurn(Vertex, Vertex);
};
