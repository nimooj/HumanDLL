#pragma once

#include <vector>

#include "BodySegments.h"
#include "Vertex.h"
#include "Joint.h"
#include "GrahamScan.h"

#define Girth 0
#define Length 1
#define Mark 2

class Landmark {
public:
	Landmark();
	Landmark(char*, float);
	Landmark(char*, float, float);
	Landmark(char*, float, float, vector<int>);
	Landmark(char*, float, float, float, vector<int>);
	Landmark(char*, vector<int>, float, float, float, vector<int>);
	~Landmark();

	bool SetGirthFeature(float, vector<int>&, vector<int>&, vector<Vertex>&, float);
	bool SetLengthFeature(vector<int>&, vector<Joint>&);

	char* name;
	int type;
	float value;

	float level; // for Girth feature

	vector<int> region;
	vector<int> vertIdx;
};
