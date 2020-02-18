#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#define _USE_MATH_DEFINES
#include <math.h>

//#include "Bone.h"

#define Axis_X 0
#define Axis_Y 1
#define Axis_Z 2

using namespace std;

class Vertex {
public:
	Vertex();
	Vertex(float, float, float);
	Vertex(int, float, float, float);
	Vertex(int, Vertex);
	Vertex(Vertex*);
	~Vertex();
	
	int idx;
	float x, y, z;
	float nx, ny, nz;
	vector<int> meshIds;
	bool updated = false;

	vector<int> refBone;
	vector<float> refWeight;

	vector<int> jointsRelated;
	vector<float> jointWeights;

	bool isNull();
	void set();
	void set(float, float, float);
	void del();
	void print();

	bool operator==(const Vertex& v);
	bool operator!=(const Vertex& v);

	Vertex operator+(const Vertex&);
	Vertex operator-(const Vertex&);
	Vertex operator*(const Vertex&);
	Vertex operator+(const float&);
	Vertex operator/(const float&);
	Vertex operator-(const int&);
	Vertex operator*(const float&);

	float distance(Vertex);
	float distanceToLine(Vertex, Vertex);
	Vertex closest(vector<Vertex>);
	Vertex closest(float, float, float, float);

	Vertex subtract(Vertex);
	Vertex add(Vertex);
	Vertex add(float);
	Vertex multiply(float);
	Vertex divide(float);
	Vertex cross(Vertex);
	Vertex dot(Vertex);
	Vertex normalize();

	float length();
	float sum();

	void rotate(int, float);

private:
	bool nullFlag;
};