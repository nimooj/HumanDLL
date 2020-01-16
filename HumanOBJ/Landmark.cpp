#include "stdafx.h"
#include "Landmark.h"

Landmark::Landmark() {
	name = (char*)"";
}

Landmark::Landmark(char* ctr, float t) {
	name = ctr;
	type = t;
}

Landmark::Landmark(char* ctr, float t, float val) {
	name = ctr;
	type = t;
	value = val;
}

Landmark::Landmark(char* ctr, float t, float val, vector<int> inds) {
	name = ctr;
	type = t;
	value = val;
	vertIdx.insert(vertIdx.end(), inds.begin(), inds.end());
}

Landmark::Landmark(char* ctr, float t, float val, float h, vector<int> inds) {
	name = ctr;
	type = t;
	value = val;
	level = h;
	vertIdx.insert(vertIdx.end(), inds.begin(), inds.end());
}

Landmark::Landmark(char* ctr, vector<int> s, float t, float val, float h, vector<int> inds) {
	name = ctr;
	type = t;
	value = val;
	level = h;
	region = s;
	vertIdx.insert(vertIdx.end(), inds.begin(), inds.end());
}

Landmark::~Landmark() {

}

bool Landmark::SetGirthFeature(float scale, vector<int>& secs, vector<int>& inds, vector<Vertex>& verts, float h) {
	region.clear();
	vertIdx.clear();
	
	float ys = 0;
	float dist = 0;
	vector<Vertex> girth;
	GrahamScan g;

	region = secs;
	
	/*** Limit relevants ***/
	int trial = 0;
	float range = 0.5 * 1/scale;
	while (trial <= 10 && girth.size() < 12) {
		girth.clear();
		ys = 0;

		for (int i = 0; i < inds.size(); i++) {
			if (abs(verts[inds[i] - 1].y - h) <= range) {
				Vertex v = verts[inds[i] - 1];
				girth.push_back(Vertex(v.idx, v.x, h, v.z));
				ys += v.y;
			}
		}

		trial++;
		range += range/8;
	}

	if (girth.empty()) {
		return false;
	}

	/*** Unify offset ***/
	for (int i = 0; i < girth.size(); i++) {
		girth[i].y = h;
	}

	level = h;

	/*** Get Convex Hull  ***/
	g = GrahamScan(girth);
	vector<Vertex> convex = g.GenerateConvexHull();

	for (int i = 1; i < convex.size(); i++) {
		dist += convex[i].distance(convex[i - 1]);
		vertIdx.push_back(convex[i].idx);
	}
	dist += convex[convex.size() - 1].distance(convex[0]);
	value = dist;

	return true;
}

bool Landmark::SetLengthFeature(vector<int>& secs, vector<Joint>& joints) {
	region.clear();
	vertIdx.clear();

	region = secs;

	level = -1;
	if (joints.size() > 2) {
		return false;
	}
	/*** Assuming joints.size() == 2 ***/
	/*** Size has to be 2 for now ***/
	vertIdx.push_back(joints[0].id);
	vertIdx.push_back(joints[1].id);

	for (int i = 1; i < joints.size(); i++) {
		Vertex joint_1 = joints[i - 1].getCoord();
		Vertex joint_2 = joints[i].getCoord();

		value += joint_1.distance(joint_2);
	}

	/***********************************/
	return true;
}