#pragma once

#include <vector>
#include "Joint.h"

// Joints
#define Neck 0
#define ClavicleMid 1
#define ShoulderR 2
#define ShoulderL 3
#define Waist 4
#define PelvisMid 5
#define PelvisR 6
#define PelvisL 7
#define KneeR 8
#define KneeL 9
#define AnkleR 10
#define AnkleL 11
#define ElbowR 12
#define WristR 13
#define ElbowL 14
#define WristL 15

class JointTree {
public:
	JointTree() {};
	~JointTree() {};

	int getSize() {
		return jointTree.size();
	}

	vector<Joint> getJointTree() {
		return jointTree;
	}

	Joint& at(int i) {
		return jointTree[i];
	}

	void append(Joint j) {
		jointTree.push_back(j);
	}

private:
	vector<Joint> jointTree;
};
