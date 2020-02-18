#pragma once

#include "Joint.h"

class Bone {
public:
	Bone();
	Bone(int, int);
	Bone(int, int, int);
	~Bone();

	int parentBone;
	int upperJoint, lowerJoint;
};