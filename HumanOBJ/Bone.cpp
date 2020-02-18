#include "stdafx.h"
#include "Bone.h"

Bone::Bone() {

}

Bone::Bone(int first, int second) {
	upperJoint = first;
	lowerJoint = second;
}

Bone::Bone(int p, int first, int second) {
	parentBone = p;
	upperJoint = first;
	lowerJoint = second;
}

Bone::~Bone() {
}