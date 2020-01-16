#include "stdafx.h"
#include "Bone.h"

Bone::Bone() {

}

Bone::Bone(Joint& first, Joint& second) {
	j1 = &first;
	j2 = &second;
}

Bone::~Bone() {

}

/*
vector<Joint*> Bone::getJoints() {
	vector<Joint*> j;
	j.push_back(j1);
	j.push_back(j2);

	return j;
}
*/