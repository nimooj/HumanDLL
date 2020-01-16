#pragma once

#include "Joint.h"

class Bone {
public :
	Bone();
	Bone(Joint&, Joint&);
	~Bone();

	//vector<Joint*> getJoints();

private :
	Joint* j1, *j2;
};
