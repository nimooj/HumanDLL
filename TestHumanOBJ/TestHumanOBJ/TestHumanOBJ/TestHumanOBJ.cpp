// TestHumanOBJ.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include "pch.h"
#include <iostream>
#include "HumanOBJ.h"

using namespace std;

int main() {
    cout << "Hello HumanOBJ!\n"; 

	/*
	loadOBJ((char*) "Body/Mick.obj");
	loadJoints((char*)"Body/Mick_joints.Joint");
	loadLandmarks((char*)"Body/Mick_landmarks.Landmark");
	*/

	loadHuman((char*) "Body/avatar_man.PBody");

	cout << "*** Human Sizes 00 ***" << endl;
	cout << "- Height : " << getHeight() << endl;
	cout << "- Bust : " << getBustSize() << endl;
	cout << "- Waist : " << getWaistSize() << endl;
	cout << "- Hip : " << getHipSize() << endl;
	cout << "*******************" << endl << endl;

	setTPose(1.0);
	writeToOBJ((char*) "Body/result00.obj");
	writeToHuman((char*) "Body/result00.PBody");

	float *sizes = new float[4];
	sizes[0] = 100.0f; // Height
	sizes[1] = 100.0f; // Bust
	sizes[2] = 50.0f; // Waist
	sizes[3] = 100.0f; // Hip

	resize(sizes); 

	cout << "*** Human Sizes 01 ***" << endl;
	cout << "- Height : " << getHeight() << endl;
	cout << "- Bust : " << getBustSize() << endl;
	cout << "- Waist : " << getWaistSize() << endl;
	cout << "- Hip : " << getHipSize() << endl;
	cout << "*******************" << endl << endl;

	//setTPose(1.0);
	writeToOBJ((char*) "Body/result01.obj");
	writeToHuman((char*) "Body/result01.PBody");

	sizes[0] = 164.0f; // Height
	sizes[1] = 90.0f; // Bust
	sizes[2] = 70.0f; // Waist
	sizes[3] = 93.0f; // Hip
	
	resize(sizes); 

	cout << "*** Human Sizes 02 ***" << endl;
	cout << "- Height : " << getHeight() << endl;
	cout << "- Bust : " << getBustSize() << endl;
	cout << "- Waist : " << getWaistSize() << endl;
	cout << "- Hip : " << getHipSize() << endl;
	cout << "*******************" << endl << endl;

	writeToOBJ((char*) "Body/result02.obj");
	writeToHuman((char*) "Body/result02.PBody");

	sizes[0] = 140.0f; // Height
	sizes[1] = 90.0f; // Bust
	sizes[2] = 70.0f; // Waist
	sizes[3] = 110.0f; // Hip
	
	resize(sizes); 

	cout << "*** Human Sizes 03 ***" << endl;
	cout << "- Height : " << getHeight() << endl;
	cout << "- Bust : " << getBustSize() << endl;
	cout << "- Waist : " << getWaistSize() << endl;
	cout << "- Hip : " << getHipSize() << endl;
	cout << "*******************" << endl << endl;

	writeToOBJ((char*) "Body/result03.obj");
	writeToHuman((char*) "Body/result03.PBody");
	delete[] sizes;
}
