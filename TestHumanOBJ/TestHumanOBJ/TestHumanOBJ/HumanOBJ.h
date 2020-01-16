#pragma once

#ifdef HUMANOBJ_EXPORTS
#define HUMANOBJ_API __declspec(dllexport)
#else
#define HUMANOBJ_API __declspec(dllimport)
#endif

#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <algorithm>
using namespace std;

#include "Vertex.h"
#include "Mesh.h"
#include "JointTree.h"
#include "Skinning.h"
#include "GrahamScan.h"
#include "Landmark.h"

#define TIGHTS 0
#define DRESS 1
#define TOP 2
#define PANTS 3
#define SKIRT 4

void updateRigs();
void centering(float, float, float);

void setRigs();
void setFeatures();

extern "C" HUMANOBJ_API void initOBJ();
extern "C" HUMANOBJ_API void loadHuman(char*);
extern "C" HUMANOBJ_API void loadOBJ(char*);
extern "C" HUMANOBJ_API void loadJoints(char*);
extern "C" HUMANOBJ_API void loadLandmarks(char*);
extern "C" HUMANOBJ_API void getVert(int, float*);
extern "C" HUMANOBJ_API void getVerts(float*);
extern "C" HUMANOBJ_API void getMesh(int, int*);
extern "C" HUMANOBJ_API void getIndices(int*);
extern "C" HUMANOBJ_API float getSize(int);
extern "C" HUMANOBJ_API int getVertSize();
extern "C" HUMANOBJ_API int getMeshSize();
extern "C" HUMANOBJ_API int getLandmarkNum();

extern "C" HUMANOBJ_API void loadPose(char*);
extern "C" HUMANOBJ_API void initHuman();
extern "C" HUMANOBJ_API void setPose(float);
extern "C" HUMANOBJ_API void setTPose(float);

extern "C" HUMANOBJ_API void setHeight(float);
extern "C" HUMANOBJ_API void setBustSize(float, float);
extern "C" HUMANOBJ_API void setWaistSize(float, float);
extern "C" HUMANOBJ_API void setHipSize(float, float);
extern "C" HUMANOBJ_API bool setThreeSize(float, float*);
extern "C" HUMANOBJ_API void setSize(float, int, int, float, float);
extern "C" HUMANOBJ_API bool resize(float*);

extern "C" HUMANOBJ_API float getHeight();
extern "C" HUMANOBJ_API float getBustSize();
extern "C" HUMANOBJ_API float getWaistSize();
extern "C" HUMANOBJ_API float getHipSize();


extern "C" HUMANOBJ_API void getSizeName(int, char*);
extern "C" HUMANOBJ_API int getSizePathLength(int);
extern "C" HUMANOBJ_API void getSizePath(int, float*);
extern "C" HUMANOBJ_API void getSizePathPos(int, int, float*);
extern "C" HUMANOBJ_API int getBodyPartNum(char*);
extern "C" HUMANOBJ_API void getBodyPartOrigin(int, float*);
extern "C" HUMANOBJ_API void getBodyPartDirection(int, float*);
extern "C" HUMANOBJ_API int getBodyPartPointNum(int);
extern "C" HUMANOBJ_API void getBodyPartPointIndex(int, int*);
extern "C" HUMANOBJ_API void getBodyPartPointPos(int, float*);

extern "C" HUMANOBJ_API void setHullPoint(int, int, float*);
extern "C" HUMANOBJ_API void setHullElem(int, int, int*);
extern "C" HUMANOBJ_API void getHullPoint(int, float*);
extern "C" HUMANOBJ_API int getHullPointNum(int);
extern "C" HUMANOBJ_API void getHullElem(int, int*);
extern "C" HUMANOBJ_API int getHullElemNum(int);

extern "C" HUMANOBJ_API void writeToOBJ(char*);
extern "C" HUMANOBJ_API void writeToHuman(char*);
