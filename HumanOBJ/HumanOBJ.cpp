// HumanOBJ.cpp : Defines the exported functions for the DLL application.
//

#include "stdafx.h"
#include "HumanOBJ.h"

float scaleFactor;
Skinning skinning;

vector<Vertex> origVertices;
vector<Vertex> vertices;
vector<Joint> joints;
vector<Joint> origJoints;
vector<Vertex> textures;
vector<Vertex> normals;
vector<Mesh> meshes; // Always Tri-mesh
vector<int> indices; 
vector<Landmark> landmarks;

vector<int> segmentGroup;
vector<int> weightGroup;
vector<float> weightValues;

vector<int> segmentHash[SegmentNum]; // vert indices
vector<int> weightHash[WeightNum]; // vert indices
vector<float> weightValueHash[WeightNum]; // vert indices

vector<float> segment3DConvex[SegmentNum];
vector<int> segment3DIndices[SegmentNum];

int maxTrials = 10;
float thresholdScale = 0.01;
float thresholdSize = 0.1;

float origTopMostLevel = -1, origBottomMostLevel = -1;
float topMostLevel, bottomMostLevel, leftMostLevel, rightMostLevel;
float leftMostOffset, rightMostOffset;
int topMostIndex, bottomMostIndex, leftMostIndex, rightMostIndex;

float height = -1;
float origHeight = -1;
float currHeight = -1;
float origBustSize = -1, origWaistSize = -1, origHipSize = -1;
float currBustSize = -1, currWaistSize = -1, currHipSize = -1;
float bustSize = -1, waistSize = -1, hipSize = -1;

float origBustLevel = -1, origWaistLevel = -1, origHipLevel = -1;
float currBustLevel = -1, currWaistLevel = -1, currHipLevel = -1;
float bustLevel = -1, waistLevel = -1, hipLevel = -1;

int shoulderRIndex, shoulderLIndex;
vector<int> bustConvexIndices, waistConvexIndices, hipConvexIndices;

void setRigs() {
	if (segmentHash->empty()) {
		skinning.setSegments(vertices, joints, segmentGroup, weightHash, weightValueHash);
	}
	else {
		skinning.setImportedSegments(segmentHash, weightHash);
	}
	setFeatures();  
}

void setFeatures() {
	float s = scaleFactor;
	float minZ = 1000 / s, maxZ = -1000 / s;
	float minX = 1000 / s, maxX = -1000 / s;
	float minY = 1000 / s, maxY = -1000 / s;
	vector<int> inds;
	vector<int> secs;

	if (landmarks.size() == 0) {
		for (int i = 0; i < segmentGroup.size(); i++) {
			segmentHash[segmentGroup[i]].push_back(i); // push vertices ARRAY index
		}

		/*** HEIGHT ***/
		int topLevelIdx = 0;
		for (int i = 0; i < segmentHash[Segment_Head].size(); i++) {
			if (vertices[segmentHash[Segment_Head][i]].y > maxY) {
				topMostLevel = vertices[segmentHash[Segment_Head][i]].y;
				topLevelIdx = segmentHash[Segment_Head][i];
				maxY = topMostLevel;
			}
		}

		int bottomLevelIdx = 0;
		vector<int> segment_lowLegs;
		segment_lowLegs.insert(segment_lowLegs.end(), segmentHash[Segment_FootR].begin(), segmentHash[Segment_FootR].end());
		segment_lowLegs.insert(segment_lowLegs.end(), segmentHash[Segment_FootL].begin(), segmentHash[Segment_FootL].end());
		for (int i = 0; i < segment_lowLegs.size(); i++) {
			if (vertices[segment_lowLegs[i]].y < minY) {
				bottomMostLevel = vertices[segment_lowLegs[i]].y;
				bottomLevelIdx = segment_lowLegs[i];
				minY = bottomMostLevel;
			}
		}

		height = (topMostLevel - bottomMostLevel);

		inds.push_back(topLevelIdx + 1);
		inds.push_back(bottomLevelIdx + 1);
		secs.push_back(Segment_Head);
		secs.push_back(Segment_FootR);
		secs.push_back(Segment_FootL);
		landmarks.push_back(Landmark((char*)"Height", secs, Length, height, 0, inds));
		/**************/

		/********************** BUST **********************/
		maxY = -1000 / s;
		minY = 1000 / s;
		// Bust : max z val in segment 8
		for (int i = 0; i < segmentHash[Segment_UpperTorso].size(); i++) {
			if (vertices[segmentHash[Segment_UpperTorso][i]].z > maxZ && vertices[segmentHash[Segment_UpperTorso][i]].y > joints[Joint_waist].getCoord().y) {
				maxZ = vertices[segmentHash[Segment_UpperTorso][i]].z;
				bustLevel = vertices[segmentHash[Segment_UpperTorso][i]].y;
			}

			if (vertices[segmentHash[Segment_UpperTorso][i]].x < minX) {
				minX = vertices[segmentHash[Segment_UpperTorso][i]].x;
				shoulderRIndex = segmentHash[Segment_UpperTorso][i];
			}

			if (vertices[segmentHash[Segment_UpperTorso][i]].x > maxX) {
				maxX = vertices[segmentHash[Segment_UpperTorso][i]].x;
				shoulderLIndex = segmentHash[Segment_UpperTorso][i];
			}
		}
		/*** Bust level calibration ***/
		Vertex neck = joints[Joint_neck].getCoord();
		Vertex waist = joints[Joint_waist].getCoord();
		if (bustLevel < (neck.y + waist.y) / 2) {
			bustLevel = (neck.y + waist.y) / 2;
		}
		/******************************/
		/*** Align verts near bust level ***/
		vector<Vertex> nearBust;
		int trial = 0;
		float range = 0.5 * 1 / s;
		while (nearBust.size() < 12 && trial < 20) {
			nearBust.clear();

			for (int i = 0; i < segmentHash[Segment_UpperTorso].size(); i++) {
				if (abs(vertices[segmentHash[Segment_UpperTorso][i]].y - bustLevel) <= range) {
					Vertex v = vertices[segmentHash[Segment_UpperTorso][i]];
					nearBust.push_back(Vertex(v.idx, v.x, bustLevel, v.z));
				}
			}
			range += range / 8;
			trial++;
		}
		/*** Get convex hull***/
		GrahamScan g = GrahamScan(nearBust);
		vector<Vertex> bustConvex = g.GenerateConvexHull();
		float dist = 0;
		for (int i = 1; i < bustConvex.size(); i++) {
			dist += bustConvex[i].distance(bustConvex[i - 1]);
			bustConvexIndices.push_back(bustConvex[i].idx);
		}
		dist += bustConvex[bustConvex.size() - 1].distance(bustConvex[0]);
		bustSize = dist;

		inds.clear();
		secs.clear();
		inds.insert(inds.end(), bustConvexIndices.begin(), bustConvexIndices.end());
		secs.push_back(Segment_UpperTorso);

		landmarks.push_back(Landmark((char*)"Bust", secs, Girth, bustSize, bustLevel, inds));
		/**************************************************/

		/********************** WAIST **********************/
		// Waist : min X val around waist level in segment 8 and 9
		minX = 1000 / s;
		minY = 1000 / s;
		Vertex waistJoint = joints[Joint_waist].getCoord();
		vector<int> segment_torso;
		//segment_torso.insert(segment_torso.end(), segmentHash[Segment_UpperTorso].begin(), segmentHash[Segment_UpperTorso].end());
		segment_torso.insert(segment_torso.end(), segmentHash[Segment_LowerTorso].begin(), segmentHash[Segment_LowerTorso].end());
		for (int i = 0; i < segment_torso.size(); i++) {
			if (vertices[segment_torso[i]].y < bustLevel) {
				if (abs(vertices[segment_torso[i]].x) < minX && abs(vertices[segment_torso[i]].y - joints[Joint_waist].getCoord().y) < range) {
					minX = abs(vertices[segment_torso[i]].x);
					waistLevel = vertices[segment_torso[i]].y;
				}
			}
		}
		/*** Align verts near waist level ***/
		vector<Vertex> nearWaist;
		trial = 0;
		range = 0.5 * 1 / s;
		while (nearWaist.size() < 12 && trial < 20) {
			nearWaist.clear();
			for (int i = 0; i < segment_torso.size(); i++) {
				if (abs(vertices[segment_torso[i]].y - waistLevel) <= range) {
					Vertex v = vertices[segment_torso[i]];
					nearWaist.push_back(Vertex(v.idx, v.x, waistLevel, v.z));
				}
			}
			range += range / 8;
			trial++;
		}
		/*** Get convex hull***/
		g = GrahamScan(nearWaist);
		vector<Vertex> waistConvex = g.GenerateConvexHull();
		dist = 0;
		for (int i = 1; i < waistConvex.size(); i++) {
			dist += waistConvex[i].distance(waistConvex[i - 1]);
			waistConvexIndices.push_back(waistConvex[i].idx);
		}
		dist += waistConvex[waistConvex.size() - 1].distance(waistConvex[0]);
		waistSize = dist;

		inds.clear();
		secs.clear();
		inds.insert(inds.end(), waistConvexIndices.begin(), waistConvexIndices.end());
		secs.push_back(Segment_UpperTorso);
		secs.push_back(Segment_LowerTorso);
		landmarks.push_back(Landmark((char*)"Waist", secs, Girth, waistSize, waistLevel, inds));
		/***************************************************/

		/********************** Hip **********************/
		// Hip : min Z val around pelvisMid level in segment 9, 10 and 13
		minZ = 1000 / s;
		Vertex pelvisJoint = joints[Joint_pelvisMid].getCoord();
		vector<int> segment_legs;
		segment_legs.insert(segment_legs.end(), segmentHash[Segment_LowerTorso].begin(), segmentHash[Segment_LowerTorso].end());
		segment_legs.insert(segment_legs.end(), segmentHash[Segment_UpperLegR].begin(), segmentHash[Segment_UpperLegR].end());
		segment_legs.insert(segment_legs.end(), segmentHash[Segment_UpperLegL].begin(), segmentHash[Segment_UpperLegL].end());
		for (int i = 0; i < segment_legs.size(); i++) {
			if (abs(vertices[segment_legs[i]].y - pelvisJoint.y) <= 1) {
				if (vertices[segment_legs[i]].z < minZ) {
					minZ = vertices[segment_legs[i]].z;
					hipLevel = vertices[segment_legs[i]].y;
				}
			}
		}
		/*** Align verts near hip level ***/
		vector<Vertex> nearHip;
		trial = 0;
		range = 0.5 * 1 / s;
		while (nearHip.size() < 12 && trial < 10) {
			nearHip.clear();

			for (int i = 0; i < segment_legs.size(); i++) {
				if (abs(vertices[segment_legs[i]].y - hipLevel) <= range) {
					Vertex v = vertices[segment_legs[i]];
					nearHip.push_back(Vertex(v.idx, v.x, hipLevel, v.z));
				}
			}
			range += range / 8;
			trial++;
		}
		/*** Get convex hull***/
		g = GrahamScan(nearHip);
		vector<Vertex> hipConvex = g.GenerateConvexHull();
		dist = 0;
		for (int i = 1; i < hipConvex.size(); i++) {
			dist += hipConvex[i].distance(hipConvex[i - 1]);
			hipConvexIndices.push_back(hipConvex[i].idx);
		}
		dist += hipConvex[hipConvex.size() - 1].distance(hipConvex[0]);
		hipSize = dist;

		inds.clear();
		secs.clear();
		inds.insert(inds.end(), hipConvexIndices.begin(), hipConvexIndices.end());
		secs.push_back(Segment_LowerTorso);
		secs.push_back(Segment_UpperLegR);
		secs.push_back(Segment_UpperLegL);
		landmarks.push_back(Landmark((char*)"Hip", secs, Girth, hipSize, hipLevel, inds));
		/*************************************************/

		/********************** Neck **********************/
	}
	else {
		if (segmentHash->empty()) {
			for (int i = 0; i < segmentGroup.size(); i++) {
				segmentHash[segmentGroup[i]].push_back(i); // push vertices ARRAY index
			}
		}

		height = landmarks[0].value;

		bustSize = landmarks[1].value;
		bustLevel = landmarks[1].level;
		bustConvexIndices.insert(bustConvexIndices.end(), landmarks[1].vertIdx.begin(), landmarks[1].vertIdx.end());

		waistSize = landmarks[2].value;
		waistLevel = landmarks[2].level;
		waistConvexIndices.insert(waistConvexIndices.end(), landmarks[2].vertIdx.begin(), landmarks[2].vertIdx.end());

		hipSize = landmarks[3].value;
		hipLevel = landmarks[3].level;
		hipConvexIndices.insert(hipConvexIndices.end(), landmarks[3].vertIdx.begin(), landmarks[3].vertIdx.end());
	}

	origHeight = height;
	origBustSize = bustSize;
	origWaistSize = waistSize;
	origHipSize = hipSize;

	origBustLevel = landmarks[1].level;
	origWaistLevel = landmarks[2].level;
	origHipLevel = landmarks[3].level;

}

float getHeight() {
	return topMostLevel - bottomMostLevel;
}

float getBustSize() {
	float dist = 0;
	Vertex prevV, currV;
	for (int i = 1; i < bustConvexIndices.size(); i++) {
		prevV = vertices[bustConvexIndices[i - 1] - 1];
		currV = vertices[bustConvexIndices[i] - 1];

		prevV.y = bustLevel;
		currV.y = bustLevel;

		dist += currV.distance(prevV);
	}

	prevV = vertices[bustConvexIndices[bustConvexIndices.size() - 1] - 1];
	currV = vertices[bustConvexIndices[0] - 1];

	prevV.y = bustLevel;
	currV.y = bustLevel;

	dist += currV.distance(prevV);

	return dist;
}

float getWaistSize() {
	float dist = 0;
	Vertex prevV, currV;
	for (int i = 1; i < waistConvexIndices.size(); i++) {
		prevV = vertices[waistConvexIndices[i - 1] - 1];
		currV = vertices[waistConvexIndices[i] - 1];

		prevV.y = waistLevel;
		currV.y = waistLevel;

		dist += currV.distance(prevV);
	}

	prevV = vertices[waistConvexIndices[waistConvexIndices.size() - 1] - 1];
	currV = vertices[waistConvexIndices[0] - 1];

	prevV.y = waistLevel;
	currV.y = waistLevel;

	dist += currV.distance(prevV);

	return dist;
}

float getHipSize() {
	float dist = 0;
	Vertex prevV, currV;
	for (int i = 1; i < hipConvexIndices.size(); i++) {
		prevV = vertices[hipConvexIndices[i - 1] - 1];
		currV = vertices[hipConvexIndices[i] - 1];

		prevV.y = hipLevel;
		currV.y = hipLevel;

		dist += currV.distance(prevV);
	}

	prevV = vertices[hipConvexIndices[hipConvexIndices.size() - 1] - 1];
	currV = vertices[hipConvexIndices[0] - 1];

	prevV.y = hipLevel;
	currV.y = hipLevel;

	dist += currV.distance(prevV);

	return dist;
}

void updateRigs() {
	//skinning.updateRigs(vertices, joints);
	/*
	for (int i = 0; i < joints.size(); i++) {
		int jointId = joints[i].id;
		Vertex *j = &joints[i].getCoord();
		vector<int> rv = joints[i].getRelatedVerts();

		j->y = (vertices[rv[2]].y + vertices[rv[3]].y) / 2;
		j->z = (vertices[rv[4]].z + vertices[rv[5]].z) / 2;

		if (jointId == Joint_neck || jointId == Joint_shoulderMid || jointId == Joint_waist || jointId == Joint_pelvisMid)
			j->x = 0;
		else
			j->x = (vertices[rv[0]].x + vertices[rv[1]].x) / 2;
	}
	*/
}

void centering(float x, float y, float z) {
	for (int i = 0; i < vertices.size(); i++) {
		vertices[i].x -= x;
		vertices[i].y -= y;
		vertices[i].z -= z;
	}
}

void initOBJ() {
	scaleFactor = 1;

	topMostLevel = 0; bottomMostLevel = 0; leftMostLevel = 0; rightMostLevel = 0;
	leftMostOffset = 0; rightMostOffset = 0;
	topMostIndex = 0; bottomMostIndex = 0; leftMostIndex = 0; rightMostIndex = 0;

	height = 0;
	bustSize = 0; waistSize = 0; hipSize = 0;
	shoulderRIndex = 0; shoulderLIndex = 0;
	bustLevel = 0; waistLevel = 0; hipLevel = 0;

	vertices.clear();
	joints.clear();
	textures.clear();
	normals.clear();
	meshes.clear();
	indices.clear();
	landmarks.clear();
	segmentGroup.clear();
	weightGroup.clear();
	weightValues.clear();

	bustConvexIndices.clear();
	waistConvexIndices.clear();
	hipConvexIndices.clear();

	for (int i = 0; i < SegmentNum; i++) {
		segmentHash[i].clear();
	}
	for (int i = 0; i < WeightNum; i++)
	{
		weightHash[i].clear();
	}
	for (int i = 0; i < WeightNum; i++) {
		weightValueHash[i].clear();
	}
}

void loadHuman(char* dir) {
	vertices.clear();
	origVertices.clear();
	textures.clear();
	normals.clear();
	landmarks.clear();

	joints.clear();
	origJoints.clear();

	int defaultFlag = 0, humanFlag = 0, jointFlag = 0, landmarkFlag = 0, segmentFlag = 0, weightFlag = 0, tposeFlag = 0;
	int orig_idx = 1, idx = 1, joint_idx = 0, segment_idx = 0, weight_idx = 0;
	int landmark_line = 0;
	string line, s;
	Landmark l;
	ifstream basics(dir);

	float maxX = -1000000000, minX = 1000000000;
	float maxY = -1000000000, minY = 1000000000;
	float maxZ = -1000000000, minZ = 1000000000;

	bustLevel = 0;
	waistLevel = 0;
	hipLevel = 0;

	skinning = Skinning();

	vector<Vertex> tmpnormals, tmptextures;
	while (getline(basics, line)) {
		if (joint_idx == 18) {
			jointFlag = 0;
		}
		
		if (line == "### Default") {
			defaultFlag = 1;
			humanFlag = 0;
			jointFlag = 0;
			landmarkFlag = 0;
			segmentFlag = 0;
			weightFlag = 0;
			tposeFlag = 0;
			continue;
		}
		if (line == "### Human") {
			defaultFlag = 0;
			humanFlag = 1;
			jointFlag = 0;
			landmarkFlag = 0;
			segmentFlag = 0;
			weightFlag = 0;
			tposeFlag = 0;
			continue;
		}
		if (line == "### Joint") {
			defaultFlag = 0;
			humanFlag = 0;
			jointFlag = 1;
			landmarkFlag = 0;
			segmentFlag = 0;
			weightFlag = 0;
			tposeFlag = 0;
			joints.clear();
			continue;
		}
		if (line == "### Landmark" ) {
			defaultFlag = 0;
			humanFlag = 0;
			jointFlag = 0;
			landmarkFlag = 1;
			segmentFlag = 0;
			weightFlag = 0;
			tposeFlag = 0;
			landmarks.clear();
			continue;
		}
		if (line == "### Segments") {
			defaultFlag = 0;
			humanFlag = 0;
			jointFlag = 0;
			landmarkFlag = 0;
			segmentFlag = 1;
			weightFlag = 0;
			tposeFlag = 0;
			continue;
		}
		if (line == "### Weights") {
			defaultFlag = 0;
			humanFlag = 0;
			jointFlag = 0;
			landmarkFlag = 0;
			segmentFlag = 0;
			weightFlag = 1;
			tposeFlag = 0;
			continue;
		}
		if (line == "### Tpose") {
			defaultFlag = 0;
			humanFlag = 0;
			jointFlag = 0;
			landmarkFlag = 0;
			segmentFlag = 0;
			weightFlag = 0;
			tposeFlag = 1;
			skinning.history.clear();
			continue;
		}

		if (defaultFlag == 1) {
			istringstream iss(line);
			float x = -1, y = -1, z = -1;

			iss >> x >> y >> z;

			origVertices.push_back(Vertex(orig_idx, x, y, z));
			orig_idx++;
		}

		if (humanFlag == 1) {
			istringstream iss(line);
			string f;
			float x = -1, y = -1, z = -1, w = -1;

			iss >> f >> x >> y >> z >> w;

			if (f == "v") {
				if (minX > x) {
					minX = x;
					leftMostLevel = x;
				}
				if (x > maxX) {
					maxX = x;
					rightMostLevel = y;
				}
				if (minY > y) {
					minY = y;
					bottomMostIndex = idx - 1;
				}
				if (y > maxY) {
					maxY = y;
					topMostIndex = idx - 1;
				}
				if (minZ > z) {
					minZ = z;
				}
				if (z > maxZ) {
					maxZ = z;
				}

				vertices.push_back(Vertex(idx, x, y, z));
				textures.push_back(Vertex());
				normals.push_back(Vertex());
				idx++;
				/* Initialize vertex-joint group */
				segmentGroup.push_back(0);
			}
			else if (f == "vt") {
				tmptextures.push_back(Vertex(x, y, z));
			}
			else if (f == "vn") {
				tmpnormals.push_back(Vertex(x, y, z));
				//normals.push_back(Vertex(x, y, z));
			}
			else if (f == "f") {
				string newLine = line.substr(2, line.length());
				string delimiter1 = " ";
				string delimiter2 = "/";
				size_t pos = 0;
				string token;
				vector<string> ss;
				vector<string> index;
				vector<string> texture;
				vector<string> normal;

				/*** Truncate type area ***/
				while ((pos = newLine.find(delimiter1)) != string::npos) {
					token = newLine.substr(0, pos);
					ss.push_back(token);
					newLine.erase(0, pos + delimiter1.length());
				}
				ss.push_back(newLine);
				/**************************/

				/*** Get values ***/
				while (!ss.empty()) {
					string sn = ss.back();
					int count = 0;
					//int empty = 1;
					while ((pos = sn.find(delimiter2)) != string::npos) {
						//empty = 0;
						token = sn.substr(0, pos);
						if (count == 0)
							index.push_back(token);
						else if (count == 1 && !tmptextures.empty())
							texture.push_back(token);
						sn.erase(0, pos + delimiter2.length());
						count++;
					}
					//if (empty)
						//index.push_back(sn);
					if (count == 0)
						index.push_back(sn);
					//if (!empty && count == 2 )
					if (count == 2)
						normal.push_back(sn);

					ss.pop_back();
				}
				/******************/

				if (index.size() == 4) {
					int i0 = atoi(index[3].c_str());
					int i1 = atoi(index[2].c_str());
					int i2 = atoi(index[1].c_str());
					int i3 = atoi(index[0].c_str());

					if (i0 < 0) {
						i0 = vertices.size() + i0 + 1;
						i1 = vertices.size() + i1 + 1;
						i2 = vertices.size() + i2 + 1;
						i3 = vertices.size() + i3 + 1;
					}
					if (texture.size() > 0) {
						int t0 = atoi(texture[3].c_str());
						int t1 = atoi(texture[2].c_str());
						int t2 = atoi(texture[1].c_str());
						int t3 = atoi(texture[0].c_str());
						if (t0 < 0) {
							t0 = tmptextures.size() + t0 + 1;
							t1 = tmptextures.size() + t1 + 1;
							t2 = tmptextures.size() + t2 + 1;
							t3 = tmptextures.size() + t3 + 1;
						}
						if (tmptextures.size() > 0) {
							textures[i0 - 1] = tmptextures[t0 - 1];
							textures[i1 - 1] = tmptextures[t1 - 1];
							textures[i2 - 1] = tmptextures[t2 - 1];
							textures[i3 - 1] = tmptextures[t3 - 1];
						}
					}

					if (normal.size() > 0) {
						int n0 = atoi(normal[3].c_str());
						int n1 = atoi(normal[2].c_str());
						int n2 = atoi(normal[1].c_str());
						int n3 = atoi(normal[0].c_str());

						if (n0 < 0) {
							n0 = tmpnormals.size() + n0 + 1;
							n1 = tmpnormals.size() + n1 + 1;
							n2 = tmpnormals.size() + n2 + 1;
							n3 = tmpnormals.size() + n3 + 1;
						}

						if (tmpnormals.size() > 0) {
							normals[i0 - 1] = tmpnormals[n0 - 1];
							normals[i1 - 1] = tmpnormals[n1 - 1];
							normals[i2 - 1] = tmpnormals[n2 - 1];
							normals[i3 - 1] = tmpnormals[n3 - 1];
						}
					}

					indices.push_back(i0);
					indices.push_back(i1);
					indices.push_back(i3);
					meshes.push_back(Mesh(vertices[i0 - 1], vertices[i1 - 1], vertices[i3 - 1]));

					indices.push_back(i1);
					indices.push_back(i2);
					indices.push_back(i3);
					meshes.push_back(Mesh(vertices[i1 - 1], vertices[i2 - 1], vertices[i3 - 1]));
				}
				else if (index.size() == 3) {
					vector<int> tmpIdx;
					for (int i = index.size() - 1; i >= 0; i--) {
						int idx = atoi(index[i].c_str());
						if (texture.size() > 0) {
							int t = atoi(texture[i].c_str());
							if (t < 0) {
								t = tmptextures.size() + t + 1;
							}
							if (tmptextures.size() > 0)
								textures[idx - 1] = tmptextures[t - 1];
						}
						if (normal.size() > 0) {
							int n = atoi(normal[i].c_str());
							if (n < 0) {
								n = tmpnormals.size() + n + 1;
							}
							if (tmpnormals.size() > 0)
								normals[idx - 1] = tmpnormals[n - 1];
						}

						if (idx < 0) {
							idx = vertices.size() + idx + 1;
						}

						tmpIdx.push_back(idx);

						indices.push_back(idx);
					}
					meshes.push_back(Mesh(vertices[tmpIdx[0] - 1], vertices[tmpIdx[1] - 1], vertices[tmpIdx[2] - 1]));
				}
			}
		}

		if (jointFlag == 1) {
			istringstream iss(line);
			float x, y, z;
			iss >> x >> y >> z;

			Joint joint = Joint(joint_idx, Vertex(x, y, z));
			joints.push_back(joint);
			joint_idx++;
		}

		if (landmarkFlag == 1) {
			if (line == "") {
				landmark_line = -1;
			}
 
			if (landmark_line == 0) {
				l = Landmark();
				l.name = new char(strlen(line.c_str()) + 1);
				strcpy(l.name, line.c_str());
			}
			else if (landmark_line == 1) {
				istringstream iss(line);
				/*
				float t = 0;
				iss >> t;
				l.type = t;
				*/
				l.type = stof(line);
			}
			else if (landmark_line == 2) {
				istringstream iss(line);
				/*
				float t = 0;
				iss >> t;
				l.level = t;
				*/
				l.level = stof(line);
			}
			else if (landmark_line == 3) {
				istringstream iss(line);
				/*
				float t = 0;
				iss >> t;
				l.value = t;
				*/
				l.value = stof(line);
			}
			else if (landmark_line == 4) {
				string delimiter1 = " ";
				size_t pos = 0;
				string token;
				vector<string> ss;

				while ((pos = line.find(delimiter1)) != string::npos) {
					token = line.substr(0, pos);
					ss.push_back(token);
					line.erase(0, pos + delimiter1.length());
				}

				for (int i = 0; i < ss.size(); i++) {
					l.region.push_back(atoi(ss[i].c_str()));
				}
			}
			else if (landmark_line == 5) {
				string delimiter1 = " ";
				size_t pos = 0;
				string token;
				vector<string> ss;

				while ((pos = line.find(delimiter1)) != string::npos) {
					token = line.substr(0, pos);
					ss.push_back(token);
					line.erase(0, pos + delimiter1.length());
				}

				for (int i = 0; i < ss.size(); i++) {
					l.vertIdx.push_back(atoi(ss[i].c_str()));
				}
				landmarks.push_back(l);
			}

			landmark_line++;
		}
	
		if (segmentFlag == 1) {
			string delimiter = " ";
			size_t pos = 0;
			string token;

			while ((pos = line.find(delimiter)) != string::npos) {
				token = line.substr(0, pos);
				segmentHash[segment_idx].push_back(stof(token));
				line.erase(0, pos + delimiter.length());
			}
			segment_idx++;
		}

		if (weightFlag == 1) {
			string delimiter = " ";
			size_t pos = 0;
			string token;

			while ((pos = line.find(delimiter)) != string::npos) {
				token = line.substr(0, pos);
				weightHash[weight_idx].push_back(stof(token));
				line.erase(0, pos + delimiter.length());
			}
			weight_idx++;
		}

		if (tposeFlag == 1) {
			istringstream iss(line);
			int part, i_axis;
			float val;
			iss >> part >> i_axis >> val;

			skinning.setTPose(part, i_axis, val);
		}
	}

	/*** Assign tri-mesh ids to vertices ***/
	for (int i = 0; i < meshes.size(); i++) {
		int index1 = meshes[i].index1;
		int index2 = meshes[i].index2;
		int index3 = meshes[i].index3;

		vertices[index1 - 1].meshIds.push_back(i);
		vertices[index2 - 1].meshIds.push_back(i);
		vertices[index3 - 1].meshIds.push_back(i);
	}

	topMostLevel = maxY;
	bottomMostLevel = minY;
	leftMostOffset = minX;
	rightMostOffset = maxX;

	centering((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2);

	float unit = abs(maxZ - minZ);
	if (unit < 10) {
		scaleFactor = 1;
	}
	else if (unit >= 10) {
		scaleFactor = 0.1;
	}
	else if (unit >= 100) {
		scaleFactor = 0.01;
	}
	else if (unit >= 1000) {
		scaleFactor = 0.001;
	}
	else if (unit >= 10000) {
		scaleFactor = 0.0001;
	}
	else if (unit >= 100000) {
		scaleFactor = 0.00001;
	}

	topMostLevel -= (minY + maxY) / 2;
	bottomMostLevel -= (minY + maxY) / 2;
	leftMostLevel -= (minY + maxY) / 2;
	rightMostLevel -= (minY + maxY) / 2;
	leftMostOffset -= (minX + maxX) / 2;
	rightMostOffset -= (minX + maxX) / 2;

	basics.close();

	setRigs();

	origVertices.insert(origVertices.end(), vertices.begin(), vertices.end());
	origJoints.insert(origJoints.end(), joints.begin(), joints.end());
	origTopMostLevel = topMostLevel;
	origBottomMostLevel = bottomMostLevel;
}

void loadOBJ(char* dir) {
	vertices.clear();
	origVertices.clear();
	textures.clear();
	normals.clear();

	int idx = 1;
	string line;
	ifstream basics(dir);
	float maxX = -1000000000, minX = 1000000000;
	float maxY = -1000000000, minY = 1000000000;
	float maxZ = -1000000000, minZ = 1000000000;

	bustLevel = 0;
	waistLevel = 0;
	hipLevel = 0;

	vector<Vertex> tmpnormals, tmptextures;
	while (getline(basics, line)) {
		istringstream iss(line);
		string f;
		float x = -1, y = -1, z = -1, w = -1;

		iss >> f >> x >> y >> z >> w;

		if (f == "v") {
			if (minX > x) {
				minX = x;
				leftMostLevel = x;
			}
			if (x > maxX) {
				maxX = x;
				rightMostLevel = y;
			}
			if (minY > y) {
				minY = y;
				bottomMostIndex = idx - 1;
			}
			if (y > maxY) {
				maxY = y;
				topMostIndex = idx - 1;
			}
			if (minZ > z) {
				minZ = z;
			}
			if (z > maxZ) {
				maxZ = z;
			}

			vertices.push_back(Vertex(idx, x, y, z));
			textures.push_back(Vertex());
			normals.push_back(Vertex());
			idx++;
			/* Initialize vertex-joint group */
			segmentGroup.push_back(0);
		}
		else if (f == "vt") {
			tmptextures.push_back(Vertex(x, y, z));
		}
		else if (f == "vn") {
			tmpnormals.push_back(Vertex(x, y, z));
			//normals.push_back(Vertex(x, y, z));
		}
		else if (f == "f") {
			string newLine = line.substr(2, line.length());
			string delimiter1 = " ";
			string delimiter2 = "/";
			size_t pos = 0;
			string token;
			vector<string> ss;
			vector<string> index;
			vector<string> texture;
			vector<string> normal;

			/*** Truncate type area ***/
			while ((pos = newLine.find(delimiter1)) != string::npos) {
				token = newLine.substr(0, pos);
				ss.push_back(token);
				newLine.erase(0, pos + delimiter1.length());
			}
			ss.push_back(newLine);
			/**************************/

			/*** Get values ***/
			while (!ss.empty()) {
				string sn = ss.back();
				int count = 0;
				//int empty = 1;
				while ((pos = sn.find(delimiter2)) != string::npos) {
					//empty = 0;
					token = sn.substr(0, pos);
					if (count == 0)
						index.push_back(token);
					else if (count == 1 && !tmptextures.empty())
						texture.push_back(token);
					sn.erase(0, pos + delimiter2.length());
					count++;
				}
				//if (empty)
					//index.push_back(sn);
				if (count == 0)
					index.push_back(sn);
				//if (!empty && count == 2 )
				if (count == 2)
					normal.push_back(sn);

				ss.pop_back();
			}
			/******************/

			if (index.size() == 4) {
				int i0 = atoi(index[3].c_str());
				int i1 = atoi(index[2].c_str());
				int i2 = atoi(index[1].c_str());
				int i3 = atoi(index[0].c_str());

				if (i0 < 0) {
					i0 = vertices.size() + i0 + 1;
					i1 = vertices.size() + i1 + 1;
					i2 = vertices.size() + i2 + 1;
					i3 = vertices.size() + i3 + 1;
				}
				if (texture.size() > 0) {
					int t0 = atoi(texture[3].c_str());
					int t1 = atoi(texture[2].c_str());
					int t2 = atoi(texture[1].c_str());
					int t3 = atoi(texture[0].c_str());
					if (t0 < 0) {
						t0 = tmptextures.size() + t0 + 1;
						t1 = tmptextures.size() + t1 + 1;
						t2 = tmptextures.size() + t2 + 1;
						t3 = tmptextures.size() + t3 + 1;
					}
					if (tmptextures.size() > 0) {
						textures[i0 - 1] = tmptextures[t0 - 1];
						textures[i1 - 1] = tmptextures[t1 - 1];
						textures[i2 - 1] = tmptextures[t2 - 1];
						textures[i3 - 1] = tmptextures[t3 - 1];
					}
				}

				if (normal.size() > 0) {
					int n0 = atoi(normal[3].c_str());
					int n1 = atoi(normal[2].c_str());
					int n2 = atoi(normal[1].c_str());
					int n3 = atoi(normal[0].c_str());

					if (n0 < 0) {
						n0 = tmpnormals.size() + n0 + 1;
						n1 = tmpnormals.size() + n1 + 1;
						n2 = tmpnormals.size() + n2 + 1;
						n3 = tmpnormals.size() + n3 + 1;
					}

					if (tmpnormals.size() > 0) {
						normals[i0 - 1] = tmpnormals[n0 - 1];
						normals[i1 - 1] = tmpnormals[n1 - 1];
						normals[i2 - 1] = tmpnormals[n2 - 1];
						normals[i3 - 1] = tmpnormals[n3 - 1];
					}
				}

				indices.push_back(i0);
				indices.push_back(i1);
				indices.push_back(i3);
				meshes.push_back(Mesh(vertices[i0 - 1], vertices[i1 - 1], vertices[i3 - 1]));

				indices.push_back(i1);
				indices.push_back(i2);
				indices.push_back(i3);
				meshes.push_back(Mesh(vertices[i1 - 1], vertices[i2 - 1], vertices[i3 - 1]));
			}
			else if (index.size() == 3) {
				vector<int> tmpIdx;
				for (int i = index.size() - 1; i >= 0; i--) {
					int idx = atoi(index[i].c_str());
					if (texture.size() > 0) {
						int t = atoi(texture[i].c_str());
						if (t < 0) {
							t = tmptextures.size() + t + 1;
						}
						if (tmptextures.size() > 0)
							textures[idx - 1] = tmptextures[t - 1];
					}
					if (normal.size() > 0) {
						int n = atoi(normal[i].c_str());
						if (n < 0) {
							n = tmpnormals.size() + n + 1;
						}
						if (tmpnormals.size() > 0)
							normals[idx - 1] = tmpnormals[n - 1];
					}

					if (idx < 0) {
						idx = vertices.size() + idx + 1;
					}

					tmpIdx.push_back(idx);

					indices.push_back(idx);
				}
				meshes.push_back(Mesh(vertices[tmpIdx[0] - 1], vertices[tmpIdx[1] - 1], vertices[tmpIdx[2] - 1]));
			}
		}
	}

	/*** Assign tri-mesh ids to vertices ***/
	for (int i = 0; i < meshes.size(); i++) {
		int index1 = meshes[i].index1;
		int index2 = meshes[i].index2;
		int index3 = meshes[i].index3;

		vertices[index1 - 1].meshIds.push_back(i);
		vertices[index2 - 1].meshIds.push_back(i);
		vertices[index3 - 1].meshIds.push_back(i);
	}

	topMostLevel = maxY;
	bottomMostLevel = minY;
	leftMostOffset = minX;
	rightMostOffset = maxX;

	centering((minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2);

	float unit = abs(maxZ - minZ);
	if (unit >= 10)
		scaleFactor = 0.1;
	else if (unit >= 100)
		scaleFactor = 0.01;
	else if (unit >= 1000)
		scaleFactor = 0.001;
	else if (unit >= 10000)
		scaleFactor = 0.0001;
	else if (unit >= 100000)
		scaleFactor = 0.00001;


	topMostLevel -= (minY + maxY) / 2;
	bottomMostLevel -= (minY + maxY) / 2;
	leftMostLevel -= (minY + maxY) / 2;
	rightMostLevel -= (minY + maxY) / 2;
	leftMostOffset -= (minX + maxX) / 2;
	rightMostOffset -= (minX + maxX) / 2;

	skinning = Skinning();
	origVertices.insert(origVertices.end(), vertices.begin(), vertices.end());
	origTopMostLevel = topMostLevel;
	origBottomMostLevel = bottomMostLevel;
}

void loadJoints(char* dir) {
	joints.clear();
	origJoints.clear();

	ifstream infile(dir);
	string s;
	int idx = 0;
	int line = 0;
	Joint joint;

	while (getline(infile, s)) {
		if (s == "") {
			line = -1;
		}
		if (line == 0) {
			istringstream iss(s);
			float x, y, z;
			iss >> x >> y >> z;
			joint = Joint(idx, Vertex(x, y, z));
			idx++;
			joints.push_back(joint);
		}
		line++;
	}
	infile.close();

	origJoints.insert(origJoints.end(), joints.begin(), joints.end());

	setRigs();
}

void loadLandmarks(char* dir) {
	landmarks.clear();

	ifstream infile(dir);
	string s;
	int line = 0;

	Landmark l;
	while (getline(infile, s)) {
		if (s == "") {
			line = -1;
		}

		if (line == 0) {
			l = Landmark();
			l.name = new char(strlen(s.c_str()) + 1);
			strcpy(l.name, s.c_str());
		}
		else if (line == 1) {
			istringstream iss(s);
			float t = 0;
			iss >> t;
			l.type = t;
		}
		else if (line == 2) {
			istringstream iss(s);
			float t = 0;
			iss >> t;
			l.level = t;
		}
		else if (line == 3) {
			istringstream iss(s);
			float t = 0;
			iss >> t;
			l.value = t;
		}
		else if (line == 4) {
			string delimiter1 = " ";
			size_t pos = 0;
			string token;
			vector<string> ss;

			while ((pos = s.find(delimiter1)) != string::npos) {
				token = s.substr(0, pos);
				ss.push_back(token);
				s.erase(0, pos + delimiter1.length());
			}

			for (int i = 0; i < ss.size(); i++) {
				l.region.push_back(atoi(ss[i].c_str()));
			}
		}
		else if (line == 5) {
			string delimiter1 = " ";
			size_t pos = 0;
			string token;
			vector<string> ss;

			while ((pos = s.find(delimiter1)) != string::npos) {
				token = s.substr(0, pos);
				ss.push_back(token);
				s.erase(0, pos + delimiter1.length());
			}

			for (int i = 0; i < ss.size(); i++) {
				l.vertIdx.push_back(atoi(ss[i].c_str()));
			}
			landmarks.push_back(l);
		}

		line++;
	}

	infile.close();
}

void loadPose(char* dir) {
	skinning.setPose(dir);
}

void initHuman() {
	vertices.clear();
	joints.clear();

	vertices.insert(vertices.end(), origVertices.begin(), origVertices.end());
	joints.insert(joints.end(), origJoints.begin(), origJoints.end());

	/*
	height = origHeight;
	bustSize = origBustSize;
	waistSize = origWaistSize;
	hipSize = origHipSize;

	bustLevel = origBustLevel;
	waistLevel = origWaistLevel;
	hipLevel = origHipLevel;

	landmarks[0].value = origHeight;

	landmarks[1].value = origBustSize;
	landmarks[1].level = origBustLevel;

	landmarks[2].value = origWaistSize;
	landmarks[2].level = origWaistLevel;

	landmarks[3].value = origHipSize;
	landmarks[3].level = origHipLevel;

	topMostLevel = origTopMostLevel;
	bottomMostLevel = origBottomMostLevel;
	*/
}

void setPose(float r) {
	skinning.history.clear();

	for (int i = 0; i < skinning.history.size(); i++) {
		poseHistory his = skinning.history[i];
		float currAngle = 0;
		Vertex currJoint;
		his.value *= r;

		skinning.axis = his.axis;

		if (his.part == Joint_shoulderR) {
			//currJoint = joints[Joint_elbowR].getCoord();
			//currAngle = skinning.getAngle(his.axis, currJoint);

			skinning.rotateArmR(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_shoulderL) {
			skinning.rotateArmL(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_elbowR) {
			skinning.rotateElbowR(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_elbowL) {
			skinning.rotateElbowL(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_elbowL) {
			skinning.rotateElbowL(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_pelvisR) {
			skinning.rotateLegR(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_pelvisL) {
			skinning.rotateLegL(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_kneeR) {
			skinning.rotateKneeR(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_kneeL) {
			skinning.rotateKneeL(his.part, his.value, vertices, joints);
		}
	}
}

void setTPose(float r) {
	for (int i = 0; i < skinning.thistory.size(); i++) {
		poseHistory his = skinning.thistory[i];
		skinning.axis = his.axis;

		his.value *= r;

		if (his.part == Joint_shoulderR) {
			skinning.rotateArmR(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_shoulderL) {
			skinning.rotateArmL(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_elbowR) {
			skinning.rotateElbowR(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_elbowL) {
			skinning.rotateElbowL(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_pelvisR) {
			skinning.rotateLegR(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_pelvisL) {
			skinning.rotateLegL(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_kneeR) {
			skinning.rotateKneeR(his.part, his.value, vertices, joints);
		}
		else if (his.part == Joint_kneeL) {
			skinning.rotateKneeL(his.part, his.value, vertices, joints);
		}
	}
}

void getVert(int i, float* coord) {
	coord[0] = vertices[i].x;
	coord[1] = vertices[i].y;
	coord[2] = vertices[i].z;
}

void getVerts(float* coord) {
	int j = 0;
	for (int i = 0; i < vertices.size(); i++) {
		coord[j++] = vertices[i].x;
		coord[j++] = vertices[i].y;
		coord[j++] = vertices[i].z;
	}
}

void getMesh(int i, int* node) {
	node[0] = meshes[i].index1;
	node[1] = meshes[i].index2;
	node[2] = meshes[i].index3;
}

void getIndices(int* node) {
	int j = 0;
	for (int i = 0; i < indices.size(); i += 3) {
		node[j++] = indices[i];
		node[j++] = indices[i + 1];
		node[j++] = indices[i + 2];
	}
}

float getSize(int i) {
	return landmarks[i].value;
}

int getVertSize() {
	return vertices.size();
}

int getMeshSize() {
	return meshes.size();
}

int getLandmarkNum() {
	return landmarks.size();
}

void setSize(float s, int type, int index, float oldSize, float newSize) {
	float scale = newSize / oldSize;
	vector<int> sections(landmarks[index].region);
	float movement = 0;

	int upperTorsoFlag = 0, lowerTorsoFlag = 0;

	if (index == 0 || type == Girth) { // Handle height here
		float level = landmarks[index].level;

		float topLevel = 0;
		float bottomLevel = 0;

		vector<float> bigger, smaller;

		for (int i = 0; i < landmarks.size(); i++) {
			if (i != index) {
				float thisLevel = landmarks[i].level;
				if (thisLevel > level) {
					bigger.push_back(thisLevel);
				}
				else if (thisLevel < level) {
					smaller.push_back(thisLevel);
				}
			}
		}

		sort(bigger.begin(), bigger.end());
		sort(smaller.begin(), smaller.end(), greater<float>());

		if (!bigger.empty()) {
			topLevel = bigger[0];
		}
		else {
			topLevel = topMostLevel;
		}
		if (!smaller.empty()) {
			bottomLevel = smaller[0];
		}
		else {
			bottomLevel = bottomMostLevel;
		}

		float range = 1 / s;
		int trial = 0;

		vector<int> rangeVs;
		while (rangeVs.size() < 12 && trial < 20) {
			rangeVs.clear();

			for (int i = 0; i < sections.size(); i++) {
				for (int j = 0; j < segmentHash[sections[i]].size(); j++) {
					int idx = segmentHash[sections[i]][j];

					if (abs(vertices[idx].y - level) < range) {
						rangeVs.push_back(idx);
					}
				}
			}

			range += range / 8;
			trial++;
		}

		for (int i = 0; i < sections.size(); i++) {
			for (int j = 0; j < segmentHash[sections[i]].size(); j++) {
				int idx = segmentHash[sections[i]][j];

				if (abs(vertices[idx].y - level) < range) {
					vertices[idx].x *= scale;
					vertices[idx].z *= scale;
				}
				else if (vertices[idx].y < topLevel && vertices[idx].y >= level + range) {
					Vertex* v = &vertices[idx];
					Vertex v_upper = vertices[idx];
					Vertex v_lower = vertices[idx];

					float toUpper = (topLevel)-v->y;
					float toLower = v->y - (level + range);

					v_upper.x *= 1;
					v_upper.z *= 1;
					v_lower.x *= scale;
					v_lower.z *= scale;

					v->x = (v_upper.x * toLower + v_lower.x * toUpper) / (toUpper + toLower);
					v->z = (v_upper.z * toLower + v_lower.z * toUpper) / (toUpper + toLower);
				}
				else if (vertices[idx].y <= level - range && vertices[idx].y > bottomLevel) {
					Vertex* v = &vertices[idx];
					Vertex v_upper = vertices[idx];
					Vertex v_lower = vertices[idx];

					float toUpper = (level - range) - v->y;
					float toLower = v->y - (bottomLevel);

					v_upper.x *= scale;
					v_upper.z *= scale;
					v_lower.x *= 1;
					v_lower.z *= 1;

					v->x = (v_upper.x * toLower + v_lower.x * toUpper) / (toUpper + toLower);
					v->z = (v_upper.z * toLower + v_lower.z * toUpper) / (toUpper + toLower);
				}
			}
		}
	}
	else if (type == Length) {
		/*** TODO :: Deformations of length type landmarks other than Upper/Lower Torso ***/

		for (int i = 0; i < sections.size(); i++) {
			for (int j = 0; j < segmentHash[sections[i]].size(); j++) {
				int idx = segmentHash[sections[i]][j];

				if (sections[i] == Segment_UpperTorso) {
					upperTorsoFlag = 1;
					float jointDiff = joints[Joint_waist].getCoord().y*scale - joints[Joint_waist].getCoord().y;
					Vertex shoulderMid = joints[Joint_shoulderMid].getCoord();
					Vertex waist = joints[Joint_waist].getCoord();

					float y_start = shoulderMid.y;
					float y_end = waist.y;

					float da = y_start - vertices[idx].y;
					float db = vertices[idx].y - y_end;

					vertices[idx].y = (y_start * db + (y_end - jointDiff) * da) / (da + db);
				}
				else if (sections[i] == Segment_LowerTorso) {
					lowerTorsoFlag = 1;
					float jointDiff = joints[Joint_waist].getCoord().y*scale - joints[Joint_waist].getCoord().y;

					Vertex waist = joints[Joint_waist].getCoord();
					Vertex pelvisMid = joints[Joint_pelvisMid].getCoord();

					float y_start = waist.y;
					float y_end = pelvisMid.y;

					float da = y_start - vertices[idx].y;
					float db = vertices[idx].y - y_end;

					vertices[idx].y = ((y_start - jointDiff)* db + y_end * da) / (da + db);
				}
				else {
					vertices[idx].y *= scale;
				}
			}
		}
	}

	if (upperTorsoFlag) {
		float jointDiff = joints[Joint_waist].getCoord().y*scale - joints[Joint_waist].getCoord().y;
		for (int i = 0; i < segmentHash[Segment_LowerTorso].size(); i++) {
			int idx = segmentHash[Segment_LowerTorso][i];
			Vertex waist = joints[Joint_waist].getCoord();
			Vertex pelvisMid = joints[Joint_pelvisMid].getCoord();

			float y_start = waist.y;
			float y_end = pelvisMid.y;

			float da = y_start - vertices[idx].y;
			float db = vertices[idx].y - y_end;

			vertices[idx].y = ((y_start - jointDiff)* db + y_end * da) / (da + db);
		}
	}
	if (lowerTorsoFlag) {
		float jointDiff = joints[Joint_waist].getCoord().y*scale - joints[Joint_waist].getCoord().y;
		for (int i = 0; i < segmentHash[Segment_UpperTorso].size(); i++) {
			int idx = segmentHash[Segment_UpperTorso][i];
			Vertex shoulderMid = joints[Joint_shoulderMid].getCoord();
			Vertex waist = joints[Joint_waist].getCoord();

			float y_start = shoulderMid.y;
			float y_end = waist.y;

			float da = y_start - vertices[idx].y;
			float db = vertices[idx].y - y_end;

			vertices[idx].y = (y_start * db + (y_end - jointDiff) * da) / (da + db);
		}
	}

	updateRigs();
}

bool setThreeSize(float s, float* sizes) {
	float bustScale = sizes[1] / landmarks[1].value;
	float waistScale = sizes[2] / landmarks[2].value;
	float hipScale = sizes[3] / landmarks[3].value;
	float armHoleMovementRx = 0;
	float armHoleMovementLx = 0;

	float heightScale = sizes[0] / landmarks[0].value;

	vector<int> armRIdx;
	vector<int> armLIdx;
	vector<int> vertIdx;
	armRIdx.insert(armRIdx.end(), segmentHash[Segment_UpperArmR].begin(), segmentHash[Segment_UpperArmR].end());
	armRIdx.insert(armRIdx.end(), segmentHash[Segment_LowerArmR].begin(), segmentHash[Segment_LowerArmR].end());
	armRIdx.insert(armRIdx.end(), segmentHash[Segment_HandR].begin(), segmentHash[Segment_HandR].end());

	armLIdx.insert(armLIdx.end(), segmentHash[Segment_UpperArmL].begin(), segmentHash[Segment_UpperArmL].end());
	armLIdx.insert(armLIdx.end(), segmentHash[Segment_LowerArmL].begin(), segmentHash[Segment_LowerArmL].end());
	armLIdx.insert(armLIdx.end(), segmentHash[Segment_HandL].begin(), segmentHash[Segment_HandL].end());

	vertIdx.insert(vertIdx.end(), segmentHash[Segment_Head].begin(), segmentHash[Segment_Head].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_Neck].begin(), segmentHash[Segment_Neck].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_UpperTorso].begin(), segmentHash[Segment_UpperTorso].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_LowerTorso].begin(), segmentHash[Segment_LowerTorso].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_UpperLegR].begin(), segmentHash[Segment_UpperLegR].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_LowerLegR].begin(), segmentHash[Segment_LowerLegR].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_FootR].begin(), segmentHash[Segment_FootR].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_UpperLegL].begin(), segmentHash[Segment_UpperLegL].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_LowerLegL].begin(), segmentHash[Segment_LowerLegL].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_FootL].begin(), segmentHash[Segment_FootL].end());

	float range = 1 / (s * 8);
	for (int i = 0; i < vertIdx.size(); i++) {
		int idx = vertIdx[i];
		if (abs(vertices[idx].y - bustLevel) < range) {
			vertices[idx].x *= bustScale;
			vertices[idx].z *= bustScale;
			vertices[idx].y *= heightScale;
		}
		else if (abs(vertices[idx].y - waistLevel) < range) {
			vertices[idx].x *= waistScale;
			vertices[idx].z *= waistScale;
			vertices[idx].y *= heightScale;
		}
		else if (abs(vertices[idx].y - hipLevel) < range) {
			vertices[idx].x *= hipScale;
			vertices[idx].z *= hipScale;
			vertices[idx].y *= heightScale;
		}
		else if (vertices[idx].y >= bustLevel + range) {
			if (idx == shoulderRIndex) {
				armHoleMovementRx = vertices[idx].x*bustScale - vertices[idx].x;
			}
			if (idx == shoulderLIndex) {
				armHoleMovementLx = vertices[idx].x*bustScale - vertices[idx].x;
			}
			Vertex* v = &vertices[idx];
			Vertex vn = vertices[idx];
			Vertex vb = vertices[idx];

			float a = (topMostLevel - v->y);
			float b = (v->y - bustLevel);

			vn.x *= 1;
			vn.z *= 1;
			vb.x *= bustScale;
			vb.z *= bustScale;

			v->x = (vn.x*b + vb.x*a) / (a + b);
			v->z = (vn.z*b + vb.z*a) / (a + b);
			v->y *= heightScale;

		}
		else if (vertices[idx].y <= bustLevel - range && vertices[idx].y >= waistLevel + range) {
			Vertex* v = &vertices[idx];
			Vertex vb = vertices[idx];
			Vertex vw = vertices[idx];

			float a = (bustLevel - v->y);
			float b = (v->y - waistLevel);

			vb.x *= bustScale;
			vb.z *= bustScale;
			vw.x *= waistScale;
			vw.z *= waistScale;

			v->x = (vb.x*b + vw.x*a) / (a + b);
			v->z = (vb.z*b + vw.z*a) / (a + b);
			v->y *= heightScale;
		}
		else if (vertices[idx].y <= waistLevel - range && vertices[idx].y >= hipLevel + range) {
			Vertex* v = &vertices[idx];
			Vertex vw = vertices[idx];
			Vertex vh = vertices[idx];

			float a = (waistLevel - v->y);
			float b = (v->y - hipLevel);

			vw.x *= waistScale;
			vw.z *= waistScale;
			vh.x *= hipScale;
			vh.z *= hipScale;

			v->x = (vw.x*b + vh.x * a) / (a + b);
			v->z = (vw.z*b + vh.z * a) / (a + b);
			v->y *= heightScale;
		}
		else if (vertices[idx].y <= hipLevel - range) {
			Vertex* v = &vertices[idx];
			Vertex vh = vertices[idx];
			Vertex vf = vertices[idx];

			float a = (hipLevel - v->y);
			float b = (v->y - bottomMostLevel);

			vh.x *= hipScale;
			vh.z *= hipScale;
			vf.x *= 1;
			vf.z *= 1;

			v->x = (vh.x*b + vf.x * a) / (a + b);
			v->z = (vh.z*b + vf.z * a) / (a + b);
			v->y *= heightScale;
		}
	}
	/*** Move arm ***/
	for (int j = 0; j < armRIdx.size(); j++) {
		vertices[armRIdx[j]].x += armHoleMovementRx;
		vertices[armRIdx[j]].y *= heightScale;
	}
	for (int j = 0; j < armLIdx.size(); j++) {
		vertices[armLIdx[j]].x += armHoleMovementLx;
		vertices[armLIdx[j]].y *= heightScale;

	}

	/*** Manual joint update ***/
	for (int i = 0; i < joints.size(); i++) {
		Vertex joint = joints[i].getCoord();
		joints[i].setCoord(joint.x, joint.y * heightScale, joint.z);
	}
	/***************************/

	bustLevel *= heightScale;
	waistLevel *= heightScale;
	hipLevel *= heightScale;
	topMostLevel *= heightScale;
	bottomMostLevel *= heightScale;

	bustSize = getBustSize();
	waistSize = getWaistSize();
	hipSize = getHipSize();

	landmarks[0].value = topMostLevel - bottomMostLevel;
	landmarks[1].value = bustSize;
	landmarks[2].value = waistSize;
	landmarks[3].value = hipSize;

	for (int i = 0; i < landmarks.size(); i++) {
		landmarks[i].level *= heightScale;
	}

	//updateRigs();

	return true;
}

void setHeight(float h) {
	float bustScale = 1;
	float waistScale = 1;
	float hipScale = 1;
	float heightScale = h / height;

	if (abs(1 - heightScale) < thresholdScale || abs(h - height) < thresholdSize) {
		return;
	}

	vector<int> vertIdx;

	vertIdx.insert(vertIdx.end(), segmentHash[Segment_UpperArmR].begin(), segmentHash[Segment_UpperArmR].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_LowerArmR].begin(), segmentHash[Segment_LowerArmR].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_HandR].begin(), segmentHash[Segment_HandR].end());

	vertIdx.insert(vertIdx.end(), segmentHash[Segment_UpperArmL].begin(), segmentHash[Segment_UpperArmL].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_LowerArmL].begin(), segmentHash[Segment_LowerArmL].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_HandL].begin(), segmentHash[Segment_HandL].end());

	vertIdx.insert(vertIdx.end(), segmentHash[Segment_Head].begin(), segmentHash[Segment_Head].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_Neck].begin(), segmentHash[Segment_Neck].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_UpperTorso].begin(), segmentHash[Segment_UpperTorso].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_LowerTorso].begin(), segmentHash[Segment_LowerTorso].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_UpperLegR].begin(), segmentHash[Segment_UpperLegR].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_LowerLegR].begin(), segmentHash[Segment_LowerLegR].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_FootR].begin(), segmentHash[Segment_FootR].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_UpperLegL].begin(), segmentHash[Segment_UpperLegL].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_LowerLegL].begin(), segmentHash[Segment_LowerLegL].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_FootL].begin(), segmentHash[Segment_FootL].end());

	for (int i = 0; i < vertIdx.size(); i++) {
		int idx = vertIdx[i];
		vertices[idx].y *= heightScale;
	}

	for (int i = 0; i < joints.size(); i++) {
		Vertex j = joints[i].getCoord();
		joints[i].setCoord(j.x, j.y * heightScale, j.z);
	}

	topMostLevel = vertices[topMostIndex].y;
	bottomMostLevel = vertices[bottomMostIndex].y;
	height = topMostLevel - bottomMostLevel;

	landmarks[0].value = height;
	for (int i = 1; i < landmarks.size(); i++) {
		landmarks[i].level *= heightScale;
	}

	bustLevel = landmarks[1].level;
	waistLevel = landmarks[2].level;
	hipLevel = landmarks[3].level;

}

void setBustSize(float s, float b) {
	float bustScale = b / bustSize;
	float waistScale = 1;
	float armHoleMovementRx = 0;
	float armHoleMovementLx = 0;

	if (abs(1 - bustScale) <= thresholdScale || abs(b - bustSize) <= thresholdSize) {
		return;
	}

	vector<int> armRIdx;
	vector<int> armLIdx;
	vector<int> vertIdx;

	vector<int> relatedJointsR;
	vector<int> relatedJointsL;

	armRIdx.insert(armRIdx.end(), segmentHash[Segment_UpperArmR].begin(), segmentHash[Segment_UpperArmR].end());
	armRIdx.insert(armRIdx.end(), segmentHash[Segment_LowerArmR].begin(), segmentHash[Segment_LowerArmR].end());
	armRIdx.insert(armRIdx.end(), segmentHash[Segment_HandR].begin(), segmentHash[Segment_HandR].end());

	armLIdx.insert(armLIdx.end(), segmentHash[Segment_UpperArmL].begin(), segmentHash[Segment_UpperArmL].end());
	armLIdx.insert(armLIdx.end(), segmentHash[Segment_LowerArmL].begin(), segmentHash[Segment_LowerArmL].end());
	armLIdx.insert(armLIdx.end(), segmentHash[Segment_HandL].begin(), segmentHash[Segment_HandL].end());

	vertIdx.insert(vertIdx.end(), segmentHash[Segment_Head].begin(), segmentHash[Segment_Head].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_Neck].begin(), segmentHash[Segment_Neck].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_UpperTorso].begin(), segmentHash[Segment_UpperTorso].end());

	relatedJointsR.push_back(Joint_shoulderR);
	relatedJointsR.push_back(Joint_elbowR);
	relatedJointsR.push_back(Joint_wristR);

	relatedJointsL.push_back(Joint_shoulderL);
	relatedJointsL.push_back(Joint_elbowL);
	relatedJointsL.push_back(Joint_wristL);

	float range = 1 / (s * 8);
	int trial = 0;
	while (trial < maxTrials && (abs(1 - bustScale) > thresholdScale) || abs(b - bustSize) > thresholdSize) {
		bustScale = b / bustSize;

		for (int i = 0; i < vertIdx.size(); i++) {
			int idx = vertIdx[i];

			if (idx == shoulderRIndex) {
				armHoleMovementRx = vertices[idx].x*bustScale - vertices[idx].x;
			}
			if (idx == shoulderLIndex) {
				armHoleMovementLx = vertices[idx].x*bustScale - vertices[idx].x;
			}

			if (vertices[idx].y < bustLevel + range && vertices[idx].y > bustLevel - range) {
				vertices[idx].x *= bustScale;
				vertices[idx].z *= bustScale;
			}
			else if (vertices[idx].y >= bustLevel + range) {
				Vertex* v = &vertices[idx];
				Vertex vn = vertices[idx];
				Vertex vb = vertices[idx];

				float a = (topMostLevel - v->y);
				float b = (v->y - (bustLevel + range));

				vn.x *= 1;
				vn.z *= 1;
				vb.x *= bustScale;
				vb.z *= bustScale;

				v->x = (vn.x*b + vb.x*a) / (a + b);
				v->z = (vn.z*b + vb.z*a) / (a + b);
			}
			else if (vertices[idx].y <= bustLevel - range && vertices[idx].y >= waistLevel + range) {
				Vertex* v = &vertices[idx];
				Vertex vb = vertices[idx];
				Vertex vw = vertices[idx];

				float a = (bustLevel - range) - v->y;
				float b = v->y - (waistLevel + range);

				vb.x *= bustScale;
				vb.z *= bustScale;
				vw.x *= waistScale;
				vw.z *= waistScale;

				v->x = (vb.x*b + vw.x*a) / (a + b);
				v->z = (vb.z*b + vw.z*a) / (a + b);
			}
		}

		/*** Move arm ***/
		for (int i = 0; i < armRIdx.size(); i++) {
			vertices[armRIdx[i]].x += armHoleMovementRx;
		}
		for (int i = 0; i < armLIdx.size(); i++) {
			vertices[armLIdx[i]].x += armHoleMovementLx;
		}
		/****************/

		/*** Move joints ***/
		for (int i = 0; i < relatedJointsR.size(); i++) {
			Vertex j = joints[relatedJointsR[i]].getCoord();
			joints[relatedJointsR[i]].setCoord(j.x + armHoleMovementRx, j.y, j.z);
		}
		for (int i = 0; i < relatedJointsL.size(); i++) {
			Vertex j = joints[relatedJointsL[i]].getCoord();
			joints[relatedJointsL[i]].setCoord(j.x + armHoleMovementLx, j.y, j.z);
		}
		/*******************/
		bustSize = getBustSize();
		trial++;
	}

	landmarks[1].value = bustSize;
}

void setWaistSize(float s, float w) {
	float bustScale = 1;
	float waistScale = w / waistSize;
	float hipScale = 1;

	if (abs(1 - waistScale) <= thresholdScale || abs(w - waistSize) <= thresholdSize) {
		return;
	}

	vector<int> vertIdx;

	vertIdx.insert(vertIdx.end(), segmentHash[Segment_Head].begin(), segmentHash[Segment_Head].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_Neck].begin(), segmentHash[Segment_Neck].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_UpperTorso].begin(), segmentHash[Segment_UpperTorso].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_LowerTorso].begin(), segmentHash[Segment_LowerTorso].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_UpperLegR].begin(), segmentHash[Segment_UpperLegR].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_LowerLegR].begin(), segmentHash[Segment_LowerLegR].end());

	float range = 1 / (s * 8);
	int trial = 0;
	while (trial < maxTrials && (abs(1 - waistScale) > thresholdScale || abs(w - waistSize) > thresholdSize)) {
		waistScale = w / waistSize;

		for (int i = 0; i < vertIdx.size(); i++) {
			int idx = vertIdx[i];

			if (abs(vertices[idx].y - waistLevel) < range) {
				vertices[idx].x *= waistScale;
				vertices[idx].z *= waistScale;
			}
			else if (vertices[idx].y <= bustLevel - range && vertices[idx].y > waistLevel + range) {
				Vertex* v = &vertices[idx];
				Vertex vb = vertices[idx];
				Vertex vw = vertices[idx];

				float a = (bustLevel - range) - v->y;
				float b = v->y - (waistLevel + range);

				vb.x *= bustScale;
				vb.z *= bustScale;
				vw.x *= waistScale;
				vw.z *= waistScale;

				v->x = (vb.x*b + vw.x*a) / (a + b);
				v->z = (vb.z*b + vw.z*a) / (a + b);
			}
			else if (vertices[idx].y < waistLevel - range && vertices[idx].y > hipLevel + range) {
				Vertex* v = &vertices[idx];
				Vertex vw = vertices[idx];
				Vertex vh = vertices[idx];

				float a = (waistLevel - range) - v->y;
				float b = v->y - (hipLevel + range);

				vw.x *= waistScale;
				vw.z *= waistScale;
				vh.x *= hipScale;
				vh.z *= hipScale;

				v->x = (vw.x*b + vh.x*a) / (a + b);
				v->z = (vw.z*b + vh.z*a) / (a + b);
			}
		}

		waistSize = getWaistSize();
		trial++;
	}

	landmarks[2].value = waistSize;
}

void setHipSize(float s, float h) {
	float bustScale = 1;
	float waistScale = 1;
	float hipScale = h / hipSize;

	if (abs(1 - hipScale) <= thresholdScale || abs(h - hipSize) <= thresholdSize) {
		return;
	}

	vector<int> vertIdx;
	vector<int> relatedJoints;

	vertIdx.insert(vertIdx.end(), segmentHash[Segment_UpperTorso].begin(), segmentHash[Segment_UpperTorso].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_LowerTorso].begin(), segmentHash[Segment_LowerTorso].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_UpperLegR].begin(), segmentHash[Segment_UpperLegR].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_LowerLegR].begin(), segmentHash[Segment_LowerLegR].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_FootR].begin(), segmentHash[Segment_FootR].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_UpperLegL].begin(), segmentHash[Segment_UpperLegL].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_LowerLegL].begin(), segmentHash[Segment_LowerLegL].end());
	vertIdx.insert(vertIdx.end(), segmentHash[Segment_FootL].begin(), segmentHash[Segment_FootL].end());

	relatedJoints.push_back(Joint_pelvisR);
	relatedJoints.push_back(Joint_highLegR);
	relatedJoints.push_back(Joint_kneeR);
	relatedJoints.push_back(Joint_ankleR);
	relatedJoints.push_back(Joint_pelvisL);
	relatedJoints.push_back(Joint_highLegL);
	relatedJoints.push_back(Joint_kneeL);
	relatedJoints.push_back(Joint_ankleL);

	float range = 1 / (s * 8);
	int trial = 0;
	while (trial < maxTrials && (abs(1 - hipScale) > thresholdScale || abs(h - hipSize) > thresholdSize)) {
		hipScale = h / hipSize;

	for (int i = 0; i < vertIdx.size(); i++) {
		int idx = vertIdx[i];
		if (abs(vertices[idx].y - hipLevel) < range) {
			vertices[idx].x *= hipScale;
			vertices[idx].z *= hipScale;
		}
		else if (vertices[idx].y <= waistLevel - range && vertices[idx].y >= hipLevel + range) {
			Vertex* v = &vertices[idx];
			Vertex vw = vertices[idx];
			Vertex vh = vertices[idx];

			float a = (waistLevel - range) - v->y;
			float b = v->y - (hipLevel + range);

			vw.x *= waistScale;
			vw.z *= waistScale;
			vh.x *= hipScale;
			vh.z *= hipScale;

			v->x = (vw.x*b + vh.x * a) / (a + b);
			v->z = (vw.z*b + vh.z * a) / (a + b);
		}
		else if (vertices[idx].y <= hipLevel - range) {
			Vertex* v = &vertices[idx];
			Vertex vh = vertices[idx];
			Vertex vf = vertices[idx];

			float a = (hipLevel - range) - v->y;
			float b = (v->y - bottomMostLevel);

			vh.x *= hipScale;
			vh.z *= hipScale;
			vf.x *= 1;
			vf.z *= 1;

			v->x = (vh.x*b + vf.x * a) / (a + b);
			v->z = (vh.z*b + vf.z * a) / (a + b);
		}

	}

	for (int i = 0; i < relatedJoints.size(); i++) {
		Vertex j = joints[relatedJoints[i]].getCoord();
		Vertex j_h = joints[relatedJoints[i]].getCoord();
		Vertex j_f = joints[relatedJoints[i]].getCoord();

		float a = (hipLevel - range) - j.y;
		float b = j.y - bottomMostLevel;

		j_h.x *= hipScale;
		j_h.z *= hipScale;
		j_f.x *= 1;
		j_f.z *= 1;

		j.x = (j_h.x * b + j_f.x * a) / (a + b);
		j.z = (j_h.z * b + j_f.z * a) / (a + b);

		joints[relatedJoints[i]].setCoord(j.x, j.y, j.z);
	}

		hipSize = getHipSize();
		trial++;
	}

	landmarks[3].value = hipSize;
}

bool resize(float* sizes) {
	setBustSize(scaleFactor, sizes[1]);
	setWaistSize(scaleFactor, sizes[2]);
	setHipSize(scaleFactor, sizes[3]);
	setHeight(sizes[0]);

	currBustSize = getBustSize();
	currWaistSize = getWaistSize();
	currHipSize = getHipSize();
	currHeight = getHeight();

	return true;
}

void getSizeName(int i, char* n) {
	strcpy(n, landmarks[i].name);
}

int getSizePathLength(int i) {
	return landmarks[i].vertIdx.size();
}

void getSizePath(int idx, float* coord) {
	int j = 0;
	for (int i = 0; i < landmarks[i].vertIdx.size(); i++) {
		coord[j++] = vertices[landmarks[idx].vertIdx[i]].x;
		coord[j++] = vertices[landmarks[idx].vertIdx[i]].y;
		coord[j++] = vertices[landmarks[idx].vertIdx[i]].z;
	}
}

void getSizePathPos(int i, int n, float* coord) {
	coord[0] = vertices[landmarks[i].vertIdx[n]].x;
	coord[1] = vertices[landmarks[i].vertIdx[n]].y;
	coord[2] = vertices[landmarks[i].vertIdx[n]].z;
}

int getBodyPartNum(char* name) {
	if (!strcmp(name, "Head")) {
		return Segment_Head;
	}
	if (!strcmp(name, "Neck")) {
		return Segment_Neck;
	}
	if (!strcmp(name, "Upper Torso")) {
		return Segment_UpperTorso;
	}
	if (!strcmp(name, "Lower Torso")) {
		return Segment_LowerTorso;
	}
	if (!strcmp(name, "Upper Arm R")) {
		return Segment_UpperArmR;
	}
	if (!strcmp(name, "Upper Arm L")) {
		return Segment_UpperArmL;
	}
	if (!strcmp(name, "Lower Arm R")) {
		return Segment_LowerArmR;
	}
	if (!strcmp(name, "Lower Arm L")) {
		return Segment_LowerArmL;
	}
	if (!strcmp(name, "Hand R")) {
		return Segment_HandR;
	}
	if (!strcmp(name, "Hand L")) {
		return Segment_HandL;
	}
	if (!strcmp(name, "Upper Leg R")) {
		return Segment_UpperLegR;
	}
	if (!strcmp(name, "Upper Leg L")) {
		return Segment_UpperLegL;
	}
	if (!strcmp(name, "Lower Leg R")) {
		return Segment_LowerLegR;
	}
	if (!strcmp(name, "Lower Leg L")) {
		return Segment_LowerLegL;
	}
	if (!strcmp(name, "Foot R")) {
		return Segment_FootR;
	}
	if (!strcmp(name, "Foot L")) {
		return Segment_FootL;
	}

	return -1;
}

void getBodyPartOrigin(int part, float* coord) {
	if (part == Segment_Head) {
		coord[0] = vertices[topMostIndex].x;
		coord[1] = vertices[topMostIndex].y;
		coord[2] = vertices[topMostIndex].z;
	}
	else if (part == Segment_Neck) {
		coord[0] = joints[Joint_neck].getCoord().x;
		coord[1] = joints[Joint_neck].getCoord().y;
		coord[2] = joints[Joint_neck].getCoord().z;
	}
	else if (part == Segment_UpperTorso) {
		coord[0] = joints[Joint_shoulderMid].getCoord().x;
		coord[1] = joints[Joint_shoulderMid].getCoord().y;
		coord[2] = joints[Joint_shoulderMid].getCoord().z;
	}
	else if (part == Segment_LowerTorso) {
		coord[0] = joints[Joint_waist].getCoord().x;
		coord[1] = joints[Joint_waist].getCoord().y;
		coord[2] = joints[Joint_waist].getCoord().z;
	}
	else if (part == Segment_UpperArmR) {
		coord[0] = joints[Joint_shoulderR].getCoord().x;
		coord[1] = joints[Joint_shoulderR].getCoord().y;
		coord[2] = joints[Joint_shoulderR].getCoord().z;
	}
	else if (part == Segment_LowerArmR) {
		coord[0] = joints[Joint_elbowR].getCoord().x;
		coord[1] = joints[Joint_elbowR].getCoord().y;
		coord[2] = joints[Joint_elbowR].getCoord().z;
	}
	else if (part == Segment_HandR) {
		coord[0] = joints[Joint_wristR].getCoord().x;
		coord[1] = joints[Joint_wristR].getCoord().y;
		coord[2] = joints[Joint_wristR].getCoord().z;
	}
	else if (part == Segment_UpperArmL) {
		coord[0] = joints[Joint_shoulderL].getCoord().x;
		coord[1] = joints[Joint_shoulderL].getCoord().y;
		coord[2] = joints[Joint_shoulderL].getCoord().z;
	}
	else if (part == Segment_LowerArmL) {
		coord[0] = joints[Joint_elbowL].getCoord().x;
		coord[1] = joints[Joint_elbowL].getCoord().y;
		coord[2] = joints[Joint_elbowL].getCoord().z;
	}
	else if (part == Segment_HandL) {
		coord[0] = joints[Joint_wristL].getCoord().x;
		coord[1] = joints[Joint_wristL].getCoord().y;
		coord[2] = joints[Joint_wristL].getCoord().z;
	}
	else if (part == Segment_UpperLegR) {
		coord[0] = joints[Joint_highLegR].getCoord().x;
		coord[1] = joints[Joint_highLegR].getCoord().y;
		coord[2] = joints[Joint_highLegR].getCoord().z;
	}
	else if (part == Segment_LowerLegR) {
		coord[0] = joints[Joint_kneeR].getCoord().x;
		coord[1] = joints[Joint_kneeR].getCoord().y;
		coord[2] = joints[Joint_kneeR].getCoord().z;
	}
	else if (part == Segment_FootR) {
		coord[0] = joints[Joint_ankleR].getCoord().x;
		coord[1] = joints[Joint_ankleR].getCoord().y;
		coord[2] = joints[Joint_ankleR].getCoord().z;
	}
	else if (part == Segment_UpperLegL) {
		coord[0] = joints[Joint_highLegL].getCoord().x;
		coord[1] = joints[Joint_highLegL].getCoord().y;
		coord[2] = joints[Joint_highLegL].getCoord().z;
	}
	else if (part == Segment_LowerLegL) {
		coord[0] = joints[Joint_kneeL].getCoord().x;
		coord[1] = joints[Joint_kneeL].getCoord().y;
		coord[2] = joints[Joint_kneeL].getCoord().z;
	}
	else if (part == Segment_FootL) {
		coord[0] = joints[Joint_ankleL].getCoord().x;
		coord[1] = joints[Joint_ankleL].getCoord().y;
		coord[2] = joints[Joint_ankleL].getCoord().z;
	}
}

void getBodyPartDirection(int part, float* coord) {
	if (part == Segment_Head) {
		coord[0] = joints[Joint_neck].getCoord().x;
		coord[1] = joints[Joint_neck].getCoord().y;
		coord[2] = joints[Joint_neck].getCoord().z;
	}
	else if (part == Segment_Neck) {
		coord[0] = joints[Joint_shoulderMid].getCoord().x;
		coord[1] = joints[Joint_shoulderMid].getCoord().y;
		coord[2] = joints[Joint_shoulderMid].getCoord().z;
	}
	else if (part == Segment_UpperTorso) {
		coord[0] = joints[Joint_pelvisMid].getCoord().x;
		coord[1] = joints[Joint_pelvisMid].getCoord().y;
		coord[2] = joints[Joint_pelvisMid].getCoord().z;
	}
	else if (part == Segment_LowerTorso) {
		coord[0] = joints[Joint_pelvisMid].getCoord().x;
		coord[1] = (joints[Joint_pelvisMid].getCoord().y + joints[Joint_kneeR].getCoord().y) / 2;
		coord[2] = joints[Joint_pelvisMid].getCoord().z;
	}
	else if (part == Segment_UpperArmR) {
		coord[0] = joints[Joint_elbowR].getCoord().x;
		coord[1] = joints[Joint_elbowR].getCoord().y;
		coord[2] = joints[Joint_elbowR].getCoord().z;
	}
	else if (part == Segment_LowerArmR) {
		coord[0] = joints[Joint_wristR].getCoord().x;
		coord[1] = joints[Joint_wristR].getCoord().y;
		coord[2] = joints[Joint_wristR].getCoord().z;
	}
	else if (part == Segment_HandR) {
		coord[0] = vertices[rightMostIndex].x;
		coord[1] = vertices[rightMostIndex].y;
		coord[2] = vertices[rightMostIndex].z;
	}
	else if (part == Segment_UpperArmL) {
		coord[0] = joints[Joint_elbowL].getCoord().x;
		coord[1] = joints[Joint_elbowL].getCoord().y;
		coord[2] = joints[Joint_elbowL].getCoord().z;
	}
	else if (part == Segment_LowerArmL) {
		coord[0] = joints[Joint_wristL].getCoord().x;
		coord[1] = joints[Joint_wristL].getCoord().y;
		coord[2] = joints[Joint_wristL].getCoord().z;
	}
	else if (part == Segment_HandL) {
		coord[0] = vertices[leftMostIndex].x;
		coord[1] = vertices[leftMostIndex].y;
		coord[2] = vertices[leftMostIndex].z;
	}
	else if (part == Segment_UpperLegR) {
		coord[0] = joints[Joint_kneeR].getCoord().x;
		coord[1] = joints[Joint_kneeR].getCoord().y;
		coord[2] = joints[Joint_kneeR].getCoord().z;
	}
	else if (part == Segment_LowerLegR) {
		coord[0] = joints[Joint_ankleR].getCoord().x;
		coord[1] = joints[Joint_ankleR].getCoord().y;
		coord[2] = joints[Joint_ankleR].getCoord().z;
	}
	else if (part == Segment_FootR) {
		coord[0] = joints[Joint_ankleR].getCoord().x;
		coord[1] = vertices[bottomMostIndex].y;
		coord[2] = vertices[bottomMostIndex].z;
	}
	else if (part == Segment_UpperLegL) {
		coord[0] = joints[Joint_kneeL].getCoord().x;
		coord[1] = joints[Joint_kneeL].getCoord().y;
		coord[2] = joints[Joint_kneeL].getCoord().z;
	}
	else if (part == Segment_LowerLegL) {
		coord[0] = joints[Joint_ankleL].getCoord().x;
		coord[1] = joints[Joint_ankleL].getCoord().y;
		coord[2] = joints[Joint_ankleL].getCoord().z;
	}
	else if (part == Segment_FootL) {
		coord[0] = joints[Joint_ankleL].getCoord().x;
		coord[1] = vertices[bottomMostIndex].y;
		coord[2] = vertices[bottomMostIndex].z;
	}
}

int getBodyPartPointNum(int part) {
	return segmentHash[part].size();
}

void getBodyPartPointIndex(int part, int* nums) {
	int j = 0;
	for (int i = 0; i < segmentHash[part].size(); i++) {
		nums[j++] = segmentHash[part][i];
	}
}

void getBodyPartPointPos(int part, float* coords) {
	int j = 0;
	for (int i = 0; i < segmentHash[part].size(); i++) {
		coords[j++] = vertices[segmentHash[part][i]].x;
		coords[j++] = vertices[segmentHash[part][i]].y;
		coords[j++] = vertices[segmentHash[part][i]].z;
	}
}

void setHullPoint(int part, int pointNum, float* points) {
	for (int i = 0; i < pointNum * 3; i++) {
		segment3DConvex[part].push_back(points[i]);
	}
}

void setHullElem(int part, int elemNum, int* elements) {
	for (int i = 0; i < elemNum * 3; i++) {
		segment3DIndices[part].push_back(elements[i]);
	}
}

void getHullPoint(int part, float* points) {
	for (int i = 0; i < segment3DConvex[part].size(); i++) {
		points[i] = segment3DConvex[part][i];
	}
}

int getHullPointNum(int part) {
	return segment3DConvex[part].size()/3;
}

void getHullElem(int part, int* elements) {
	for (int i = 0; i < segment3DIndices[part].size(); i++) {
		elements[i] = segment3DIndices[part][i];
	}
}

int getHullElemNum(int part) {
	return segment3DIndices[part].size()/3;
}

void writeToOBJ(char* path) {
	ofstream outfile(path);

	for (int i = 0; i < vertices.size(); i++)
		outfile << "v " << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << endl;

	for (int i = 0; i < indices.size(); i += 3) {
		int i1 = indices[i];
		int i2 = indices[i + 1];
		int i3 = indices[i + 2];
		outfile << "f " << i1 << " " << i2 << " " << i3 << endl;
	}

	outfile.close();
}

void writeToHuman(char* path) {
	ofstream outfile(path);

	outfile << "### Default" << endl;
	for (int i = 0; i < origVertices.size(); i++) {
		outfile << origVertices[i].x << " " << origVertices[i].y << " " << origVertices[i].z << endl;
	}

	outfile << "\n### Human" << endl;
	for (int i = 0; i < vertices.size(); i++) {
		outfile << "v " << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << endl;
	}

	// Indices
	for (int i = 0; i < indices.size(); i += 3) {
		int i1 = indices[i];
		int i2 = indices[i + 1];
		int i3 = indices[i + 2];
		outfile << "f " << i1 << " " << i2 << " " << i3 << endl;
	}

	outfile << "\n### Joint" << endl;
	for (int i = 0; i < joints.size(); i++) {
		Vertex j = joints[i].getCoord();

		outfile << j.x << " " << j.y << " " << j.z << endl;
	}

	outfile << "\n### Landmark" << endl;
	for (int i = 0; i < landmarks.size(); i++) {
		Landmark l = landmarks[i];
		outfile << l.name << endl;
		outfile << l.type << endl;
		outfile << l.level << endl;
		outfile << l.value << endl;

		for (int j = 0; j < l.region.size(); j++) {
			outfile << l.region[j] << " ";
		}
		outfile << endl;

		for (int j = 0; j < l.vertIdx.size(); j++) {
			outfile << l.vertIdx[j] << " ";
		}
		outfile << endl << endl;
	}

	outfile << "\n### Segments" << endl;
	for (int i = 0; i < SegmentNum; i++) {
		for (int j = 0; j < segmentHash[i].size(); j++) {
			outfile << segmentHash[i][j] << " ";
		}
		outfile << "\n";
	}

	outfile << "\n### Weights" << endl;
	for (int i = 0; i < JointNum; i++) {
		for (int j = 0; j < weightHash[i].size(); j++) {
			outfile << weightHash[i][j] << " ";
		}
		outfile << "\n";
	}

	/*** Hard-code T pose for now ***/
	outfile << "\n### Tpose" << endl;
	outfile << "2 2 -20" << endl;
	outfile << "2 2 -8" << endl;
	outfile << "2 2 -11" << endl;
	outfile << "15 2 25" << endl;
	outfile << "15 2 15" << endl;
	outfile << "7 2 5" << endl;
	outfile << "11 2 -5" << endl;

	outfile.close();
}
