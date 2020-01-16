#include "stdafx.h"
#include "GrahamScan.h"

GrahamScan::GrahamScan() {
	P = Vertex();
}

GrahamScan::GrahamScan(vector<Vertex> v) {
	P = Vertex();
	vector<Vertex> t(v);
	verts = t;
}

GrahamScan::~GrahamScan() {

}

vector<Vertex> GrahamScan::GenerateConvexHull() {
	float maxX = 0;
	float minX = 0;
	float minZ = 0;

	// 1. Find point with the loweset z-coord; if more than one, choose one with the lowest x val.
	for ( vector<Vertex>::iterator v = verts.begin(); v != verts.end(); v++) {
		if ((*v).x > maxX)
			maxX = (*v).x;
		if ((*v).x < minX)
			minX = (*v).x;

		// When more than one minZ, choose one with the lowest x.
		if ((*v).z == minZ) {
			if (!P.isNull() && (*v).x >= P.x) {  // P stays
				newVerts.push_back(*v);
				continue;
			}
			else { // P changes
				if (!P.isNull())
					newVerts.push_back(P); // Old P to newVerts

				// update P
				P = (*v);
				P.set();
			}
		}
		else if ((*v).z < minZ) {
			minZ = (*v).z;
			if (!P.isNull()) {
				newVerts.push_back(P); // Old P to newVerts
			}
			// update P
			P = (*v);
			P.set();
		}
		else {
			newVerts.push_back(*v);
			//P = (*v);
			//P.set();
		}
			
	}

	// 2. Sort points in increasing order of the angle they and P make with the x-axis.
	vector<Vertex> tmpVerts = mergeSort(newVerts, P);

	// split positivie and negative slopes
	vector<Vertex> pslopes;
	vector<Vertex> nslopes;

	for (vector<Vertex>::iterator v = tmpVerts.begin(); v != tmpVerts.end(); v++) {
		if (slope(*v, P) < 0) {
			nslopes.push_back(*v);
		}
		else if (slope(*v, P) > 0) {
			pslopes.push_back(*v);
		}
	}

	// align verts with slopes
	vector<Vertex> slopes;
	slopes.reserve(pslopes.size() + nslopes.size());
	slopes.insert(slopes.begin(), pslopes.begin(), pslopes.end());
	slopes.insert(slopes.end(), nslopes.begin(), nslopes.end());
	vector<Vertex> sverts(slopes);

	// 3. Generate ConvexHull
	vector<Vertex> convexHull;
	convexHull.push_back(P);
	convexHull.push_back(sverts[0]);

	int w = 10;
	int l = (sverts.size() < w)? sverts.size() : w;

	for (int i = 1; i < sverts.size(); i++) {
		int idx = convexHull.size();
		Vertex last(convexHull[idx - 1]);
		Vertex secondToLast(convexHull[idx - 2]);

		Vertex v1(last.subtract(secondToLast));
		Vertex v2(sverts[i].subtract(secondToLast));

		while (convexHull.size() > 2 && isLeftTurn(v1, v2)) {
			convexHull.pop_back();
			idx = convexHull.size();
			last = convexHull[idx - 1];
			secondToLast = convexHull[idx - 2];
			v1 = last.subtract(secondToLast);
			v2 = sverts[i].subtract(secondToLast);
		}
		convexHull.push_back(sverts[i]);
	}

	convexHull.push_back(P);

	return convexHull;
}

bool GrahamScan::isLeftTurn(Vertex v1, Vertex v2) {
	float z = v1.z*v2.x - v1.x*v2.z;
	if (z >= 0) {
		return true;
	}
	else
		return false;
}

vector<Vertex> GrahamScan::mergeSort(vector<Vertex> v, Vertex p){
	return mSort(v, p);
}

vector<Vertex> GrahamScan::mSort(vector<Vertex> v, Vertex p) {
	vector<Vertex> l;
	vector<Vertex> r;

	// sorted
	if (v.size() < 2) {
		return v;
	}

	for (int i = 0; i < v.size(); i++) {
		if (i < v.size() / 2) {
			l.push_back(v[i]);
		}
		else {
			r.push_back(v[i]);
		}
	}

	// sort left
	l = mSort(l, p);
	// sort right
	r = mSort(r, p);

	// merge left + right
	return merge(l, r, p);
}

vector<Vertex> GrahamScan::merge(vector<Vertex> v1, vector<Vertex> v2, Vertex p) {
	vector<Vertex> result;
	while (v1.size() != 0 && v2.size() != 0) {
		if (slope(v1[0], p) < slope(v2[0], p)) {
			result.push_back(v1[0]);
			v1.erase(v1.begin());
		}
		else {
			result.push_back(v2[0]);
			v2.erase(v2.begin());
		}
	}

	// If previous while-loop terminated with v2.size = 0
	while (v1.size() != 0) {
		result.push_back(v1[0]);
		v1.erase(v1.begin());
	}

	// If previous while-loop terminated with v1.size = 0 
	while (v2.size() != 0) {
		result.push_back(v2[0]);
		v2.erase(v2.begin());
	}

	return result;
}

float GrahamScan::slope(Vertex v1, Vertex v2) {
	float s = 0;

	if (v1.x != v2.x) {
		s = (v2.z - v1.z) / (v2.x - v1.x);
	}

	return s;
}