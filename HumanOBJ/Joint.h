#pragma once

#include "Vertex.h"
#include "Layer.h"

class Joint {
public:
	Joint();
	Joint(int, Vertex);
	~Joint();

	int id;
	Vertex coord;

	bool hasParent;
	bool hasChildren;

	void setCoord(Vertex&);
	void setCoord(float, float, float);
	Vertex& getCoord();

	void setParent(int);
	void setChild(int);
	void setChildren(int, int);
	void setChildren(int, int, int);
	int getParent();
	vector<int>& getChildren();

	void setLayer(Layer&);
	void setLayers(vector<Layer>&);
	Layer& getLayer(int);
	vector<Layer>& getLayers();

	void setRelatedSegment(int);
	vector<int>& getRelatedSegments();

	void setRelatedVerts(int, int);
	vector<int>& getRelatedVerts();

	int parent;
	vector<int> children;

	vector<Layer> layers;

	vector<int> relatedSegments; 
	vector<int> relatedVerts; 
};
