#pragma once

#include <vector>
#include "Layer.h"

class LayerTree {
public :
	LayerTree() {
		size = 0;
	}
	LayerTree(int idx, Layer l) {
		size = 1;
		//layerIndices.push_back(idx);
		layerTree.push_back(l);
	}

	~LayerTree() {

	}
	
	void append(int idx, Layer& l) {
		//layerIndices.push_back(idx);
		layerTree.push_back(l);
		size += 1;
	}

	/*
	void appendIndex(int idx) {
		layerIndices.push_back(idx);
	}
	*/

	void appendLayer(Layer& l) {
		layerTree.push_back(l);
		size++;
	}

	void insert(int loc, Layer l) {
		layerTree.insert(layerTree.begin() + loc, l);
		size++;
	}

	int getSize() { return size; }

	Layer& at(int idx) { 
		return layerTree[idx]; 
	}

	int getLayerIdx(Layer l) {
		for (int i = 0; i < layerTree.size(); i++) {
			if (l == layerTree[i])
				return i;
		}
		return -1;
	}
	
	//vector<int> getAllIndices() { return layerIndices; }
	vector<Layer> getAllLayers() { return layerTree; }
private :
	int size;
	//vector<int> layerIndices;
	vector<Layer> layerTree;
};
