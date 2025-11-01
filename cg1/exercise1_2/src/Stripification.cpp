// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "Stripification.h"

#include <random>
#include "sample_set.h"




unsigned int ExtractTriStrips(HEMesh& mesh, OpenMesh::FPropHandleT<int> perFaceStripIdProperty, unsigned int nTrials)
{
	//prepare random engine
	std::mt19937 eng;

	//initialize strip index to -1 for each face
	for (auto f : mesh.faces())
		mesh.property(perFaceStripIdProperty, f) = -1;

	int nStrips = 0;

	/*Task 2.2.2*/
	
	return nStrips;
}