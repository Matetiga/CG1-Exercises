// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include <map>
#include <queue>
#include "util/UnionFind.h"
#include "ShellExtraction.h"


unsigned int ExtractShells (HEMesh &m, OpenMesh::FPropHandleT<int> perFaceShellIndex)
{
	//reset the shell indices to -1 for every face
	for (auto f : m.faces())
		m.property(perFaceShellIndex, f) = -1;

	/*Task 2.2.1*/
	return 0;
}
