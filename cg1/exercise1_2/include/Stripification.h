// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once

#include "util/OpenMeshUtils.h"

//Extracts triangle strips from the mesh, writes the Strip Id of each face in 
//perFaceStripIdProperty, and returns the number of strips.
unsigned int ExtractTriStrips(HEMesh& m, OpenMesh::FPropHandleT<int> perFaceStripIdProperty, unsigned int nTrials);