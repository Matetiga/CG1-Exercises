// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once
#include "util/OpenMeshUtils.h"

//Finds the connected components in the mesh. Writes the non-negative shell index into perFaceShellIndex
//and returns the number of shells.
unsigned int ExtractShells (HEMesh &m, OpenMesh::FPropHandleT<int> perFaceShellIndex);
