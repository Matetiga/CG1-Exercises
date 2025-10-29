// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once

#include "Viewer.h"
#include "util/OpenMeshUtils.h"


// Updates the vertex positions by Laplacian smoothing
void SmoothUniformLaplacian (HEMesh &m, float lambda);

// Updates the vertex positions by Laplacian smoothing with cotangent discretization
void SmoothCotanLaplacian (HEMesh &m, float lambda);

// Offsets the vertex positions with noise along the vertex normal
void AddNoise (HEMesh &m, OpenMesh::MPropHandleT<Viewer::BBoxType> bbox_prop);
