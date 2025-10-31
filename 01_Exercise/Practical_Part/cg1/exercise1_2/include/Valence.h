// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once

#include <map>
#include "util/OpenMeshUtils.h"

// The type used for storing the valence histogram
typedef std::map<unsigned, unsigned> ValenceHistogram;

// Container storing the custom mesh property handles required for valence computation
// Task 1.2.3: use an OpenMesh vertex property handle to store per-vertex valences
typedef OpenMesh::VPropHandleT<unsigned> VertexValenceProperty;

// Container for referencing the per-vertex valence attributes
struct VertexValenceProperties
{
	VertexValenceProperty faceValences, vextexValences;
};

// Add the properties storing vertex valences to the given mesh
VertexValenceProperties AddValenceProperties (HEMesh &m);

// Compute a histogram for the given vertex valence property
ValenceHistogram ComputeValenceHistogram (const HEMesh &m, const VertexValenceProperty valence);

// Computes the per-vertex face incidence count (aka. vertex face valences) for the given mesh,
// using the indicated property to store the results
void ComputeVertexFaceValences (HEMesh &m, const VertexValenceProperty valence);

// Computes the vertex valences for the given mesh, using the indicated property to store the results
void ComputeVertexVertexValences (HEMesh &m, const VertexValenceProperty valence);
