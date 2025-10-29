// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include <iostream>
#include <set>
#include <unordered_map>
#include <memory>
#include "Valence.h"


// Add the properties storing vertex valences to the given mesh
VertexValenceProperties AddValenceProperties (HEMesh &m)
{
	/* Task 1.2.3 */
	/* Add your properties to the mesh and return their handles. */
	std::cerr << "Adding the custom valence properties is not implemented." << std::endl;
	
	return VertexValenceProperties(); // <- return your added property handles inside a VertexValenceProperties container
}

// Compute a histogram for the given vertex valence property
ValenceHistogram ComputeValenceHistogram (const HEMesh &m, const VertexValenceProperty valence)
{
	ValenceHistogram ret;
	/* Task 1.2.3 - create a histogram of vertex valences from the values stored in your
	                custom mesh property. */
	return ret;
}

// Computes the per-vertex face incidence count (aka. vertex face valences) for the given mesh,
// using the indicated property to store the results
void ComputeVertexFaceValences (HEMesh &m, const VertexValenceProperty valence)
{
	/* Task 1.2.3 - compute number of incident faces for each vertex using only simple-mesh
	                capabilities and store them in the given custom mesh property. */
	std::cerr << "Face-based valence computation is not implemented." << std::endl;
}

// Computes the vertex valences for the given mesh, using the indicated property to store the results
void ComputeVertexVertexValences (HEMesh &m, const VertexValenceProperty valence)
{
	using SetOfVertices = std::set<OpenMesh::VertexHandle>;
	using VertexAdjacencyMap = std::unordered_map<OpenMesh::VertexHandle, SetOfVertices>;

	/* Task 1.2.3 - compute vertex valences using only simple-mesh capabilities and store them
	                in the given custom mesh property.
	   Hint 1: to replicate the ordered list of face vertices you have in a simple mesh data structure,
	           iterate through the edges of a face - for example by using the SmartFaceHandle::edges()
	           range object!
	   Hint 2: OpenMesh smart handles are automatically down-casted to ordinary handles, so you can
	           directly use the two above helper types SetOfVertices and VertexAdjacencyMap with smart
	           handles also, if you decide to use the smart handle APIs.*/
	
	std::cerr << "Vertex-based valence computation is not implemented." << std::endl;
}
