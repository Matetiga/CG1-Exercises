// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include <iostream>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include "Valence.h"


// Add the properties storing vertex valences to the given mesh
VertexValenceProperties AddValenceProperties (HEMesh &m)
{
	/* Task 1.2.3 */
	/* Add your properties to the mesh and return their handles. */
	VertexValenceProperties props;
	// Register two per-vertex properties: one for face-incidence valences and one for vertex-adjacency valences
	m.add_property(props.faceValences, "vertex_face_valence");
	m.add_property(props.vextexValences, "vertex_vertex_valence");

	return props;
}

// Compute a histogram for the given vertex valence property
ValenceHistogram ComputeValenceHistogram (const HEMesh &m, const VertexValenceProperty valence)
{
	ValenceHistogram ret;
	/* Task 1.2.3 - create a histogram of vertex valences from the values stored in your
	                custom mesh property. */

	if (!valence.is_valid())
	{
		std::cerr << "Property for valences is not valid." << std::endl;
		return ret;
	}

	for (const auto& v : m.vertices())
	{
		unsigned val = m.property(valence, v);
		++ret[val];
	}
	return ret;
}

// Computes the per-vertex face incidence count (aka. vertex face valences) for the given mesh,
// using the indicated property to store the results
void ComputeVertexFaceValences (HEMesh &m, const VertexValenceProperty valence)
{
	/* Task 1.2.3 - compute number of incident faces for each vertex using only simple-mesh
	                capabilities and store them in the given custom mesh property. */
	if (!valence.is_valid())
	{
		std::cerr << "Property for face valences is not valid." << std::endl;
		return;
	}

	// initialize to zero
	for (const auto& v : m.vertices())
		m.property(valence, v) = 0u;

	// for each face, increment the count for each of its vertices
	for (const auto& f : m.faces())
	{
		for (const auto& vh : m.fv_range(f))
		{
			m.property(valence, vh) += 1u;
		}
	}
}

// Computes the vertex valences for the given mesh, using the indicated property to store the results
void ComputeVertexVertexValences (HEMesh &m, const VertexValenceProperty valence)
{
	/* Task 1.2.3 - compute vertex valences using only simple-mesh capabilities and store them
	                in the given custom mesh property.
	   Hint 1: to replicate the ordered list of face vertices you have in a simple mesh data structure,
	           iterate through the edges of a face - for example by using the SmartFaceHandle::edges()
	           range object!
	   Hint 2: OpenMesh smart handles are automatically down-casted to ordinary handles, so you can
	           directly use the two above helper types SetOfVertices and VertexAdjacencyMap with smart
	           handles also, if you decide to use the smart handle APIs.*/
	
	if (!valence.is_valid())
	{
		std::cerr << "Property for vertex valences is not valid." << std::endl;
		return;
	}

	using SetOfVertexIds = std::unordered_set<int>;
	using VertexAdjacencyMap = std::unordered_map<int, SetOfVertexIds>;

	VertexAdjacencyMap adj;
	adj.reserve(m.n_vertices());

	// Build adjacency sets: for every face, add edges' endpoint adjacency
	for (const auto& f : m.faces())
	{
		// collect vertices of face in order
		std::vector<OpenMesh::VertexHandle> verts;
		for (const auto& vh : m.fv_range(f))
			verts.push_back(vh);

		const size_t n = verts.size();
		if (n < 2) continue;

		// For each edge of the face, add both directions to adjacency set
		for (size_t i = 0; i < n; ++i)
		{
			auto a = verts[i];
			auto b = verts[(i + 1) % n];
			adj[a.idx()].insert(b.idx());
			adj[b.idx()].insert(a.idx());
		}
	}

	// write valences (size of adjacency set) to property
	for (const auto& v : m.vertices())
	{
		auto it = adj.find(v.idx());
		unsigned val = (it == adj.end()) ? 0u : static_cast<unsigned>(it->second.size());
		m.property(valence, v) = val;
	}
}
