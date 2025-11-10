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
	// Union Find Data Structure checks for disjunct sets (useful to identify shells-> each bone that compose the feet mesh)
	// Identifies if two faces are part of the same set by iterating over the edges
	nse::util::UnionFind relations;
	relations.AddItems(m.n_faces());

	unsigned int shell_id = 0;

	// Iterate over all edges and find their adjacent faces
	// Better than Halfedge iteration because each edge is only visited once?
	for(auto edge : m.edges()){
		// why does it have to be 0? -> so is on the documentation
		// add adjacent faces to relation 
		auto halfEdge = m.halfedge_handle(edge, 0);
		OpenMesh::FaceHandle adj_face = m.face_handle(halfEdge);
		OpenMesh::FaceHandle opp_face = m.face_handle(m.opposite_halfedge_handle(halfEdge));

		// check to see if faces are not buondaries (more like edge case) 
		if(adj_face.is_valid() && opp_face.is_valid()){
			// there is now a relation between the two faces 
			// this will return the same index_t for all faces in the same shell
			relations.Merge(adj_face.idx(), opp_face.idx());
		}


	}

	// this will map the index (representative) to all faces that belong to that shell
	std::map<nse::util::UnionFind::index_t, std::vector<OpenMesh::FaceHandle>> face_representatives;
	for(auto f : m.faces()){
		// representative is the root of the set	
		nse::util::UnionFind::index_t representative = relations.GetRepresentative(f.idx());
		face_representatives[representative].push_back(f);
	}

	// asing shell_id according to representatives in map
	for(const auto& entry : face_representatives){
		// for each face inside the vector
		for(const auto& face : entry.second){
			m.property(perFaceShellIndex, face) = shell_id;
		}
		shell_id++;
	}
	return shell_id;
}
