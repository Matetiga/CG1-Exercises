// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "../include/Stripification.h"

#include <random>
#include <unordered_set>
#include "sample_set.h"


struct nav_pointer{
	OpenMesh::SmartHalfedgeHandle halfEdge;
	int p;

	nav_pointer forward() const{
		// Move to next halfEdge in Zig-State -. prev(inv(hi))
		if(p == 0){
			if(halfEdge.opp().prev().is_valid())
				return {halfEdge.opp().prev(), 1};
		}
		// Move to next halfEdge in Zag-State -. next(inv(hi))
		else{
			if(halfEdge.opp().next().is_valid())
				return {halfEdge.opp().next(), 0};
			
		}

		// ToDo: what should be returned if there is no valid next halfEdge?
		return {OpenMesh::SmartHalfedgeHandle(), -1};
	}
	nav_pointer backward() const{
		if (!halfEdge.is_valid() || !halfEdge.opp().is_valid())
			return {OpenMesh::SmartHalfedgeHandle(), -1};

		if(p == 0){
			if (halfEdge.prev().opp().is_valid())
				return {halfEdge.prev().opp(), 1};	
		}
		else{
			if (halfEdge.next().opp().is_valid())
				return {halfEdge.next().opp(), 0};
		}

		return {OpenMesh::SmartHalfedgeHandle(), -1};
	}
};


unsigned int ExtractTriStrips(HEMesh& mesh, OpenMesh::FPropHandleT<int> perFaceStripIdProperty, unsigned int nTrials)
{
	//prepare random number generator engine
	std::mt19937 eng;

	/*Task 2.2.2*/
	int nStrips = 0;

	// Strip is a suset of triangles in a traingle mesh
	// where each triangle shares an edge with the next triangle in the strip
	// Most algorithms create strips 12-18 traingles long 

	// Tessellation is covering a surface with geometric shapes (triangles here)

	//initialize strip index to -1 for each face
	//each iteration is for each triangle in a face (Cube composed of 12 triangles)


	// A triangle Strip has a Zig-Zag pattern
	// Variable p helps us to determine in which state we currently are 
	// This zig-zag pattern is formed by the alternating sequence of vertices in the strip
	// v0-------v2------v4
	//   \      |   \   |
	//      \   |     \ |
	//		   v1-------v3
	// Zig-State (v0, v1, v2)  -. second and last element go to next triangle
	// Zag-State (v2, v1, v3)  --. first and last element go to next triangle
	// Zig-State (v2, v3, v4)  --. ... 

	// availableTriangles will have all Indices for then to find a "seed" (which is a start with a random triangle)
	sample_set<int> availableTriangles;
	// TODO : check if it should be dubtracted by 1
	availableTriangles.reserve(mesh.n_faces()); // check <-----------------------------------------------------------------------

	for (auto f : mesh.faces()){
		mesh.property(perFaceStripIdProperty, f) = -1;
		availableTriangles.insert(f.idx());
	}

	 	// until all triangles are processed and assigned to a strip
	while(!availableTriangles.empty()){
		// this pointer should store the longest strip found in nTrials
		std::vector<int> longest_strip;

		for(int i = 0; i < nTrials; i++){
			int seed = availableTriangles.sample(eng);
			OpenMesh::FaceHandle seed_fh = mesh.face_handle(seed);

			// this will iterate through all halfedges of the seed face (3 in total pro triangle)
			for (auto seed_start_he : mesh.fh_range(seed_fh)){

				std::vector<int> current_best_strip;
				// change to other ZigZag state
				for(int n = 0; n < 2; n++){
					nav_pointer start_pointer = {make_smart(seed_start_he, &mesh), n};
	
					// find the halfedge handle of the seed 
					OpenMesh::HalfedgeHandle he  = mesh.halfedge_handle(mesh.face_handle(seed));
					start_pointer.halfEdge = make_smart(he, &mesh); // to transform it to SmartHalfedgeHandle and have movility
		

					std::unordered_set<int> faces_in_current_strip;

					nav_pointer current_pointer = start_pointer;
					while (true){
						nav_pointer temp_pointer = current_pointer.forward();
						
						std::cout << "First While Loop : Temp pointer " << temp_pointer.halfEdge.idx() << std::endl;	
						if(!temp_pointer.halfEdge.is_valid() || // invalid halfEdge
							mesh.property(perFaceStripIdProperty, temp_pointer.halfEdge.face()) != -1 || // already assigned to a strip
							faces_in_current_strip.count(temp_pointer.halfEdge.face().idx()) > 0){ // already in current strip (to avoid cycles)
 								break;
							}
	
						// add the triangle index to the strip
						faces_in_current_strip.insert(temp_pointer.halfEdge.face().idx());
						// only update current pointer if the next halfEdge is valid
						current_pointer = temp_pointer;
		
					}
		
					std::vector<int> backward_faces;
					// reset nav pointer
					current_pointer = start_pointer;
					while(true){
						nav_pointer temp_pointer = current_pointer.backward();
						std::cout << "Second While Loop : Temp pointer " << temp_pointer.halfEdge.idx() << std::endl;	

						if(!temp_pointer.halfEdge.is_valid() || // invalid halfEdge
							mesh.property(perFaceStripIdProperty, temp_pointer.halfEdge.face()) != -1 ||
							faces_in_current_strip.count(temp_pointer.halfEdge.face().idx()) > 0){
								break;
							}


						faces_in_current_strip.insert(current_pointer.halfEdge.face().idx());
						current_pointer = temp_pointer;
					}

					// check if this is the best seed so far (greedy choice)
					if(faces_in_current_strip.size() > longest_strip.size()){
						longest_strip.assign(faces_in_current_strip.begin(), faces_in_current_strip.end());
					}
				}
			}
		}
		if (!longest_strip.empty()) {
			for(auto f_idx : longest_strip){
				mesh.property(perFaceStripIdProperty, mesh.face_handle(f_idx)) = nStrips;
				availableTriangles.remove(f_idx);
			}
			nStrips++;
		} else if (!availableTriangles.empty()) {
            // Handle leftover isolated triangles that can't form strips
            int f_idx = availableTriangles.sample(eng);
            mesh.property(perFaceStripIdProperty, mesh.face_handle(f_idx)) = nStrips++;
            availableTriangles.remove(f_idx);
        }

		// for(auto f_idx : longest_strip){
		// 	// asign the strip id to the face (nStrips acts as id)
		// 	mesh.property(perFaceStripIdProperty, mesh.face_handle(f_idx)) = nStrips;
			
		// 	availableTriangles.remove(f_idx);
		// }
		
		// nStrips++;

	}


	return nStrips;
}