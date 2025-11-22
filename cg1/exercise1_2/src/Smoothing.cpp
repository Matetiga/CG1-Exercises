// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include <memory>
#include <random>
#include "Smoothing.h"


void SmoothUniformLaplacian (HEMesh &m, float lambda)
{
	/*Task 2.2.3*/
	// Uniform discrete Laplacian smoothing
		// Uniform Laplacian means that the new position of a vertex 
		// is based on the difference between the current position and the average position of the neighbors
		// so the new point will be centered relative to its neighbors

	// 𝒑𝑖 ← 𝒑𝑖 + Δt ∙ 𝛼 ∙ Δs * p𝑖
		// angenommen Δt ∙ 𝛼 = lambda
		// and : Δs *p𝑖 = -2H n => (average - p)


	// IMPORTANT : The calculation of the average point has to be done with the OLD positions of the neighbors
	// otherwise mixed results
	std::map<OpenMesh::VertexHandle, OpenMesh::Vec3f> updated_pos;

	// the bigger lambda is, the more the point will move towards the average of its neighbors per iteration
	for(auto vertex : m.vertices()){
		OpenMesh::Vec3f average(0.f, 0.f, 0.f);
		unsigned int num_neightbors = 0;

		// iterate through neigbors 
		for (auto neighbor : m.vv_range(vertex)) {
			average += m.point(neighbor);
			num_neightbors += 1;
		}
		average /= (float) num_neightbors;
		updated_pos[vertex] = m.point(vertex) + lambda * (average - m.point(vertex));
	}

	// update all positions
	for (auto& [vertex, pos] : updated_pos) {
		m.point(vertex) = pos;
	}
}

void SmoothCotanLaplacian (HEMesh &m, float lambda)
{
	/*Task 2.2.3*/
	// Cotangent approximation parallel to surface normal 
	// no tangential smoothing : means the smoothing is done in the direction of the normal 
	// otherwise would be along the surface of the mesh
	// vertices wont drift along the surface and the overall shape is better preserved

	// Important : Obtuse anlges will be ingnored (90 < a < 180)
		// 	boundary conditions -> boundary vertices wont be updates

	//m.request_vertex_normals();
	std::map<std::pair<OpenMesh::SmartHalfedgeHandle, OpenMesh::SmartHalfedgeHandle>, float> cached_angles;
	std::map<OpenMesh::VertexHandle, OpenMesh::Vec3f> updated_pos;
	for (auto vertex : m.vertices()) {

		if (m.is_boundary(vertex)) {
			// same position 
			updated_pos[vertex] = m.point(vertex);
			std::cout << "boundary" << std::endl;
			continue;
		}

		// to calculate the Voronoi area
		float area = 0.f;

		// this will get the outgoing halfedges from vertex
		for (auto neighbor : m.voh_range(vertex)) {
			// this should get the two halfedges of the two triangles sharing the edge
			OpenMesh::SmartHalfedgeHandle neighbor1 = neighbor.opp().prev();
			OpenMesh::SmartHalfedgeHandle neighbor2 = neighbor.next();
			/*	if (!neighbor1.is_valid() || !neighbor2.is_valid()) {
					continue;
				}*/
			std::cout << "Neighbor1 Face: " << neighbor1.face().idx() << ", Neighbor2 Face: " << neighbor2.face().idx() << std::endl;

			// the angles are returned in radians
			// this should get the angle opposite of the edge of the triangle 1 
			auto alpha = m.calc_sector_angle(neighbor1);
			// angle of triangle 2
			auto beta = m.calc_sector_angle(neighbor2);

			std::cout << "Alpha: " << alpha << ", Beta: " << beta << std::endl;
			// skip obtuse angles (90 < a < 180)
			if (alpha > M_PI_2 || beta > M_PI_2) {
				continue;
			}

			// cot(x) = 1/ tan(x)
			// (cot(alpha) + cot(beta)) 
			float angle_sum = 1.f / (tan(alpha)) + 1.f / (tan(beta));
			cached_angles[{neighbor1, neighbor2}] = angle_sum;
			// this will get the lenght between vertex and neighbor and square it
			// (p_j - p_i)^2 
			//float point_diff =  pow(m.calc_edge_length(neighbor),2) ;
			//area += angle_sum * point_diff;
			area += angle_sum * pow(m.calc_edge_length(neighbor), 2) / 8.f;
		}
		//area *= 0.8f;

		// To calculate the Laplace Beltrami Operator
		// exact same code as before
		OpenMesh::Vec3f laplace_beltrami(0.f, 0.f, 0.f);
		for (auto neighbor : m.voh_range(vertex)) {
			// this can be faster if a map is implemented 
			OpenMesh::SmartHalfedgeHandle neighbor1 = neighbor.opp().prev();
			OpenMesh::SmartHalfedgeHandle neighbor2 = neighbor.next();
			float angle_sum;
			if (cached_angles.find({ neighbor1, neighbor2 }) != cached_angles.end()) {
				angle_sum = cached_angles[{neighbor1, neighbor2}];
			}
			else {
				// the angle was not cached -> obtuse angle, so skipp
				continue;
			}

			// m.oint(neighbor.to()) will get the vertex to which the halfedge is pointing
			auto neig_point = m.point(neighbor.to());
			auto vert_point = m.point(vertex);
			std::cout << "Neighbor Point: " << neig_point << ", Vertex Point: " << vert_point << std::endl;
			laplace_beltrami += angle_sum* (m.point(neighbor.to()) - m.point(vertex));
		}
		std::cout << "Current area" << area << std::endl;
		if (area > 0.1) {
			laplace_beltrami /= (2.f * area);
			updated_pos[vertex] = m.point(vertex) + lambda * laplace_beltrami;
		}
		// otherwise could cause problem with points going far beyond the edge (cylinder had this issue)
		else {
			updated_pos[vertex] = m.point(vertex);
		}
		// get the normal of the vertex
		//OpenMesh::Vec3f normal = m.normal(vertex);
		//float normal_component = laplace_beltrami.dot(normal);
		
		// updated pos 
		//updated_pos[vertex] = m.point(vertex) + lambda * normal_component * normal;

	}

	// update all positions
	for (auto& [vertex, pos] : updated_pos) {
		m.point(vertex) = pos;
	}


}


void AddNoise (HEMesh &m, OpenMesh::MPropHandleT<Viewer::BBoxType> bbox_prop)
{
	std::mt19937 rnd;
	std::normal_distribution<float> dist;

	for (auto v : m.vertices())
	{
		OpenMesh::Vec3f n;
		m.calc_vertex_normal_correct(v, n);
		const auto diag = m.property(bbox_prop).diagonal();
		const float base_diag = std::min(diag.x(), std::min(diag.y(), diag.z())) / 20.f;
		float base_nb=0, nb_num=0;
		for (auto vnb : m.vv_range(v))
		{
			base_nb += (m.point(v) - m.point(vnb)).norm();
			nb_num++;
		}
		base_nb /= 4.f * nb_num;

		m.point(v) += std::min(base_diag, base_nb) * dist(rnd) * n.normalized();
	}
}
