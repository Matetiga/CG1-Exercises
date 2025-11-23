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
	std::map<OpenMesh::VertexHandle, OpenMesh::Vec3f> updated_pos;
	for (auto vertex : m.vertices()) {

		if (m.is_boundary(vertex)) {
			// same position 
			updated_pos[vertex] = m.point(vertex);
			continue;
		}

		// to calculate the Voronoi area
		float area = 0.f;
		OpenMesh::Vec3f laplace_beltrami(0.f, 0.f, 0.f);

		// this will get the outgoing halfedges from vertex
		for (auto neighbor : m.voh_range(vertex)) {

			// this will get the halfedge pointing to a vertex which is opposite to the edge 
			// opp().prev().prev() should be the same as opp().next()
			OpenMesh::SmartHalfedgeHandle neighbor1 = neighbor.opp().next();
			OpenMesh::SmartHalfedgeHandle neighbor2 = neighbor.next();

			// the angles are returned in radians
			// get the angles of the vertex opp to the edge 
			// this calculates the angle of the vertex to whicht the halfedge points
			auto alpha = m.calc_sector_angle(neighbor1);
			auto beta = m.calc_sector_angle(neighbor2);

			//std::cout << "Alpha: " << alpha << ", Beta: " << beta << std::endl;
			// skip obtuse angles (90 < a < 180)
			if (alpha > M_PI_2 || beta > M_PI_2) {
				continue;
			}

			// cot(x) = 1/ tan(x)
			float cot_alpha = 1.f / (tan(alpha));
			float cot_beta = 1.f / (tan(beta));
			std::cout << "Cot Alpha: " << cot_alpha << ", Cot Beta: " << cot_beta << std::endl;
			float angle_sum = cot_alpha + cot_beta;

			// (f_j - f_i)
			OpenMesh::Vec3f vector_difference = m.point(neighbor.to()) - m.point(vertex);
			laplace_beltrami += angle_sum * vector_difference;

			// || p_i - p_j ||^2
			float edge_length = m.calc_edge_length(neighbor);
			std::cout << "Edge Length: " << edge_length << std::endl;
			area += angle_sum * pow(edge_length, 2) / 8.f;
		}

		// TODO : Current Problem is for areas that are too small
		// for example for cylinder top and bottom faces, edges are streching beyond the edges
		// this means area gegen 0, so division by zero problem
		if (area < 0.1) {
			std::cout << "Small area detected: " << area << std::endl;
			area = 1.f;
		}
		std::cout << "Current area " << area << std::endl;
		if (area >= 1.f) {
			laplace_beltrami /= (2.f * area);
			updated_pos[vertex] = m.point(vertex) + lambda * laplace_beltrami;
		}
		// otherwise could cause problem with points going far beyond the edge (cylinder had this issue)
		else {
			updated_pos[vertex] = m.point(vertex);
		}
	}

	// update all positions
	for (auto& [vertex, pos] : updated_pos) {
		m.point(vertex) = pos;
	}

	std::cout << "End" << std::endl;
		

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
