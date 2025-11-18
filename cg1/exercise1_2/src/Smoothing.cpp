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
