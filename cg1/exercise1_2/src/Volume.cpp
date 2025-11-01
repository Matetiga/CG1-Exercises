// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "Volume.h"

#include <iostream>
#include <vector>
#include <cmath>

float ComputeVolume(const HEMesh& m)
{
	float vol = 0.0f;
	/*Task 1.2.2*/

	for (const auto& f : m.faces())
	{
		std::vector<OpenMesh::Vec3f> pts;
		for (const auto& vh : m.fv_range(f))	
			pts.push_back(m.point(vh));

		if (pts.size() < 3) continue;

		const OpenMesh::Vec3f& p0 = pts[0];
		for (size_t i = 1; i + 1 < pts.size(); ++i)
		{
			const OpenMesh::Vec3f& p1 = pts[i];
			const OpenMesh::Vec3f& p2 = pts[i + 1];

			OpenMesh::Vec3f cr = OpenMesh::cross(p1, p2);
			float triple = p0[0] * cr[0] + p0[1] * cr[1] + p0[2] * cr[2];
			vol += triple * (1.0f / 6.0f);
		}
	}

	return std::abs(vol);
}