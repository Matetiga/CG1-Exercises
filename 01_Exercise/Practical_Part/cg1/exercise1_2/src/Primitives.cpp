// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#include "Primitives.h"
#include "util/OpenMeshUtils.h"

void CreateQuad(HEMesh& mesh)
{
	mesh.clear();
	std::vector<OpenMesh::VertexHandle> vhandles;
	vhandles.push_back(mesh.add_vertex(OpenMesh::Vec3f(-0.5f, -0.5f, 0)));
	vhandles.push_back(mesh.add_vertex(OpenMesh::Vec3f( 0.5f, -0.5f, 0)));
	vhandles.push_back(mesh.add_vertex(OpenMesh::Vec3f( 0.5f,  0.5f, 0)));
	vhandles.push_back(mesh.add_vertex(OpenMesh::Vec3f(-0.5f,  0.5f, 0)));
	mesh.add_face(vhandles);	
}

void CreateCube(HEMesh& mesh)
{
	mesh.clear();
	/* Task 1.2.1 */
	std::vector<OpenMesh::VertexHandle> v(8);

	// front face (+z)
	v[0] = mesh.add_vertex(OpenMesh::Vec3f(-0.5f, -0.5f, 0.5f)); // 0: front bottom left
	v[1] = mesh.add_vertex(OpenMesh::Vec3f(0.5f, -0.5f, 0.5f)); // 1: front bottom right
	v[2] = mesh.add_vertex(OpenMesh::Vec3f(0.5f, 0.5f, 0.5f)); // 2: front top right
	v[3] = mesh.add_vertex(OpenMesh::Vec3f(-0.5f, 0.5f, 0.5f)); // 3: front top left

	// back face (-z)
	v[4] = mesh.add_vertex(OpenMesh::Vec3f(-0.5f, -0.5f, -0.5f)); // 4: back bottom left
	v[5] = mesh.add_vertex(OpenMesh::Vec3f(0.5f, -0.5f, -0.5f)); // 5: back bottom right
	v[6] = mesh.add_vertex(OpenMesh::Vec3f(0.5f, 0.5f, -0.5f)); // 6: back top right
	v[7] = mesh.add_vertex(OpenMesh::Vec3f(-0.5f, 0.5f, -0.5f)); // 7: back top left

	// faces (quads) - winding chosen to give outward normals
	mesh.add_face(v[0], v[1], v[2], v[3]); // front
	mesh.add_face(v[1], v[5], v[6], v[2]); // right
	mesh.add_face(v[5], v[4], v[7], v[6]); // back
	mesh.add_face(v[4], v[0], v[3], v[7]); // left
	mesh.add_face(v[3], v[2], v[6], v[7]); // top
	mesh.add_face(v[4], v[5], v[1], v[0]); // bottom
}

void CreateTetrahedron(HEMesh& mesh, float a)
{
	mesh.clear();

	std::vector<OpenMesh::VertexHandle> vhandles(4);

	vhandles[0] = mesh.add_vertex(OpenMesh::Vec3f(sqrt(3.0f)*a / 3.0f, 0, 0));
	vhandles[1] = mesh.add_vertex(OpenMesh::Vec3f(-sqrt(3.0f)*a / 6.0f, -a / 2.0f, 0));
	vhandles[2] = mesh.add_vertex(OpenMesh::Vec3f(-sqrt(3.0f)*a / 6.0f, a / 2.0f, 0));
	vhandles[3] = mesh.add_vertex(OpenMesh::Vec3f(0, 0, sqrt(6.0f)*a / 3.0f));

	mesh.add_face(vhandles[0], vhandles[1], vhandles[2]);
	mesh.add_face(vhandles[0], vhandles[2], vhandles[3]);
	mesh.add_face(vhandles[0], vhandles[3], vhandles[1]);
	mesh.add_face(vhandles[3], vhandles[2], vhandles[1]);
}

void CreateDisk(HEMesh& mesh, float radius, int slices)
{
	mesh.clear();

	std::vector<OpenMesh::VertexHandle> vhandles(slices + 1);
	vhandles[0] = mesh.add_vertex(OpenMesh::Vec3f(0, 0, 0));
	for (int i = 0; i < slices; i++)
	{
		float angle = -i * 2 * 3.14159f / slices;
		vhandles[i + 1] = mesh.add_vertex(OpenMesh::Vec3f(sin(angle)*radius, cos(angle)*radius, 0));
	}
	for (int i = 0; i < slices; i++)
		mesh.add_face(vhandles[0], vhandles[1 + i%slices], vhandles[1 + (1 + i) % slices]);
}

//create a cylinder mesh 
void CreateCylinder(HEMesh& mesh, float radius, float height, int stacks, int slices)
{
	assert(slices >= 3 && stacks >= 1);

	mesh.clear();

	int n = 2 + slices*(stacks + 1);
	std::vector<OpenMesh::VertexHandle> vhandles(n);

	vhandles[0] = mesh.add_vertex(OpenMesh::Vec3f(0.0f, height, 0.0f));

	int k = 1;
	for (int i = 0; i < stacks + 1; i++)
	{
		float h = (stacks - i)*height / (stacks);

		for (int j = 0; j < slices; j++)
		{
			float angle2 = j*2.0f*3.14159f / (float)(slices);
			vhandles[k] = mesh.add_vertex(OpenMesh::Vec3f(cos(angle2)*radius, h, sin(angle2)*radius));

			k++;
		}
	}

	vhandles[k] = mesh.add_vertex(OpenMesh::Vec3f(0.0f, 0.0f, 0.0f));

	for (int i = 0; i < slices; i++)
	{
		mesh.add_face(vhandles[0], vhandles[1 + (1 + i) % slices], vhandles[1 + i%slices]);

		for (int j = 0; j < stacks; j++)
		{
			int a, b, c, d;
			a = 1 + j*slices + (i) % slices;
			b = 1 + j*slices + (1 + i) % slices;
			c = 1 + (j + 1)*slices + (1 + i) % slices;
			d = 1 + (j + 1)*slices + (i) % slices;
			mesh.add_face(vhandles[a], vhandles[b], vhandles[c], vhandles[d]);
		}
		mesh.add_face(vhandles[vhandles.size() - 1],
			vhandles[1 + (stacks)*slices + (i) % slices],
			vhandles[1 + (stacks)*slices + (1 + i) % slices]);
	}
}

//create a sphere mesh
void CreateSphere(HEMesh& mesh, float radius, int slices, int stacks)
{
	assert(slices >= 3 && stacks >= 3);

	mesh.clear();

	int n = slices*(stacks - 1) + 2;
	std::vector<OpenMesh::VertexHandle> vhandles(n);

	vhandles[0] = mesh.add_vertex(OpenMesh::Vec3f(0.0f, radius, 0.0f));

	int k = 1;
	for (int i = 1; i < stacks; i++)
	{
		float angle1 = 3.14159f / 2.0f - i*3.14159f / (float)stacks;
		float r = cos(angle1)*radius;
		float height = sin(angle1)*radius;

		for (int j = 0; j < slices; j++)
		{
			float angle2 = j*2.0f*3.14159f / (float)(slices);
			vhandles[k] = mesh.add_vertex(OpenMesh::Vec3f(cos(angle2)*r, height, sin(angle2)*r));
			k++;
		}
	}

	vhandles[k] = mesh.add_vertex(OpenMesh::Vec3f(0.0f, -radius, 0.0f));

	for (int i = 0; i < slices; i++)
	{
		mesh.add_face(vhandles[0], vhandles[1 + (1 + i) % slices], vhandles[1 + i%slices]);

		for (int j = 0; j < stacks - 2; j++)
		{
			int a, b, c, d;
			a = 1 + j*slices + (i) % slices;
			b = 1 + j*slices + (1 + i) % slices;
			c = 1 + (j + 1)*slices + (1 + i) % slices;
			d = 1 + (j + 1)*slices + (i) % slices;
			mesh.add_face(vhandles[a], vhandles[b], vhandles[c], vhandles[d]);
		}
		mesh.add_face(vhandles[1 + slices*(stacks - 1)],
			vhandles[1 + (stacks - 2)*slices + (i) % slices],
			vhandles[1 + (stacks - 2)*slices + (1 + i) % slices]);
	}
}

//create a torus mesh
void CreateTorus(HEMesh& mesh, float r, float R, int nsides, int rings)
{
	assert(nsides >= 3 && rings >= 3);
	
	mesh.clear();

	int n = rings*nsides;
	std::vector<OpenMesh::VertexHandle> vhandles(n);
	int k = 0;
	for (int i = 0; i < rings; i++)
	{
		float angle1 = (float)(i*2.0*3.14159 / (rings));
		OpenMesh::Vec3f center(cos(angle1)*R, 0.0f, sin(angle1)*R);
		OpenMesh::Vec3f t1(cos(angle1), 0.0, sin(angle1));
		OpenMesh::Vec3f t2(0.0f, 1.0f, 0.0f);

		for (int j = 0; j < nsides; j++)
		{
			float angle2 = (float)(j*2.0*3.14159 / (nsides));
			vhandles[k] = mesh.add_vertex(center + (float)(sin(angle2)*r)*t1 + (float)(cos(angle2)*r)*t2);
			k++;
		}
	}

	for (int i = 0; i < rings; i++)
	{
		for (int j = 0; j < nsides; j++)
		{
			int a, b, c, d;
			a = (i + 1) % (rings)*(nsides)+j;
			b = (i + 1) % (rings)*(nsides)+(j + 1) % (nsides);
			c = i*(nsides)+(j + 1) % (nsides);
			d = i*(nsides)+j;
			mesh.add_face(
				vhandles[a], vhandles[b],
				vhandles[c], vhandles[d]);



		}
	}
}


//creates an icosahedron mesh in m 
// radius is the radius of the circum sphere
void CreateIcosahedron(HEMesh& mesh, float radius)
{
	mesh.clear();

	float a = (float)(radius*4.0 / sqrt(10.0 + 2.0*sqrt(5.0)));
	float h = (float)cos(2.0*asin(a / (2.0*radius)))*radius;
	float r2 = (float)sqrt(radius*radius - h*h);

	std::vector<OpenMesh::VertexHandle> vhandles(12);
	int k = 0;
	vhandles[k++] = mesh.add_vertex(OpenMesh::Vec3f(0, radius, 0));

	for (int i = 0; i < 5; i++)
		vhandles[k++] = mesh.add_vertex(OpenMesh::Vec3f((float)cos(i*72.0*3.14159 / 180.0)*r2, h, -(float)sin(i*72.0*3.14159 / 180.0)*r2));

	for (int i = 0; i < 5; i++)
		vhandles[k++] = mesh.add_vertex(OpenMesh::Vec3f((float)cos(36.0*3.14159 / 180.0 + i*72.0*3.14159 / 180.0)*r2, -h, -(float)sin(36.0*3.14159 / 180.0 + i*72.0*3.14159 / 180.0)*r2));

	vhandles[k] = mesh.add_vertex(OpenMesh::Vec3f(0, -radius, 0));

	for (int i = 0; i < 5; i++)
	{
		mesh.add_face(vhandles[0], vhandles[i + 1], vhandles[(i + 1) % 5 + 1]);
		mesh.add_face(vhandles[11], vhandles[(i + 1) % 5 + 6], vhandles[i + 6]);
		mesh.add_face(vhandles[i + 1], vhandles[i + 6], vhandles[(i + 1) % 5 + 1]);
		mesh.add_face(vhandles[(i + 1) % 5 + 1], vhandles[i + 6], vhandles[(i + 1) % 5 + 6]);
	}
}


//creates an octaeder mesh
// radius is the radius of the circum sphere
void CreateOctahedron(HEMesh& mesh, float radius)
{
	mesh.clear();

	std::vector<OpenMesh::VertexHandle> vhandles(6);
	int k = 0;
	vhandles[k++] = mesh.add_vertex(OpenMesh::Vec3f(0, radius, 0));


	for (int i = 0; i < 4; i++)
		vhandles[k++] = mesh.add_vertex(OpenMesh::Vec3f((float)cos(i*3.14159 / 2.0)*radius, 0, -(float)sin(i*3.14159 / 2.0)*radius));


	vhandles[k++] = mesh.add_vertex(OpenMesh::Vec3f(0, -radius, 0));



	for (int i = 0; i < 4; i++)
	{
		mesh.add_face(vhandles[0], vhandles[i + 1], vhandles[(i + 1) % 4 + 1]);
		mesh.add_face(vhandles[5], vhandles[(i + 1) % 4 + 1], vhandles[i + 1]);
	}
}

//create a unit arrow
void CreateUnitArrow(HEMesh& mesh, float stem_radius, float head_radius, float stem_height, int slices, int stem_stacks)
{
	assert(slices >= 3 && stem_stacks >= 1 && stem_height <= 1 && stem_height >= 0);

	float head_height = (float)1 - stem_height;
	mesh.clear();

	int n = 2 + slices*(stem_stacks + 2);
	std::vector<OpenMesh::VertexHandle> vhandles(n);

	float height = stem_height + head_height;
	vhandles[0] = mesh.add_vertex(OpenMesh::Vec3f(0.0f, height, 0.0f));

	int k = 1;
	for (int j = 0; j < slices; j++)
	{
		float angle2 = j*2.0f*3.14159f / (float)(slices);
		vhandles[k] = mesh.add_vertex(OpenMesh::Vec3f(cos(angle2)*head_radius, stem_height, sin(angle2)*head_radius));
		k++;
	}

	for (int i = 0; i < stem_stacks + 1; i++)
	{
		float h = (stem_stacks - i)*stem_height / (stem_stacks);

		for (int j = 0; j < slices; j++)
		{
			float angle2 = j*2.0f*3.14159f / (float)(slices);
			vhandles[k] = mesh.add_vertex(OpenMesh::Vec3f(cos(angle2)*stem_radius, h, sin(angle2)*stem_radius));
			k++;
		}
	}
	vhandles[k] = mesh.add_vertex(OpenMesh::Vec3f(0.0f, 0.0f, 0.0f));

	for (int i = 0; i < slices; i++)
	{
		mesh.add_face(vhandles[0], vhandles[1 + (1 + i) % slices], vhandles[1 + i%slices]);

		for (int j = 0; j < stem_stacks + 1; j++)
		{
			int a, b, c, d;
			a = 1 + j*slices + (i) % slices;
			b = 1 + j*slices + (1 + i) % slices;
			c = 1 + (j + 1)*slices + (1 + i) % slices;
			d = 1 + (j + 1)*slices + (i) % slices;
			mesh.add_face(vhandles[a], vhandles[b], vhandles[c], vhandles[d]);
		}
		mesh.add_face(vhandles[vhandles.size() - 1],
			vhandles[1 + (stem_stacks + 1)*slices + (i) % slices],
			vhandles[1 + (stem_stacks + 1)*slices + (1 + i) % slices]);
	}
}
