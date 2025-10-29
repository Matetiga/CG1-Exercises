// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once
#include "util/OpenMeshUtils.h"

//create a mesh containing a single quad
void CreateQuad(HEMesh& m);

//create a unit coube mesh 
void CreateCube(HEMesh& m);

//create a regular tetraeder mesh
void CreateTetrahedron(HEMesh& m, float a = 1);

//create a disc mesh
void CreateDisk(HEMesh& m, float radius, int slices);

//create a cylinder mesh 
void CreateCylinder(HEMesh& m, float radius, float height, int stacks, int slices);

//create a sphere mesh
void CreateSphere(HEMesh& m, float radius, int slices, int stacks);

//create a torus mesh
void CreateTorus(HEMesh& m, float r, float R, int nsides, int rings);

//creates an icosaeder mesh in m 
// radius is the radius of the circum sphere
void CreateIcosahedron(HEMesh& m, float radius);

//creates an octaeder mesh
// radius is the radius of the circum sphere
void CreateOctahedron(HEMesh& m, float radius);

//create a unit arrow
void CreateUnitArrow(HEMesh& m, float stem_radius = 0.04, float head_radius = 0.1, float stem_height = 0.8, int slices = 30, int stem_stacks = 1);
