// This source code is property of the Computer Graphics and Visualization 
// chair of the TU Dresden. Do not distribute! 
// Copyright (C) CGV TU Dresden - All Rights Reserved

#pragma once

#include <gui/AbstractViewer.h>
#include <util/OpenMeshUtils.h>
#include <Valence.h>


class Viewer : public nse::gui::AbstractViewer
{
public:
	typedef nse::math::BoundingBox<float, 3> BBoxType;

	Viewer();

	void drawContents();

private:
	void SetupGUI();
	void MeshUpdated(bool initNewMesh = false);

	void ColorMeshFromIds();

	bool hasColors = false;

	nanogui::ComboBox* shadingBtn;
	unsigned int smoothingIterations;
	nanogui::Slider* sldSmoothingStrength;
	unsigned int stripificationTrials;

	HEMesh polymesh;
	MeshRenderer renderer;
	OpenMesh::MPropHandleT<BBoxType> bboxProperty;
	OpenMesh::MPropHandleT<float> avgEdgeLengthProperty;

	OpenMesh::FPropHandleT<int> faceIdProperty;
	OpenMesh::FPropHandleT<Eigen::Vector4f> faceColorProperty;

	VertexValenceProperty vertexFaceValenceProperty, vertexVertexValenceProperty;
};
