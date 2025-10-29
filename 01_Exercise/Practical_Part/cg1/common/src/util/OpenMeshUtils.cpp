#include "util/OpenMeshUtils.h"

#include <vector>
#include <gui/ShaderPool.h>

MeshRenderer::MeshRenderer(const HEMesh& mesh)
	: mesh(mesh), indexCount(0),
	positionBuffer(nse::gui::VertexBuffer), normalBuffer(nse::gui::VertexBuffer), colorBuffer(nse::gui::VertexBuffer), texCoordBuffer(nse::gui::VertexBuffer),
	indexBuffer(nse::gui::IndexBuffer),
	texCoordBuffer4D(nse::gui::VertexBuffer), indexBufferTexCoords(nse::gui::IndexBuffer)
{ 
	vao.generate();	
	vaoTexCoords.generate();

	Update();
}

//EmitVertexFunctor: void(const HEMesh::HalfedgeHandle[3]) //the to-vertices of the halfedges are the triangle corners
template <typename EmitTriangleFunctor>
void TriangulateMeshFace(HEMesh::FaceHandle f, const HEMesh& mesh, EmitTriangleFunctor&& emitTriangle)
{
	OpenMesh::HalfedgeHandle base;
	for (auto h : mesh.fh_range(f))
	{
		if (base.idx() == -1)
		{
			base = h;
			continue;
		}
		auto nextH = mesh.next_halfedge_handle(h);
		if (nextH == base)
			break;
		else
		{
			HEMesh::HalfedgeHandle triangle[3] = { base, h, nextH };
			std::forward<EmitTriangleFunctor>(emitTriangle)(triangle);
		}
	}
}

void MeshRenderer::Update()
{
	if (mesh.n_vertices() == 0)
		return;

	ShaderPool::Instance()->meshShader.bind();
	vao.bind();

	std::vector<Eigen::Vector4f> positions;
	std::vector<Eigen::Vector4f> normals;
	std::vector<Eigen::Vector2f> uvs;
	positions.reserve(mesh.n_vertices());
	normals.reserve(mesh.n_vertices());
	if(mesh.has_vertex_texcoords2D())
		uvs.reserve(mesh.n_vertices());
	for (auto v : mesh.vertices())
	{
		positions.push_back(ToEigenVector4(mesh.point(v)));
		OpenMesh::Vec3f n;
		mesh.calc_vertex_normal_correct(v, n);
		normals.push_back(ToEigenVector4(n, 0));
		if (mesh.has_vertex_texcoords2D())
			uvs.push_back(ToEigenVector(mesh.texcoord2D(v)));
	}
	positionBuffer.uploadData(positions).bindToAttribute("position");
	normalBuffer.uploadData(normals).bindToAttribute("normal");
	if (mesh.has_vertex_texcoords2D())
		texCoordBuffer.uploadData(uvs).bindToAttribute("texCoords");

	std::vector<uint32_t> indices;
	indices.reserve(mesh.n_faces() * 3);
	for (auto f : mesh.faces())
	{
		TriangulateMeshFace(f, mesh, [&](const HEMesh::HalfedgeHandle h[3])
		{
			indices.push_back(mesh.to_vertex_handle(h[0]).idx());
			indices.push_back(mesh.to_vertex_handle(h[1]).idx());
			indices.push_back(mesh.to_vertex_handle(h[2]).idx());
		});
	}
	indexBuffer.uploadData(sizeof(uint32_t) * (uint32_t)indices.size(), indices.data());
	indexCount = (unsigned int)indices.size();

	vao.unbind();

	hasColor = false;

	UpdateTextureMapBuffers();
}

void MeshRenderer::UpdateWithPerFaceColor(OpenMesh::FPropHandleT<Eigen::Vector4f> colorProperty)
{
	if (mesh.n_vertices() == 0)
		return;

	ShaderPool::Instance()->meshShader.bind();
	vao.bind();

	std::vector<Eigen::Vector4f> positions;
	std::vector<Eigen::Vector4f> normals;
	std::vector<Eigen::Vector4f> colors;
	std::vector<Eigen::Vector2f> uvs;
	positions.reserve(3 * mesh.n_faces());
	normals.reserve(3 * mesh.n_faces());
	colors.reserve(3 * mesh.n_faces());
	if (mesh.has_vertex_texcoords2D())
		uvs.reserve(3 * mesh.n_faces());

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);	

	for (auto f : mesh.faces())
	{
		TriangulateMeshFace(f, mesh, [&](const HEMesh::HalfedgeHandle h[3])
		{
			for (int i = 0; i < 3; ++i)
			{
				auto v = mesh.to_vertex_handle(h[i]);
				positions.push_back(ToEigenVector4(mesh.point(v)));
				OpenMesh::Vec3f n;
				mesh.calc_vertex_normal_correct(v, n);
				normals.push_back(ToEigenVector4(n, 0));
				colors.push_back(mesh.property(colorProperty, f));
				if (mesh.has_vertex_texcoords2D())
					uvs.push_back(ToEigenVector(mesh.texcoord2D(v)));
			}			
		});
	}
	positionBuffer.uploadData(positions).bindToAttribute("position");
	normalBuffer.uploadData(normals).bindToAttribute("normal");
	colorBuffer.uploadData(colors).bindToAttribute("color");
	if (mesh.has_vertex_texcoords2D())
		texCoordBuffer.uploadData(uvs).bindToAttribute("texCoords");

	indexCount = (unsigned int)positions.size();

	vao.unbind();

	hasColor = true;

	UpdateTextureMapBuffers();
}

void MeshRenderer::UpdateTextureMapBuffers()
{
	if (!mesh.has_vertex_texcoords2D())
		return;

	ShaderPool::Instance()->simpleShader.bind();
	vaoTexCoords.bind();

	std::vector<Eigen::Vector4f> positions;	
	positions.reserve(mesh.n_vertices());
	
	for (auto v : mesh.vertices())
		positions.push_back(ToEigenVector4(mesh.texcoord2D(v)));		

	texCoordBuffer4D.uploadData(positions).bindToAttribute("position");

	std::vector<uint32_t> indices;
	indices.reserve(mesh.n_edges() * 2);
	for (auto e : mesh.edges())
	{
		auto h = mesh.halfedge_handle(e, 0);
		indices.push_back(mesh.from_vertex_handle(h).idx());
		indices.push_back(mesh.to_vertex_handle(h).idx());
	}
	indexBufferTexCoords.uploadData(sizeof(uint32_t) * (uint32_t)indices.size(), indices.data());
	indexCountTexCoords = (unsigned int)indices.size();

	vaoTexCoords.unbind();
}

void MeshRenderer::Render(const Eigen::Matrix4f& view, const Eigen::Matrix4f& projection, bool flatShading, bool withTexCoords, const Eigen::Vector4f& color) const
{
	if (indexCount == 0)
		return;

	auto& shader = ShaderPool::Instance()->meshShader;
	shader.bind();
	shader.setUniform("view", view);
	shader.setUniform("proj", projection);
	shader.setUniform("flatShading", flatShading ? 1 : 0);
	shader.setUniform("perVertexColor", hasColor ? 1 : 0);
	shader.setUniform("visualizeTexCoords", withTexCoords ? 1 : 0);
	shader.setUniform("color", color);

	vao.bind();
	if (hasColor)
		glDrawArrays(GL_TRIANGLES, 0, indexCount);
	else
		glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
	vao.unbind();
}

void MeshRenderer::RenderTextureMap(const Eigen::Matrix4f& projection, const Eigen::Vector4f& color) const
{
	if (indexCountTexCoords == 0)
		return;

	auto& shader = ShaderPool::Instance()->simpleShader;
	shader.bind();
	shader.setUniform("mvp", projection);
	shader.setUniform("color", color);

	vaoTexCoords.bind();
	glDrawElements(GL_LINES, indexCountTexCoords, GL_UNSIGNED_INT, 0);
	vaoTexCoords.unbind();
}