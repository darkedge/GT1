#include "Polyhedron.h"
#include "Transform.h"
#include <GL/glew.h>

using namespace GT1;

/************************************************************************/
/* Class Edge                                                           */
/************************************************************************/
bool Edge::operator==(const Edge &other) const {
	return
		this->v0 == other.v0 && this->v1 == other.v1
		||
		this->v0 == other.v1 && this->v1 == other.v0;
}

glm::vec3 Edge::operator[](int index) const {
	assert(index == 0 || index == 1);
	if(index == 0) return v0;
	return v1;
}

Edge Edge::GetTransformedCopy() const {
	return Edge(
		transform,
		transform->TransformPoint(v0),
		transform->TransformPoint(v1)
	);
}

/************************************************************************/
/* Class Face                                                           */
/************************************************************************/
void Face::InsertEdge(Edge edge) {
	edges.push_back(edge);
	if(std::find(vertices.begin(), vertices.end(), edge.v0) == vertices.end()) vertices.push_back(edge.v0);
	if(std::find(vertices.begin(), vertices.end(), edge.v1) == vertices.end()) vertices.push_back(edge.v1);
}

bool Face::Contains(Edge edge) const {
	return std::find(edges.begin(), edges.end(), edge) != edges.end();
}

glm::vec3 Face::GetTransformedVertex(int i) const {
	return transform->TransformPoint(vertices[i]);
}

bool Face::Contains(glm::vec3 vertex) const {
	return std::find(vertices.begin(), vertices.end(), vertex) != vertices.end();
}

Edge Face::GetTransformedEdge(int i) const {
	return edges[i].GetTransformedCopy();
}

Face Face::GetTransformedCopy() const {
	Face face(transform);
	face.normal = GetTransformedNormal();
	for(const glm::vec3& vertex : vertices)
		face.vertices.push_back(transform->TransformPoint(vertex));
	for(const Edge& edge : edges) {
		face.edges.push_back(edge.GetTransformedCopy());
	}

	// Compute plane
	face.plane.normal = face.normal;
	face.plane.distance = glm::dot(face.vertices[0], face.normal);

	return face;
}

glm::vec3 Face::GetTransformedNormal() const {
	return transform->TransformDirection(normal);
}

/************************************************************************/
/* Class Polyhedron                                                     */
/************************************************************************/

// Also calculates the face's normal and plane
void Polyhedron::InsertFace(Face face) {
	// Compute normal
	face.normal = glm::normalize(glm::cross(face.edges[0].Get(), face.edges[1].Get()));

	// Compute plane
	face.plane.normal = face.normal;
	face.plane.distance = glm::dot(face.vertices[0], face.normal);

	// Insert face
	faces.push_back(face);

	// Add unique edges and vertices to polyhedron
	for(Edge edge : face.edges) {
		if(std::find(edges.begin(), edges.end(), edge) == edges.end()) edges.push_back(edge);
		
		if(std::find(vertices.begin(), vertices.end(), edge.v0) == vertices.end()) vertices.push_back(edge.v0);
		if(std::find(vertices.begin(), vertices.end(), edge.v1) == vertices.end()) vertices.push_back(edge.v1);
	}
}

glm::vec3 Polyhedron::GetTransformedVertex(int i) const {
	return transform->TransformPoint(vertices[i]);
}

glm::vec3 Polyhedron::GetTransformedNormal(int i) const {
	return faces[i].GetTransformedNormal();
}

Edge Polyhedron::GetTransformedEdge(int i) const {
	return edges[i].GetTransformedCopy();
}

Face Polyhedron::GetTransformedFace(int i) const {
	return faces[i].GetTransformedCopy();
}

glm::vec3 Polyhedron::GetSupportPoint(const glm::vec3& dir) const {
	std::vector<glm::vec3> transformed;
	transform->TransformPoints(vertices, transformed);

	float bestProjection = -FLT_MAX;
	glm::vec3 bestVertex;

	for(const glm::vec3& v : transformed) {
		float projection = glm::dot(v, dir);

		if(projection > bestProjection) {
			bestVertex = v;
			bestProjection = projection;
		}
	}

	return bestVertex;
}

/************************************************************************/
/* Class Cube                                                           */
/************************************************************************/
Cube::Cube(Transform* transform) : Polyhedron(transform)
{
	/************************************************************************/
	/* Physics data                                                         */
	/************************************************************************/

	glm::vec3 v0(-0.5f,	0.5f, 0.5f);
	glm::vec3 v1(-0.5f,-0.5f, 0.5f);
	glm::vec3 v2(0.5f, -0.5f, 0.5f);
	glm::vec3 v3(0.5f, 0.5f, 0.5f);
	glm::vec3 v4(0.5f, 0.5f, -0.5f);
	glm::vec3 v5(0.5f, -0.5f, -0.5f);
	glm::vec3 v6(-0.5f, -0.5f, -0.5f);
	glm::vec3 v7(-0.5f, 0.5f, -0.5f);

	// Front
	Face face(transform);
	face.InsertEdge(Edge(transform, v0, v1));
	face.InsertEdge(Edge(transform, v1, v2));
	face.InsertEdge(Edge(transform, v2, v3));
	face.InsertEdge(Edge(transform, v3, v0));
	InsertFace(face);

	// Right
	face = Face(transform);
	face.InsertEdge(Edge(transform, v3, v2));
	face.InsertEdge(Edge(transform, v2, v5));
	face.InsertEdge(Edge(transform, v5, v4));
	face.InsertEdge(Edge(transform, v4, v3));
	InsertFace(face);

	// Back
	face = Face(transform);
	face.InsertEdge(Edge(transform, v4, v5));
	face.InsertEdge(Edge(transform, v5, v6));
	face.InsertEdge(Edge(transform, v6, v7));
	face.InsertEdge(Edge(transform, v7, v4));
	InsertFace(face);

	// Left
	face = Face(transform);
	face.InsertEdge(Edge(transform, v7, v6));
	face.InsertEdge(Edge(transform, v6, v1));
	face.InsertEdge(Edge(transform, v1, v0));
	face.InsertEdge(Edge(transform, v0, v7));
	InsertFace(face);

	// Up
	face = Face(transform);
	face.InsertEdge(Edge(transform, v7, v0));
	face.InsertEdge(Edge(transform, v0, v3));
	face.InsertEdge(Edge(transform, v3, v4));
	face.InsertEdge(Edge(transform, v4, v7));
	InsertFace(face);

	// Down
	face = Face(transform);
	face.InsertEdge(Edge(transform, v1, v6));
	face.InsertEdge(Edge(transform, v6, v5));
	face.InsertEdge(Edge(transform, v5, v2));
	face.InsertEdge(Edge(transform, v2, v1));
	InsertFace(face);

	assert(GetVertexCount() == 8);
	assert(GetEdgeCount() == 12);
	assert(GetFaceCount() == 6);

	/************************************************************************/
	/* Rendering data                                                       */
	/************************************************************************/

	GLfloat vertexData[] = { 
		0.5f, 0.5f, 0.5f,
		-0.5f, 0.5f, 0.5f,
		-0.5f,-0.5f, 0.5f,

		-0.5f,-0.5f, 0.5f,
		0.5f,-0.5f, 0.5f,
		0.5f, 0.5f, 0.5f,


		0.5f, 0.5f, 0.5f,
		0.5f,-0.5f, 0.5f,
		0.5f,-0.5f,-0.5f,

		0.5f,-0.5f,-0.5f,
		0.5f, 0.5f,-0.5f,
		0.5f, 0.5f, 0.5f,


		0.5f, 0.5f, 0.5f,
		0.5f, 0.5f,-0.5f,
		-0.5f, 0.5f,-0.5f,

		-0.5f, 0.5f,-0.5f,
		-0.5f, 0.5f, 0.5f,
		0.5f, 0.5f, 0.5f,


		-0.5f, 0.5f, 0.5f,
		-0.5f, 0.5f,-0.5f,
		-0.5f,-0.5f,-0.5f,

		-0.5f,-0.5f,-0.5f,
		-0.5f,-0.5f, 0.5f,
		-0.5f, 0.5f, 0.5f,


		-0.5f,-0.5f,-0.5f,
		0.5f,-0.5f,-0.5f,
		0.5f,-0.5f, 0.5f,

		0.5f,-0.5f, 0.5f,
		-0.5f,-0.5f, 0.5f,
		-0.5f,-0.5f,-0.5f,


		0.5f,-0.5f,-0.5f,
		-0.5f,-0.5f,-0.5f,
		-0.5f, 0.5f,-0.5f,

		-0.5f, 0.5f,-0.5f,
		0.5f, 0.5f,-0.5f,
		0.5f,-0.5f,-0.5f
	};

	float colorData[] = {
		1.0f,1.0f,1.0f,
		1.0f,1.0f,0.0f,
		1.0f,0.0f,0.0f,

		1.0f,0.0f,0.0f,
		1.0f,0.0f,1.0f,
		1.0f,1.0f,1.0f,


		1.0f,1.0f,1.0f,
		1.0f,0.0f,1.0f,
		0.0f,0.0f,1.0f,

		0.0f,0.0f,1.0f,
		0.0f,1.0f,1.0f,
		1.0f,1.0f,1.0f,


		1.0f,1.0f,1.0f,
		0.0f,1.0f,1.0f,
		0.0f,1.0f,0.0f,

		0.0f,1.0f,0.0f,
		1.0f,1.0f,0.0f,
		1.0f,1.0f,1.0f,


		1.0f,1.0f,0.0f,
		0.0f,1.0f,0.0f,
		0.0f,0.0f,0.0f,

		0.0f,0.0f,0.0f,
		1.0f,0.0f,0.0f,
		1.0f,1.0f,0.0f,


		0.0f,0.0f,0.0f,
		0.0f,0.0f,1.0f,
		1.0f,0.0f,1.0f,

		1.0f,0.0f,1.0f,
		1.0f,0.0f,0.0f,
		0.0f,0.0f,0.0f,


		0.0f,0.0f,1.0f,
		0.0f,0.0f,0.0f,
		0.0f,1.0f,0.0f,

		0.0f,1.0f,0.0f,
		0.0f,1.0f,1.0f,
		0.0f,0.0f,1.0f
	};

	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertexData), vertexData, GL_STATIC_DRAW);

	glGenBuffers(1, &colorbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(colorData), colorData, GL_STATIC_DRAW);
}

void Cube::Render() {
	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glVertexAttribPointer(
		0,			// attribute. No particular reason for 0, but must match the layout in the shader.
		3,			// size
		GL_FLOAT,	// type
		GL_FALSE,	// normalized?
		0,			// stride
		(void*)0	// array buffer offset
		);

	// 2nd attribute buffer : colors
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
	glVertexAttribPointer(
		1,			// attribute. No particular reason for 1, but must match the layout in the shader.
		3,			// size
		GL_FLOAT,	// type
		GL_FALSE,	// normalized?
		0,			// stride
		(void*)0	// array buffer offset
		);

	// Draw the triangle !
	glDrawArrays(GL_TRIANGLES, 0, 12*3); // 12*3 indices starting at 0 -> 12 triangles

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
}