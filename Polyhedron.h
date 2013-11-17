#pragma once
#include <vector>
#include <glm/glm.hpp>

namespace GT1 {
	class Transform;
	class Face;
	class Polyhedron;

	class Edge {
	public:
		Edge();
		Edge(Transform* _transform, glm::vec3 _v0, glm::vec3 _v1) : transform(_transform), v0(_v0), v1(_v1) {}

		glm::vec3 Get() const { return v1 - v0; }
		Edge GetTransformedCopy() const;

		bool operator==(const Edge &other) const;
		glm::vec3 operator[](int index) const;

		glm::vec3 v0;
		glm::vec3 v1;
		Transform* transform;	
	};

	// Definition of a plane:
	// A normal and its distance from the origin
	struct Plane {
		glm::vec3 normal;
		float distance;
	};

	class Face {
	public:
		Face() {}
		Face(Transform* _transform) : transform(_transform) {}
		glm::vec3 GetVertex(int i) const { return vertices[i]; }
		glm::vec3 GetTransformedVertex(int i) const;
		// For rendering
		Edge GetEdge(int i) const { return edges[i]; }
		// For math
		Edge GetTransformedEdge(int i) const;
		glm::vec3 GetNormal() const { return normal; };
		glm::vec3 GetTransformedNormal() const;
		Face GetTransformedCopy() const;

		bool Contains(Edge edge) const;
		bool Contains(glm::vec3 vertex) const;
		void InsertEdge(Edge edge);
		int GetVertexCount() const { return vertices.size(); }
		int GetEdgeCount() const { return edges.size(); }
		
		Plane GetPlane() { return plane; }

		Transform* transform;
		Plane plane;

		glm::vec3 normal;
		// Edges are ordered in the way they were entered.
		std::vector<Edge> edges;
		// Vertices are ordered in the way they were entered.
		std::vector<glm::vec3> vertices;
	};

	class Polyhedron
	{
	public:
		Polyhedron(Transform* _transform) : transform(_transform) {}

		glm::vec3 GetVertex(int i) const { return vertices[i]; }
		glm::vec3 GetTransformedVertex(int i) const;
		Edge GetTransformedEdge(int i) const;
		Face GetFace(int i) const { return faces[i]; }
		glm::vec3 GetTransformedNormal(int i) const;
		Face GetTransformedFace(int i) const;
		glm::vec3 GetSupportPoint(const glm::vec3& dir) const;

		int GetVertexCount() const { return vertices.size(); }
		int GetEdgeCount() const { return edges.size(); }
		int GetFaceCount() const { return faces.size(); }	

		virtual void Render() = 0;

		Transform* transform;

		// Vertices are unordered
		std::vector<glm::vec3> vertices;
		// Edges are unordered
		std::vector<Edge> edges;
		// Faces are ordered in the way they were entered.
		std::vector<Face> faces;

	protected:
		void InsertFace(Face face);

	private:

	};

	class Cube : public Polyhedron
	{
	public:
		Cube(Transform* transform);
		void Render();

	private:

		// Rendering data
		unsigned int vertexbuffer;
		unsigned int colorbuffer;
	};
}
