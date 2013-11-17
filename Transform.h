#pragma once
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

namespace GT1 {
	class RigidBody;
	class Mesh;
	class Polyhedron;

	// Contains a transformation matrix. Currently this is always a cube.
	class Transform
	{
	public:
		Transform(void);
		Transform(glm::vec3 position, glm::vec3 orientation, glm::vec3 scale);

		void Tick(float dt);
		void Recalculate();

		glm::vec3 position;
		glm::quat orientation;
		glm::vec3 scale;

		glm::mat4 GetMatrix() { return modelMatrix; }

		glm::vec3 TransformPoint(const glm::vec3& vec) const;
		void TransformPoints(const std::vector<glm::vec3>& src, std::vector<glm::vec3>& dest) const;
		glm::vec3 TransformDirection(const glm::vec3& dir) const;
		void TransformDirections(const std::vector<glm::vec3>& src, std::vector<glm::vec3>& dest) const;

		// Members
		RigidBody* rigidBody;
		Polyhedron* polyhedron;

	protected:
		// Matrices
		glm::mat4 translationMatrix;
		glm::mat4 rotationMatrix;
		glm::mat4 scaleMatrix;

		glm::mat4 modelMatrix;

		// Members

	};
}
