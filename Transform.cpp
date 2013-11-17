#include "Transform.h"
#include "RigidBody.h"
#include "Polyhedron.h"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

using namespace GT1;

Transform::Transform(glm::vec3 position, glm::vec3 orientation, glm::vec3 scale) {
	this->position = position;
	this->orientation = glm::quat(orientation);
	this->scale = scale;
	float mass = 20.0f;
	glm::mat3 inertiaTensor = glm::mat3(
		(1.0f/6.0f), 0.0f, 0.0f,
		0.0f, (1.0f/6.0f), 0.0f,
		0.0f, 0.0f, (1.0f/6.0f)
		) * mass;
	rigidBody = new RigidBody(this, mass, inertiaTensor);
	polyhedron = new Cube(this);
	scale = glm::vec3(1.0f);
	Recalculate();
}

void Transform::Tick(float dt) {
	if(rigidBody) {
		rigidBody->Tick(dt);
		Recalculate();

		polyhedron->Render();
	}
}

void Transform::Recalculate() {
	translationMatrix = glm::translate(glm::mat4(1.0f), position);
	rotationMatrix = glm::mat4_cast(orientation);
	scaleMatrix = glm::scale(glm::mat4(1.0f), scale);

	// Compute final matrix
	modelMatrix = translationMatrix * rotationMatrix * scaleMatrix;
}

glm::vec3 Transform::TransformPoint(const glm::vec3& vec) const {
	return glm::vec3(
		modelMatrix * glm::vec4(
		vec.x,
		vec.y,
		vec.z,
		1.0f
		)
	);
}

void Transform::TransformPoints(const std::vector<glm::vec3>& src, std::vector<glm::vec3>& dest) const {
	for(const glm::vec3& vec : src) {
		dest.push_back(TransformPoint(vec));
	}
}

glm::vec3 Transform::TransformDirection(const glm::vec3& dir) const {
	return glm::vec3(
		modelMatrix * glm::vec4(
		dir.x,
		dir.y,
		dir.z,
		0.0f
		)
	);
}

void Transform::TransformDirections(const std::vector<glm::vec3>& src, std::vector<glm::vec3>& dest) const {
	for(const glm::vec3& vec : src) {
		dest.push_back(TransformDirection(vec));
	}
}
