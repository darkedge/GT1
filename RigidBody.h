#pragma once
#include <glm/glm.hpp>
#include <unordered_map>

namespace GT1 {
	class Transform;

	class RigidBody {
	public:
		RigidBody() {}
		RigidBody(Transform* transform, float mass, glm::mat3 inertiaTensor);

		void Tick(float dt);
		unsigned char AddThruster(glm::vec3 position, glm::vec3 magnitude);
		void RemoveThruster(unsigned char);
		void Impulse(const glm::vec3& position, const glm::vec3& magnitude);
		void AddMomentum(const glm::vec3& linear, const glm::vec3& angular);

		glm::vec3 GetVelocity() const { return state.v; }
		glm::vec3 GetAngularVelocity() const { return state.ω; }
		float GetInvertedMass() const { return state.m⁻¹; }
		glm::mat3 GetInvertedInertiaTensor() const { return state.I⁻¹; }
		glm::vec3 NetForce() const;
		glm::vec3 NetTorque() const;
		void SetInvertedMass(float mass) { state.m⁻¹ = mass; }
		void SetInvertedInertiaTensor(const glm::mat3& tensor) { state.I⁻¹ = tensor; }

		Transform* transform;

	protected:
		struct State {
			State() {}
			State(float mass, glm::mat3 inertiaTensor);

			// Constants
			float m⁻¹;		// Inverted mass
			glm::mat3 I⁻¹;	// Inertia tensor inverse

			glm::vec3 p;	// Linear momentum
			glm::vec3 L;	// Angular momentum

			glm::vec3 v;	// Linear velocity (Read-only)
			glm::vec3 ω;	// Angular velocity (Read-only)
		};


		struct Derivative {
			glm::vec3 F; // Force
			glm::vec3 τ; // Torque
		};

		Derivative Evaluate(const State &initial, float dt, const Derivative &derivative);
		void Integrate(State &state, float dt);

		// Members
		State state;
		std::unordered_map<unsigned char, glm::vec3> forces;
		std::unordered_map<unsigned char, glm::vec3> torques;
		unsigned char handleId;
	};
}
