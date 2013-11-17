#pragma once
#include <vector>
#include <glm/glm.hpp>
#include "Solver.h"
#include "Polyhedron.h"

#define SCRWIDTH 1280
#define SCRHEIGHT 720

namespace GT1 {
	class Transform;
	class Polyhedron;
	class RigidBody;

	struct ContactPoint {
		float depth;		// Penetration magnitude
		glm::vec3 point;	// Point in world space
		glm::vec3 normal;	// Reference normal
	};

	struct SATResult{
		float min;
		float max;
		glm::vec3 minVertex;
		glm::vec3 maxVertex;
		glm::vec3 minTransformedVertex;
		glm::vec3 maxTransformedVertex;
	};

	class Game
	{
	public:
		void Init();
		void Tick(float dt);
		void Keyboard(int key, int action);
		void MouseButton(int button, int action);
		void MouseMoved(double xpos, double ypos);
		void Cleanup();
		void RecalculateProjection(int width, int height);

	protected:
		void RecalculateCamera();

		// Collision detection
		void Project(Polyhedron* mesh, glm::vec3 axis, SATResult& result);
		bool TestIntersection(Polyhedron* C0, Polyhedron* C1, std::vector<ContactPoint>& result);
		glm::vec3 ComputeIntersection(const glm::vec3& prev, const glm::vec3& next, const Plane& plane);

		std::vector<Transform*> transforms;

		bool paused;
		float accumulator;
	};
}
