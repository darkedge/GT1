#include "RigidBody.h"
#include "Transform.h"

using namespace GT1;
using glm::vec3;

RigidBody::RigidBody(Transform* transform, float mass, glm::mat3 inertiaTensor)
{
	this->transform = transform;
	handleId = 0;
	state.m⁻¹ = 1.0f / mass;
	state.I⁻¹ = glm::inverse(inertiaTensor);
}

void RigidBody::Tick(float dt) {
	Integrate(state, dt);
}

// Adds a thruster to the rigid body.
unsigned char RigidBody::AddThruster(vec3 position, vec3 magnitude) {
	// Force
	forces.emplace(handleId, magnitude);

	// Torque
	torques.emplace(handleId, glm::cross(position, magnitude));

	return handleId++;
}

// Applies an impulse directly to the rigid body's momentum.
void RigidBody::Impulse(const vec3& position, const vec3& magnitude) {
	state.p += magnitude;
	//printf("Not implemented: %s\nIn file: %s\nLine number: %i", __FUNCSIG__, __FILE__, __LINE__);
}

// Removes a thruster from the rigid body.
void RigidBody::RemoveThruster(unsigned char handle) {
	forces.erase(handle);
	torques.erase(handle);
}

void RigidBody::AddMomentum(const vec3& linear, const vec3& angular) {
	state.p += linear;
	state.L += angular;
}

// Compute the net force on the rigid body.
// TODO: Calculate once per tick
vec3 RigidBody::NetForce() const {
	vec3 netForce;
	for (const auto &pair : forces) netForce += pair.second;
	return netForce;
}

// Compute the net torque on the rigid body.
// TODO: Calculate once per tick
vec3 RigidBody::NetTorque() const {
	vec3 netTorque;
	for (const auto &pair : torques) netTorque += pair.second;
	return netTorque;
}

RigidBody::Derivative RigidBody::Evaluate(const State &initial, float dt, const Derivative &derivative)
{
	// State += Derivative * dt
	State state = initial;
	state.p	+= derivative.F * dt;
	state.L	+= derivative.τ	* dt;

	// Calculate new derivative
	// Use state if necessary
	Derivative output;
	output.F = NetForce();
	output.τ = NetTorque();

	return output;
}

void RigidBody::Integrate(State &state, float dt)
{
	// 4 samples
	Derivative k1 = Evaluate(state, 0.0f,		Derivative());
	Derivative k2 = Evaluate(state, dt * 0.5f,	k1			);
	Derivative k3 = Evaluate(state, dt * 0.5f,	k2			);
	Derivative k4 = Evaluate(state, dt,			k3			);

	// Compute the weighted averaged derivative
	const vec3 dpdt = (1.0f/6.0f) * (k1.F + 2.0f * (k2.F + k3.F) + k4.F);
	const vec3 dLdt = (1.0f/6.0f) * (k1.τ + 2.0f * (k2.τ + k3.τ) + k4.τ);

	// State += Derivative * dt
	state.p	+= dpdt * dt;
	state.L	+= dLdt * dt;

	// Recalculate velocities
	state.v	= state.p	* state.m⁻¹;
	state.ω	= state.I⁻¹	* state.L;
	
	// Apply velocities
	transform->position	+= state.v;
	transform->orientation = transform->orientation * glm::quat(state.ω);
}

RigidBody::State::State(float m, glm::mat3 I) {
	this->m⁻¹	= 1.0f / m;
	this->I⁻¹	= glm::inverse(I);
}