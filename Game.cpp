#include "Game.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "RigidBody.h"
#include "Transform.h"
#include "Polyhedron.h"
#include "shader.hpp"
#include <cmath>

using namespace GT1;
using glm::vec3;
using glm::mat4;

// Shader program
GLuint program;

// Location of MVP uniform in the program
GLint mvpLocation;

// MVP matrix
mat4 projection;
mat4 view;

Transform* cannonBall;

void Game::Init() {
	accumulator = 0.0f;
	paused = false;

	Transform* floor = new Transform(
		vec3(5,-1,5),	// Position
		vec3(0,0,0),	// Orientation
		vec3(20,1,20)	// Scale
	);
	floor->rigidBody->SetInvertedMass(0.0f);
	floor->rigidBody->SetInvertedInertiaTensor(glm::mat3(0.0f));
	transforms.push_back(floor);

	for(int i = 0; i<3; i++) {
		Transform* block = new Transform(
			vec3(i*4.1f,1,0),	// Position
			vec3(0,0,0),	// Orientation
			vec3(4, 2, 2)	// Scale
		);
		block->rigidBody->AddThruster(vec3(), vec3(0,-3,0));
		transforms.push_back(block);
	}

	for(int i = 1; i<3; i++) {
		Transform* block = new Transform(
			vec3(i*4.1f-2.05f,3,0),	// Position
			vec3(0,0,0),	// Orientation
			vec3(4, 2, 2)	// Scale
			);
		block->rigidBody->AddThruster(vec3(), vec3(0,-3,0));
		transforms.push_back(block);
	}

	for(int i = 2; i<3; i++) {
		Transform* block = new Transform(
			vec3(i*4.1f-4.1f,5,0),	// Position
			vec3(0,0,0),	// Orientation
			vec3(4, 2, 2)	// Scale
			);
		block->rigidBody->AddThruster(vec3(), vec3(0,-3,0));
		transforms.push_back(block);
	}

	cannonBall = new Transform(
		vec3(4.1f,1.5,10),	// Position
		vec3(0,0,0),	// Orientation
		vec3(2, 2, 2)	// Scale
	);
	cannonBall->rigidBody->AddThruster(vec3(), vec3(0,-3,0));
	transforms.push_back(cannonBall);
	
	// Create and compile our GLSL program from the shaders
	program = LoadShaders( "shaders/TransformVertexShader.vertexshader", "shaders/ColorFragmentShader.fragmentshader" );

	// Get location of MVP uniform in the program
	mvpLocation = glGetUniformLocation(program, "MVP");

	RecalculateProjection(SCRWIDTH, SCRHEIGHT);
	
	view = glm::lookAt(
		vec3(10.0f, 5.0f, 10.0f),	// Position
		vec3(0.0f, 0.0f, 0.0f),	// Target
		vec3(0.0f, 1.0f, 0.0f)	// Up vector
	);
}

void Game::Tick(float dt) {

	/************************************************************************/
	/* Render                                                               */
	/************************************************************************/

	// Clear screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Select shader
	glUseProgram(program);	

	for(auto* transform : transforms) {
		if(transform->polyhedron) {
			// Calculate new MVP matrix
			mat4 model = transform->GetMatrix();
			mat4 MVP = projection * view * model;
			glUniformMatrix4fv(mvpLocation, 1, GL_FALSE, &MVP[0][0]);

			transform->polyhedron->Render();
		}
	}

	/************************************************************************/
	/* Update                                                               */
	/************************************************************************/

	if(!paused) {
		for(Transform* const t : transforms) {
			t->Tick(0.01f);
		}

		Solver::Clear();

		/************************************************************************/
		/* Constraints                                                          */
		/************************************************************************/
		std::vector<ContactPoint> contactPoints;
		for(int i = 0; i < (int) transforms.size(); i++) {
			Transform* t_A = transforms[i];
			Polyhedron* p_A = t_A->polyhedron;
			RigidBody* rb_A = t_A->rigidBody;
			vec3 v_A = rb_A->GetVelocity();

			for(int j = i + 1; j < (int) transforms.size(); j++) {
				Transform* t_B = transforms[j];
				Polyhedron* p_B = t_B->polyhedron;
				RigidBody* rb_B = t_B->rigidBody;
				vec3 v_B = rb_B->GetVelocity();

				if(TestIntersection(p_A, p_B, contactPoints)) {
					for(const ContactPoint& p : contactPoints) {
						vec3 point = p.point;	// Incident vertex
						vec3 normal = p.normal;	// Reference normal
						float depth = p.depth;	// Penetration magnitude
						vec3 penetration = normal * p.depth; // vector from face to point

						vec3 r_A = point - penetration - t_A->position;
						vec3 r_B = point - t_B->position;

						ContactConstraint* c = new ContactConstraint(
							normal,		// Normal
							-depth,		// Penetration
							r_A,		// r of A
							r_B,		// r of B
							rb_A,		// Rigid body A
							rb_B		// Rigid body B
						);
						Solver::AddConstraint(c);
						/*
						// mass = 10.0f for all objects
						FrictionConstraint* f = new FrictionConstraint(
							normal,
							10.0f / contactPoints.size(),
							r_A,
							r_B,
							rb_A,
							rb_B
						);
						Solver::AddConstraint(f);
							
						f = new FrictionConstraint(
							normal,
							10.0f / contactPoints.size(),
							r_A,
							r_B,
							rb_A,
							rb_B
						);
						Solver::AddConstraint(f);
						*/
					}
					contactPoints.clear();
				}
			}
		}

		Solver::Solve(0.01f);
	}
}

// Line-plane intersection
// This assumes that there is an intersection
// and we just need to find the point of intersection.
vec3 Game::ComputeIntersection(const glm::vec3& V, const glm::vec3& W, const Plane& plane) {
	vec3 N = plane.normal;
	float d = plane.distance;

	float denominator = glm::dot(V,N) - glm::dot(W,N);
	float a = (d - glm::dot(W,N)) / denominator;
	vec3 P = V * a + W * (1.0f - a);
	return P;
}

void Game::MouseMoved(double xpos, double ypos) {
	// TODO
}

void Game::MouseButton(int button, int action) {
	// TODO
	/*
	switch(button) {
	
	}
	*/
}

void Game::Keyboard(int key, int action) {
	switch(key) {
	case GLFW_KEY_P:
		if(action == GLFW_PRESS)
			paused = !paused;
		break;
	case GLFW_KEY_SPACE:
		if(action == GLFW_PRESS)
			cannonBall->rigidBody->Impulse(vec3(), vec3(0,2,-10));
	case GLFW_KEY_W:
		break;
	case GLFW_KEY_A:
		break;		
	case GLFW_KEY_S:
		break;
	case GLFW_KEY_D:
		break;
	case GLFW_KEY_Q:
		break;
	case GLFW_KEY_E:
		break;
	}
}

void Game::Cleanup() {
	glDeleteProgram(program);
}

void Game::RecalculateProjection(int width, int height) {
	float ratio = (float) width / (float) height;

	projection = glm::perspective(
		90.0f,	// Vertical FOV
		ratio,	// Screen ratio
		1.0f,	// Near Z plane
		1000.0f	// Far Z plane
	);
}

void Game::Project ( Polyhedron* poly , vec3 axis , SATResult& result ) {
	result.minVertex = poly->GetVertex(0);
	result.maxVertex = result.minVertex;
	result.minTransformedVertex = poly->GetTransformedVertex(0);
	result.maxTransformedVertex = result.minTransformedVertex;
	result.min = glm::dot (axis , result.minTransformedVertex) ;
	result.max = result.min;

	for (int i = 1; i < poly->GetVertexCount(); i++ ) {
		vec3 transformedVertex = poly->GetTransformedVertex(i);
		vec3 vertex = poly->GetVertex(i);
		float val = glm::dot (axis , transformedVertex ) ;
		if ( val < result.min ) {
			result.minTransformedVertex = transformedVertex;
			result.min = val ;
			result.minVertex = vertex;
		} else if (val > result.max) {
			result.maxTransformedVertex = transformedVertex;
			result.max = val ;
			result.maxVertex = vertex;
		}
	}
}

// From the slides
bool Game::TestIntersection(Polyhedron* C0, Polyhedron* C1, std::vector<ContactPoint>& result) {
	bool useFaceNormal = true;
	float bestDistance = FLT_MAX;
	Polyhedron* bestPolyhedron = nullptr; // For looking up reference face
	Polyhedron* incidentPoly = nullptr;
	vec3 bestDirection;
	SATResult result0, result1;
	vec3 transformedIncidentVertex;
	vec3 incidentVertex;

	for (int i = 0; i < C0->GetFaceCount(); i++) {
		vec3 D = C0->GetTransformedNormal(i);
		Project(C0, D, result0);
		Project(C1, D, result1);
		if (result0.max < result1.min || result1.max < result0.min) return false;
		float p1 = result0.max - result1.min;
		float p2 = result1.max - result0.min;
		if(p1 < p2 && p1 < bestDistance) {
			bestDistance = p1;
			bestDirection = D;
			bestPolyhedron = C0;
			incidentPoly = C1;
			transformedIncidentVertex = result1.minTransformedVertex;
			incidentVertex = result1.minVertex;
		}
		if(p2 < p1 && p2 < bestDistance) {
			bestDistance = p2;
			bestDirection = D;
			bestPolyhedron = C0;
			incidentPoly = C1;
			transformedIncidentVertex = result1.maxTransformedVertex;
			incidentVertex = result1.maxVertex;
		}
	}
	for (int i = 0; i < C1->GetFaceCount(); i++) {
		vec3 D = C1->GetTransformedNormal(i);
		Project(C0, D, result0);
		Project(C1, D, result1);
		if (result0.max < result1.min || result1.max < result0.min) return false;
		float p1 = result0.max - result1.min;
		float p2 = result1.max - result0.min;
		if(p1 < p2 && p1 < bestDistance) {
			bestDistance = p1;
			bestDirection = D;
			bestPolyhedron = C1;
			incidentPoly = C0;
			transformedIncidentVertex = result0.minTransformedVertex;
			incidentVertex = result0.minVertex;
		}
		if(p2 < p1 && p2 < bestDistance) {
			bestDistance = p2;
			bestDirection = D;
			bestPolyhedron = C1;
			incidentPoly = C0;
			transformedIncidentVertex = result0.maxTransformedVertex;
			incidentVertex = result0.maxVertex;
		}
	}
	// test cross products of pairs of edges
	/*
	std::vector<vec3> crosses;
	for (int i = 0; i < C0->GetEdgeCount(); i++) {
		for (int j = 0; j < C1->GetEdgeCount(); j++) {
			vec3 D = glm::cross(C0->GetTransformedEdge(i).Get(),C1->GetTransformedEdge(j).Get());
			if (D == vec3()) continue;
			D = glm::normalize(D);
			if(std::find(crosses.begin(), crosses.end(), D) == crosses.end()) {
				crosses.push_back(D);
				Project(C0,D, result0);
				Project(C1,D, result1);
				if (result0.max < result1.min || result1.max < result0.min) return false;
				float p1 = result0.max - result1.min;
				float p2 = result1.max - result0.min;
				if(p1 < p2 && p1 < bestDistance) {
					useFaceNormal = false;
					bestDistance = p1;
					bestDirection = D;

				}
				if(p2 < p1 && p2 < bestDistance) {
					useFaceNormal = false;
					bestDistance = p2;
					bestDirection = D;
				}
			}
		}
	}
	*/
	
	Face referenceFace;
	if(useFaceNormal) {
		// bestDirection is a face normal
		float bestDot = FLT_MAX;
		for(const Face& face : bestPolyhedron->faces) {
			if(face.GetTransformedNormal() == bestDirection) {
				float dot = fabs(glm::dot(transformedIncidentVertex - face.GetTransformedVertex(0), face.GetTransformedNormal()));
				if(dot < bestDot) {
					bestDot = dot;
					referenceFace = face.GetTransformedCopy();
				}
			}
		}
	} else {
		// Incident point
		// TODO
	}

	Face incidentFace;
	float smallestDot = FLT_MAX;
	for(const Face& face : incidentPoly->faces) {
		if(face.Contains(incidentVertex)) {
			float dot = glm::dot(face.GetTransformedNormal(), referenceFace.normal);
			if(dot < smallestDot) {
				smallestDot = dot;
				incidentFace = face.GetTransformedCopy();
			}
		}
	}

	// Construct clipping planes from reference edges
	std::vector<Plane> clippingPlanes;
	for(const Edge& edge : referenceFace.edges) {
		Plane plane;
		plane.normal = glm::cross(referenceFace.normal, glm::normalize(edge.Get()));
		plane.distance = glm::dot(plane.normal, edge.v0);
		clippingPlanes.push_back(plane);
	}

	// Lambda for adding contact points
	auto AddPoint = [&] (vec3 intersection, vec3 bestDirection, float depth) {
		ContactPoint point;
		point.point = intersection;
		point.normal = glm::normalize(bestDirection);
		point.depth = glm::dot(intersection - referenceFace.vertices[0], referenceFace.normal);
		result.push_back(point);
	};

	// Clip incident face
	std::vector<vec3> contactPoints;
	for(const Plane& plane : clippingPlanes) {
		for(const Edge& edge : incidentFace.edges) {
			vec3 S = edge.v0;
			vec3 E = edge.v1;
			if(glm::dot(E, plane.normal) - plane.distance >= 0) {
				if(glm::dot(S, plane.normal) - plane.distance < 0) {
					vec3 intersection = ComputeIntersection(S, E, plane);
					// Keep points below reference face
					if(glm::dot(intersection, referenceFace.plane.normal) - referenceFace.plane.distance < 0) {
						contactPoints.push_back(intersection);
						float depth = glm::dot(intersection - referenceFace.vertices[0], referenceFace.normal);
						AddPoint(intersection, bestDirection, depth);
					}
				}
				if(std::find(contactPoints.begin(), contactPoints.end(), E) == contactPoints.end()) {
					contactPoints.push_back(E);
					float depth = glm::dot(E - referenceFace.vertices[0], referenceFace.normal);
					AddPoint(E, bestDirection, depth);
				}
			} else if (glm::dot(S, plane.normal) - plane.distance >= 0) {
				vec3 intersection = ComputeIntersection(S, E, plane);
				if(glm::dot(intersection, referenceFace.plane.normal) - referenceFace.plane.distance < 0) {
					contactPoints.push_back(intersection);
					float depth = glm::dot(intersection - referenceFace.vertices[0], referenceFace.normal);
					AddPoint(intersection, bestDirection, depth);
				}
			}
		}
	}

	/*
	// Render intersection points (pre-transformed)
	glUseProgram(0);
	mat4 MVP = projection * view;
	glLoadMatrixf(&MVP[0][0]);
	glColor4f(1.0f, 0.0f, 0.0f, 1.0f);
	glPointSize(7.0f);
	glDisable(GL_DEPTH_TEST);
	glBegin(GL_POINTS);
	for(const vec3& point : contactPoints) {
		glVertex3f(point.x, point.y, point.z);
	}
	glEnd();
	glEnable(GL_DEPTH_TEST);
	*/

	return true;
}
