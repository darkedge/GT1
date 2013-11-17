#include "Solver.h"
#include "RigidBody.h"
#include "Transform.h"

using namespace GT1;
using glm::vec3;
using glm::mat3;

const int Solver::ITERATION_COUNT = 100;

// For the Successive Overrelaxation Method;
// @see http://mathworld.wolfram.com/SuccessiveOverrelaxationMethod.html
const float Solver::RELAXATION = 1.0f;
const float Solver::GRAVITY = 3.0f;

/************************************************************************/
/* Class vecN                                                           */
/************************************************************************/

vecN::vecN(int size) {
	this->size = size;
	cell = new float[size];
	LoadZero();
}

vecN::vecN(const vecN& copy) {
	this->size = copy.size;
	cell = new float[size];
	for(int i = 0; i < size; i++)
		cell[i] = copy[i];
}

vecN::~vecN() {
	delete[] cell;
	cell = nullptr;
}

void vecN::LoadZero() {
	for(int i = 0; i < size; i++)
		cell[i] = 0.0f;
}

// Unchecked
void vecN::InsertVec3(int i, glm::vec3 vec) {
	assert(i + 2 < size);
	assert(i >= 0);

	memcpy(cell + i, &vec[0], sizeof(float) * 3);
}

void vecN::Set(int i, float f) {
	assert(i >= 0);
	assert(i < size);

	cell[i] = f;
}

vec3 vecN::GetVec3(int i) const {
	assert(i + 2 < size);
	assert(i >= 0);

	return vec3(cell[i], cell[i + 1], cell[i + 2]);
}

/************************************************************************/
/* Class matSxN                                                         */
/************************************************************************/

matMxN::matMxN(int width, int height) {
	this->width = width;
	this->height = height;

	cell = new float*[height];
	for(int i = 0; i < height; i++)
		cell[i] = new float[width];
	
	LoadZero();
}

matMxN::matMxN(const matMxN& copy) {
	this->width = copy.width;
	this->height = copy.height;
	
	cell = new float*[height];
	for(int i = 0; i < height; i++)
		cell[i] = new float[width];

	for(int y = 0; y < height; y++)
		for(int x = 0; x < width; x++)
			cell[y][x] = copy[y][x];
}

matMxN::~matMxN() {
	for(int i = 0; i < height; i++)
		delete[] cell[i];

	delete[] cell;
}

void matMxN::LoadZero() {
	for(int y = 0; y < height; y++)
		for(int x = 0; x < width; x++)
			cell[y][x] = 0.0f;
}

void matMxN::Set(int x, int y, float val) {
	assert(x < width);
	assert(x >= 0);
	assert(y < height);
	assert(y >= 0);

	cell[y][x] = val;
}

void matMxN::InsertVec3(int x, int y, vec3 vec) {
	assert(x >= 0);
	assert(x + 2 < width);
	assert(y >= 0);
	assert(y < height);

	memcpy(cell[y] + x, &vec[0], sizeof(float) * 3);
}

/************************************************************************/
/* Transpose                                                            */
/************************************************************************/
matMxN Transpose(const matMxN& mat) {
	matMxN result(mat.height, mat.width);

	for(int x = 0; x < mat.width; x++)
		for(int y = 0; y < mat.height; y++)
			result[x][y] = mat[y][x];

	return result;
}

/************************************************************************/
/* Class MatCSR                                                         */
/************************************************************************/
// Converts a dense matrix to a Compressed Sparse Row matrix.
matCSR::matCSR(const matMxN& mat) {
	this->width = mat.width;
	this->height = mat.height;

	int row = 0;
	int k = 0;
	row_ptr.reserve(mat.height);
	for(int i = 0; i < mat.height; i++) {
		int sum = 0;
		for(int j = 0; j < mat.width; j++) {
			if(mat.cell[i][j] != 0.0f) {
				val.push_back(mat.cell[i][j]);
				col_ind.push_back(j);
				sum++;
			}
		}
		row_ptr.push_back(sum);
	}
}

/************************************************************************/
/* Class Constraint                                                     */
/************************************************************************/
ContactConstraint::ContactConstraint(glm::vec3 normal, float distance, glm::vec3 r_A, glm::vec3 r_B, RigidBody* a, RigidBody* b) {
	this->normal = normal;
	this->distance = distance;
	this->r_A = r_A;
	this->r_B = r_B;
	this->rb_A = a;
	this->rb_B = b;

	this->indexA = Solver::RegisterRigidBody(a);
	this->indexB = Solver::RegisterRigidBody(b);
}

void ContactConstraint::FillJacobian(matMxN& mat, int row, vecN& epsilon, vecN& min, vecN& max, float dt) {
	mat.InsertVec3(indexA * 6 + 0, row, -normal);
	mat.InsertVec3(indexA * 6 + 3, row, -glm::cross(r_A, normal));
	mat.InsertVec3(indexB * 6 + 0, row, normal);
	mat.InsertVec3(indexB * 6 + 3, row, glm::cross(r_B, normal));

	//epsilon->Set(row, distance * 0.00015f / dt);
	epsilon[row] = 0.0f;

	min[row] = 0.0f;
	max[row] = FLT_MAX;
}

FrictionConstraint::FrictionConstraint(glm::vec3 tangent, float mass, glm::vec3 r_A, glm::vec3 r_B, RigidBody* rb_A, RigidBody* rb_B) {
	this->tangent = tangent;
	this->mass = mass;
	this->r_A = r_A;
	this->r_B = r_B;
	this->rb_A = rb_A;
	this->rb_B = rb_B;

	this->indexA = Solver::RegisterRigidBody(rb_A);
	this->indexB = Solver::RegisterRigidBody(rb_B);
}

void FrictionConstraint::FillJacobian(matMxN& mat, int row, vecN& epsilon, vecN& min, vecN& max, float dt) {
	mat.InsertVec3(indexA * 6 + 0, row, -tangent);
	mat.InsertVec3(indexA * 6 + 3, row, -glm::cross(r_A, tangent));
	mat.InsertVec3(indexB * 6 + 0, row, tangent);
	mat.InsertVec3(indexB * 6 + 3, row, glm::cross(r_B, tangent));

	epsilon[row] = 0.0f;

	min[row] = -0.5f * mass * Solver::GRAVITY;
	max[row] = 0.5f * mass * Solver::GRAVITY;
}

/************************************************************************/
/* Class Solver                                                         */
/************************************************************************/

std::vector<Constraint*> Solver::constraints;
std::vector<RigidBody*> Solver::rigidBodies;

void Solver::Solve(float dt) {
	// Amount of constraints
	int s = (int) constraints.size();
	if(s == 0) return;

	// Number of rigid bodies * 6
	int matrixWidth = rigidBodies.size() * 6;

	// Velocity vector of all bodies
	vecN V(matrixWidth);

	// Inverted mass matrix
	matMxN M⁻¹(matrixWidth, matrixWidth);

	// External vector
	vecN Fext(matrixWidth);

	// Fill V, M⁻¹ and F
	for (int i = 0; i < (int) rigidBodies.size(); i++) {
		RigidBody* rb = rigidBodies[i];
		
//		Velocity vector layout
//		[Vx	Vy	Vz	ωx	ωy	ωz	[...]]
		V.InsertVec3(i * 6, rb->GetVelocity());
		V.InsertVec3(i * 6 + 3, rb->GetAngularVelocity());

		mat3 I⁻¹ = rb->GetInvertedInertiaTensor();

// 		Fill matrix like this:
// 
// 		m⁻¹	0	0	0	0	0	...
// 		0	m⁻¹	0	0	0	0	...
// 		0	0	m⁻¹	0	0	0	...
// 		0	0	0	I⁻¹	I⁻¹	I⁻¹	...
// 		0	0	0	I⁻¹	I⁻¹	I⁻¹	...
// 		0	0	0	I⁻¹	I⁻¹	I⁻¹	...
// 		...	...	...	...	...	...	[...]

		for(int j = 0; j < 3; j++)
			M⁻¹.Set(i * 6 + j, i * 6 + j, rb->GetInvertedMass());

		for(int y = 0; y < 3; y++)
			for(int x = 0; x < 3; x++)
				M⁻¹[i * 6 + 3 + y][i * 6 + 3 + x] = I⁻¹[y][x];

//		Force vector layout:
// 		[Fx	Fy	Fz	τx	τy	τz	[...]]
		Fext.InsertVec3(i * 6 + 0, rb->NetForce());
		Fext.InsertVec3(i * 6 + 3, rb->NetTorque());
	}

	// Create Jacobian
	matMxN J(matrixWidth, s);

	// "ϵ is the vector of force offsets
	// which allows contact forces to perform work"
	vecN epsilon(s);
	vecN projMin(s);
	vecN projMax(s);
	
	// Let each constraint fill its row
	for(int i = 0; i < s; i++)
		constraints[i]->FillJacobian(J, i, epsilon, projMin, projMax, dt);

	matMxN Jt = Transpose(J);

	// From the slides:
	// Ax = b where
	// A = J * M⁻¹ * Jt
	// b = ϵ / Δt - J * V1 - Δt * J * M⁻¹ * Fext
	matMxN A = (J * M⁻¹) * Jt;
	vecN b = (epsilon / dt) - (J * V) - dt * ((J * M⁻¹) * Fext);
	
	// Solve Ax = b iteratively using PGS
	// From this we get a Lagrange multiplier
	vecN λ = SolvePGS(A, b, RELAXATION, projMin, projMax, ITERATION_COUNT);

	// Multiplying the transposed Jacobian with the Lagrange multiplier
	// gives us a change in momentum
	vecN P = Jt * λ;

	// Apply momentum
	for(int i = 0; i < (int) rigidBodies.size(); i++)
		rigidBodies[i]->AddMomentum(P.GetVec3(i * 6 + 0), P.GetVec3(i * 6 + 3));
}

// @see: http://stackoverflow.com/questions/11719704/projected-gauss-seidel-for-lcp
// Projected Gauss-Seidel with successive over-relaxation (SOR)
vecN Solver::SolvePGS(matMxN& A, vecN& b, float relaxation, vecN& min, vecN& max, int iterations) {
	assert(A.width == b.GetSize());
	matCSR csr(A);

	int n = b.GetSize();
	float delta;
	vecN x = b;

	while(iterations--) {
		int i = 0;
		int begin = 0, end = 0;
		// for height
		for(int it = 0; it < (int) csr.row_ptr.size(); it++, i++) {
			// Reset delta
			delta = 0.0f;

			begin = end;
			end += csr.row_ptr[it];
			for(int j = begin; j < end; j++) {
				if(csr.col_ind[j] != i) {
					delta += csr.val[j] * x[csr.col_ind[j]];
				}
			}

			delta = (b[i] - delta) / A[i][i];

			// Apply relaxation
			x[i] += relaxation * (delta - x[i]);

			// Clamping
			if(x[i] < min[i])
				x[i] = min[i];
			else if(x[i] > max[i])
				x[i] = max[i];
		}
	}

	return x;
}

// Puts a rigid body into the solver and returns its index
// for future reference. If the body is already listed, then
// that body's index is returned instead.
int Solver::RegisterRigidBody(RigidBody* rb) {
	for(int i = 0; i < (int) rigidBodies.size(); i++)
		if(rigidBodies[i] == rb) return i;
	rigidBodies.push_back(rb);
	return (int) (rigidBodies.size() - 1);
}

void Solver::Clear() {
	rigidBodies.clear();
	for(Constraint* c : constraints)
		delete c;
	constraints.clear();
}