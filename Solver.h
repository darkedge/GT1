#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <map>

namespace GT1 {
	class matMxN;
	class RigidBody;

	/************************************************************************/
	/* VecN                                                                 */
	/************************************************************************/
	class vecN {
	public:
		vecN(int size);
		vecN(const vecN& copy);
		~vecN();
		int GetSize() const { return size; }
		void LoadZero();
		void InsertVec3(int offset, glm::vec3 vec);
		glm::vec3 GetVec3(int offset) const;
		void Set(int i, float f);
		const char* ToString() const;

		float& operator[](int index) const {
			assert(index >= 0);
			assert(index < size);
			return cell[index];
		}

		friend const vecN operator-(const vecN& a, const vecN& b) {
			vecN result(a.size);
			for(int i = 0; i < b.size; i++)
				result[i] = a[i] - b[i];
			return result;
		}

		friend const vecN operator*(const float& f, const vecN& vec) {
			vecN result(vec.size);
			for(int i = 0; i < vec.size; i++)
				result[i] = f * vec[i];
			return result;
		}

		friend const vecN operator/(const vecN& vec, const float& f) {
			vecN result(vec.size);
			for(int i = 0; i < vec.size; i++)
				result[i] = vec[i] / f;
			return result;
		}

		friend vecN operator*(const matMxN& a, const vecN& vec);

		friend std::ostream& operator<<(std::ostream&, const vecN&);

	protected:
		int size;
		float* cell;
	};

	/************************************************************************/
	/* MatMxN                                                               */
	/************************************************************************/
	class matMxN {
	public:
		matMxN(int width, int height);
		matMxN(const matMxN& copy);
		~matMxN();
		int GetWidth() const { return width; }
		int GetHeight() const { return height; }
		void LoadZero();

		void Set(int i, int j, float val);
		void InsertVec3(int x, int y, glm::vec3 vec);

		float* operator[](int index) const {
			assert(index < height);
			assert(index >= 0);

			return cell[index];
		}

		matMxN operator*(matMxN mat) {
			assert(this->width == mat.height);

			matMxN result(mat.width, this->height);
			for(int y = 0; y < this->height; y++) 
				for(int x = 0; x < mat.width; x++) {
					float sum = 0.0f;
					for(int i = 0; i < mat.height; i++)
						sum += this->cell[y][i] * mat[i][x];
					result[y][x] = sum;
				}

			return result;
		}

		vecN operator*(vecN vec) {
			assert(this->width == vec.GetSize());

			vecN result(this->height);
			for(int y = 0; y < this->height; y++) {
				float sum = 0.0f;
				for(int x = 0; x < this->width; x++)
					sum += this->cell[y][x] * vec[x];
				result[y] = sum;
			}

			return result;
		}

		int width;
		int height;

		float** cell;
	protected:
	};

	/************************************************************************/
	/* MatCSR                                                               */
	/************************************************************************/
	// Compressed Sparse Row matrix
	class matCSR {
	public:
		matCSR(const matMxN& mat);

		// Very efficient matrix-vector multiplication
		vecN operator*(const vecN& vec) {
			assert(this->width == vec.GetSize());

			vecN result(this->height);
			int begin = 0, end = 0;
			for(int j = 0; j < height; j++) {
				begin = end;
				end += row_ptr[j];
				for(int i = begin; i < end; i++) {
					result[j] += val[i] * vec[col_ind[i]]; 
				}
			}

			return result;
		}

		int width;
		int height;

		std::vector<float> val;
		std::vector<int> col_ind;
		std::vector<int> row_ptr;
	};

	/************************************************************************/
	/* Constraint                                                           */
	/************************************************************************/
	class Constraint {
	public:		
		virtual void FillJacobian(matMxN& mat, int row, vecN& epsilon, vecN& min, vecN& max, float dt) = 0;		
	};

	class ContactConstraint : public Constraint {
	public:
		ContactConstraint(glm::vec3 normal, float distance, glm::vec3 r_A, glm::vec3 r_B, RigidBody* a, RigidBody* b);
		void FillJacobian(matMxN& mat, int row, vecN& epsilon, vecN& min, vecN& max, float dt);

	protected:
		// Jacobian row offset
		int indexA;
		int indexB;

		glm::vec3 normal;
		float distance;
		glm::vec3 r_A;
		glm::vec3 r_B;
		RigidBody* rb_A;
		RigidBody* rb_B;
	};

	class FrictionConstraint : public Constraint {
	public:
		FrictionConstraint(glm::vec3 tangent, float mass, glm::vec3 r_A, glm::vec3 r_B, RigidBody* rb_A, RigidBody* rb_B);
		void FillJacobian(matMxN& mat, int row, vecN& epsilon, vecN& min, vecN& max, float dt);

	protected:
		int indexA;
		int indexB;

		glm::vec3 tangent; // Tangent unit vector
		float mass;
		glm::vec3 r_A;
		glm::vec3 r_B;
		RigidBody* rb_A;
		RigidBody* rb_B;
	};

	/************************************************************************/
	/* Solver                                                               */
	/************************************************************************/
	class Solver
	{
	public:

		static void Clear();

		static void AddConstraint(Constraint* c) { constraints.push_back(c); }
		static void Solve(float dt);
		static int RegisterRigidBody(RigidBody* rb);

		static const int ITERATION_COUNT;
		static const float RELAXATION;
		static const float GRAVITY;

	protected:
		static vecN SolvePGS(matMxN& A, vecN& b, float r, vecN& min, vecN& max, int iterations);

		static std::vector<Constraint*> constraints;
		static std::vector<RigidBody*> rigidBodies;
	};
}
