#pragma once

/* 
 * Tinazzi Mattia
 * Take from Millington's Book
 */ 


#include <cfloat>
#include <cmath>

namespace physic
{
    /*!
     * Holds a vector in 3 dimensions.
     */
	class Vector3
	{
	public:
		
		/*! Holds the value along the x axis. */
		float x;
		/*! Holds the value along the y axis. */
		float y;
		/*! Holds the value along the z axis. */
		float z;

	private:

		/*! Padding to ensure 4 word alignment. */
		float pad;

	public:

		/*! Default constructor. It creates a zero vector. */
		Vector3() : x(0), y(0), z(0) { }

		/*!
         * Explicit constructor. It creates a vector with the given
         * components.
         */
        Vector3(const float x, const float y, const float z)
            : x(x), y(y), z(z) {}

		const static Vector3 ZERO;
        const static Vector3 GRAVITY;
		//const static Vector3 LOW_GRAVITY;
  //      const static Vector3 HIGH_GRAVITY;
        const static Vector3 UP;
        const static Vector3 RIGHT;
        const static Vector3 OUT_OF_SCREEN;
        const static Vector3 X;
        const static Vector3 Y;
        const static Vector3 Z;

		/*! Access to a specific component. */
		float operator[](size_t i) const
        {
            if (i == 0) return x;
            if (i == 1) return y;
            return z;
        }

		/*! Access to a specific component. */
        float& operator[](size_t i)
        {
            if (i == 0) return x;
            if (i == 1) return y;
            return z;
        }

        /*! Adds the given vector to this. */
        void operator+=(const Vector3& v)
        {
            x += v.x;
            y += v.y;
            z += v.z;
        }

        /*!
         * Returns the value of the given vector added to this.
         */
        Vector3 operator+(const Vector3& v) const
        {
            return Vector3(x + v.x, y + v.y, z + v.z);
        }

        /*! Subtracts the given vector from this. */
        void operator-=(const Vector3& v)
        {
            x -= v.x;
            y -= v.y;
            z -= v.z;
        }

        /*!
         * Returns the value of the given vector subtracted from this.
         */
        Vector3 operator-(const Vector3& v) const
        {
            return Vector3(x - v.x, y - v.y, z - v.z);
        }

        /*! Multiplies this vector by the given scalar. */
        void operator*=(const float value)
        {
            x *= value;
            y *= value;
            z *= value;
        }

        /*! Returns a copy of this vector multiplied by the given value. */
        Vector3 operator*(const float value) const
        {
            return Vector3(x * value, y * value, z * value);
        }

        /*!
         * Calculates and returns a component-wise product of this
         * vector with the given vector.
         */
        Vector3 ComponentProduct(const Vector3& vector) const
        {
            return Vector3(x * vector.x, y * vector.y, z * vector.z);
        }

        /*!
         * Performs a component-wise product with the given vector and
         * sets this vector to its result.
         */
        void ComponentProductUpdate(const Vector3& vector)
        {
            x *= vector.x;
            y *= vector.y;
            z *= vector.z;
        }

        /*!
         * Calculates and returns the cross product of this vector
         * with the given vector.
         */
        Vector3 CrossProduct(const Vector3& vector) const
        {
            return Vector3(y * vector.z - z * vector.y,
                           z * vector.x - x * vector.z,
                           x * vector.y - y * vector.x);
        }

        /*!
         * Updates this vector to be the cross product of its current
         * value and the given vector.
         */
        void operator%=(const Vector3& vector)
        {
            *this = CrossProduct(vector);
        }

        /*!
         * Calculates and returns the cross product of this vector
         * with the given vector.
         */
        Vector3 operator%(const Vector3& vector) const
        {
            return Vector3(y * vector.z - z * vector.y,
                           z * vector.x - x * vector.z,
                           x * vector.y - y * vector.x);
        }

        /*!
         * Calculates and returns the dot product of this vector
         * with the given vector.
         */
        float DotProduct(const Vector3& vector) const
        {
            return x * vector.x + y * vector.y + z * vector.z;
        }

        /*!
         * Calculates and returns the dot product of this vector
         * with the given vector.
         */
        float operator*(const Vector3& vector) const
        {
            return x * vector.x + y * vector.y + z * vector.z;
        }

        /*!
         * Adds the given vector to this, scaled by the given amount.
         */
        void AddScaledVector(const Vector3& vector, float scale)
        {
            x += vector.x * scale;
            y += vector.y * scale;
            z += vector.z * scale;
        }

        /*! Gets the magnitude of this vector. */
        float Magnitude() const
        {
            return sqrt(x * x + y * y + z * z);
        }

        /*! Gets the squared magnitude of this vector. */
        float SquareMagnitude() const
        {
            return x * x + y * y + z * z;
        }

        /*! Limits the size of the vector to the given maximum. */
        void Trim(float size)
        {
            if (SquareMagnitude() > size * size)
            {
                Normalise();
                x *= size;
                y *= size;
                z *= size;
            }
        }

        /*! Turns a non-zero vector into a vector of unit length. */
        void Normalise()
        {
            float l = Magnitude();
            if (l > 0)
            {
                (*this) *= ((float)1) / l;
            }
        }

        /*! Returns the normalised version of a vector. */
        Vector3 Versor() const
        {
            Vector3 result = *this;
            result.Normalise();
            return result;
        }

        /*! Checks if the two vectors have identical components. */
        bool operator==(const Vector3& other) const
        {
            return	x == other.x &&
					y == other.y &&
					z == other.z;
        }

        /*! Checks if the two vectors have non-identical components. */
        bool operator!=(const Vector3& other) const
        {
            return !(*this == other);
        }

        /*!
         * Checks if this vector is component-by-component less than
         * the other.
         *
         * \note This does not behave like a single-value comparison:
         * !(a < b) does not imply (b >= a).
         */
        bool operator<(const Vector3& other) const
        {
            return	x < other.x && 
					y < other.y && 
					z < other.z;
        }

        /*!
         * Checks if this vector is component-by-component less than
         * the other.
         *
         * \note This does not behave like a single-value comparison:
         * !(a < b) does not imply (b >= a).
         */
        bool operator>(const Vector3& other) const
        {
            return	x > other.x && 
					y > other.y && 
					z > other.z;
        }

        /*!
         * Checks if this vector is component-by-component less than
         * the other.
         *
         * \note This does not behave like a single-value comparison:
         * !(a <= b) does not imply (b > a).
         */
        bool operator<=(const Vector3& other) const
        {
            return	x <= other.x && 
					y <= other.y && 
					z <= other.z;
        }

        /*!
         * Checks if this vector is component-by-component less than
         * the other.
         *
         * \note This does not behave like a single-value comparison:
         * !(a <= b) does not imply (b > a).
         */
        bool operator>=(const Vector3& other) const
        {
            return	x >= other.x && 
					y >= other.y && 
					z >= other.z;
        }

        /*! Zero all the components of the vector. */
        void Clear()
        {
            x = y = z = 0;
        }

        /*! Flips all the components of the vector. */
        void Invert()
        {
			x = -x;
			y = -y;
			z = -z;
        }
	};

	/*!
     * Holds a quaternion.
     *
     * \note Angular velocity and acceleration can be correctly
     * represented as vectors. Quaternions are only needed for
	 * orientation. A quaternion is only a valid rotation if it is
	 * normalised: i.e. it has a length of 1.
     */
    class Quaternion
    {
    public:

        union {
            struct {
                /*!
                 * Holds the float component of the quaternion.
                 */
                float r;

                /*!
                 * Holds the first complex component of the
                 * quaternion.
                 */
                float i;

                /*!
                 * Holds the second complex component of the
                 * quaternion.
                 */
                float j;

                /*!
                 * Holds the third complex component of the
                 * quaternion.
                 */
                float k;
            };

            /*!
             * Holds the quaternion data in array form.
             */
            float data[4];
        };

        /*!
         * Default constructor. It creates a quaternion representing
         * a zero rotation.
         */
        Quaternion() : r(1), i(0), j(0), k(0) { }

        /*!
         * Explicit constructor. It creates a quaternion with the given
         * components.
         */
        Quaternion(const float r, const float i, const float j, const float k)
            : r(r), i(i), j(j), k(k) { }

        /*!
         * Normalises the quaternion to unit length, making it a valid
         * orientation quaternion.
         */
        void Normalise()
        {
            float d = r * r + i * i + j * j + k * k;

            if (d == 0) 
			{
                r = 1;
                return;
            }

            d = ((float)1) / sqrt(d);
            r *= d;
            i *= d;
            j *= d;
            k *= d;
        }

        /*!
         * Multiplies this by the given quaternion.
         */
        void operator*=(const Quaternion& multiplier)
        {
            Quaternion q = *this;
            r = q.r * multiplier.r - q.i * multiplier.i -
                q.j * multiplier.j - q.k * multiplier.k;
            i = q.r * multiplier.i + q.i * multiplier.r +
                q.j * multiplier.k - q.k * multiplier.j;
            j = q.r * multiplier.j + q.j * multiplier.r +
                q.k * multiplier.i - q.i * multiplier.k;
            k = q.r * multiplier.k + q.k * multiplier.r +
                q.i * multiplier.j - q.j * multiplier.i;
        }

        /*!
         * Adds the given vector to this, scaled by the given amount.
         */
        void AddScaledVector(const Vector3& vector, float scale)
        {
            Quaternion q(	0,
							vector.x * scale,
							vector.y * scale,
							vector.z * scale);

            q *= *this;
            r += q.r * ((float)0.5);
            i += q.i * ((float)0.5);
            j += q.j * ((float)0.5);
            k += q.k * ((float)0.5);
        }

        void RotateByVector(const Vector3& vector)
        {
            Quaternion q(0, vector.x, vector.y, vector.z);
            (*this) *= q;
        }
    };

	/*!
     * Holds a transform matrix, consisting of a rotation matrix and
     * a position. 
	 *
	 * \note The matrix has 12 elements, it is assumed that the
     * remaining four are (0, 0, 0, 1).
     */
    class Matrix4
    {
    public:

        /*!
         * Holds the transform matrix data in array form.
         */
        float data[12];

		/*
         * Creates an identity matrix.
         */
        Matrix4()
        {
            data[1] = data[2] = data[3] = data[4] = data[6] =
                data[7] = data[8] = data[9] = data[11] = 0;

			// Diagonal elements
            data[0] = data[5] = data[10] = 1;
        }

        /*
         * Sets the matrix to be a diagonal matrix with the given coefficients.
         */
        void SetDiagonal(float a, float b, float c)
        {
            data[0]		= a;
            data[5]		= b;
            data[10]	= c;
        }

        /*
         * Returns a matrix which is this matrix multiplied by the given
         * other matrix.
         */
        Matrix4 operator*(const Matrix4& o) const
        {
            Matrix4 result;
            result.data[0]	= (o.data[0] * data[0]) + (o.data[4] * data[1]) + (o.data[8] * data[2]);
            result.data[4]	= (o.data[0] * data[4]) + (o.data[4] * data[5]) + (o.data[8] * data[6]);
            result.data[8]	= (o.data[0] * data[8]) + (o.data[4] * data[9]) + (o.data[8] * data[10]);

            result.data[1]	= (o.data[1] * data[0]) + (o.data[5] * data[1]) + (o.data[9] * data[2]);
            result.data[5]	= (o.data[1] * data[4]) + (o.data[5] * data[5]) + (o.data[9] * data[6]);
            result.data[9]	= (o.data[1] * data[8]) + (o.data[5] * data[9]) + (o.data[9] * data[10]);

            result.data[2]	= (o.data[2] * data[0]) + (o.data[6] * data[1]) + (o.data[10] * data[2]);
            result.data[6]	= (o.data[2] * data[4]) + (o.data[6] * data[5]) + (o.data[10] * data[6]);
            result.data[10] = (o.data[2] * data[8]) + (o.data[6] * data[9]) + (o.data[10] * data[10]);

            result.data[3]	= (o.data[3] * data[0]) + (o.data[7] * data[1]) + (o.data[11] * data[2]) + data[3];
            result.data[7]	= (o.data[3] * data[4]) + (o.data[7] * data[5]) + (o.data[11] * data[6]) + data[7];
            result.data[11] = (o.data[3] * data[8]) + (o.data[7] * data[9]) + (o.data[11] * data[10]) + data[11];

            return result;
        }
		
        /*
         * Transform the given vector by this matrix.
         */
        Vector3 Transform(const Vector3& vector) const
        {
            return (*this) * vector;
        }

        /*
         * Transform the given vector by this matrix.
         */
        Vector3 operator*(const Vector3& vector) const
        {
            return Vector3(
                vector.x * data[0] +
                vector.y * data[1] +
                vector.z * data[2] + data[3],

                vector.x * data[4] +
                vector.y * data[5] +
                vector.z * data[6] + data[7],

                vector.x * data[8] +
                vector.y * data[9] +
                vector.z * data[10] + data[11]
            );
        }

        /*
         * Returns the determinant of the matrix.
         */
        float GetDeterminant() const;

        /*
         * Sets this matrix to be the inverse of the given matrix.
         */
        void SetInverse(const Matrix4& m);

        /*
		 * Returns a new matrix containing the inverse of this matrix. 
		 */
        Matrix4 Inverse() const
        {
            Matrix4 result;
            result.SetInverse(*this);
            return result;
        }

        /*!
         * Inverts the matrix.
         */
        void Invert()
        {
            SetInverse(*this);
        }

        /*!
         * Transform the given direction vector by this matrix.
         *
         * note: When a direction is converted between frames of
         * reference, there is no translation required.
         */
        Vector3 TransformDirection(const Vector3& vector) const
        {
            return Vector3(
                vector.x * data[0] +
                vector.y * data[1] +
                vector.z * data[2],

                vector.x * data[4] +
                vector.y * data[5] +
                vector.z * data[6],

                vector.x * data[8] +
                vector.y * data[9] +
                vector.z * data[10]
            );
        }

        /*!
         * Transform the given direction vector by the
         * transformational inverse of this matrix.
         *
         * note: This function relies on the fact that the inverse of
         * a pure rotation matrix is its transpose.
         *
         * note: When a direction is converted between frames of
         * reference, there is no translation required.
         */
        Vector3 TransformInverseDirection(const Vector3& vector) const
        {
            return Vector3(
                vector.x * data[0] +
                vector.y * data[4] +
                vector.z * data[8],

                vector.x * data[1] +
                vector.y * data[5] +
                vector.z * data[9],

                vector.x * data[2] +
                vector.y * data[6] +
                vector.z * data[10]
            );
        }

        /*!
         * Transform the given vector by the transformational inverse
         * of this matrix.
         *
         * \note This function relies on the fact that the inverse of
         * a pure rotation matrix is its transpose.
         */
        Vector3 TransformInverse(const Vector3& vector) const
        {
            Vector3 tmp = vector;
            tmp.x -= data[3];
            tmp.y -= data[7];
            tmp.z -= data[11];

            return Vector3(
                tmp.x * data[0] +
                tmp.y * data[4] +
                tmp.z * data[8],

                tmp.x * data[1] +
                tmp.y * data[5] +
                tmp.z * data[9],

                tmp.x * data[2] +
                tmp.y * data[6] +
                tmp.z * data[10]
            );
        }

        /*!
         * Gets a vector representing one axis (i.e. one column) in the matrix.
         */
        Vector3 GetAxisVector(size_t i) const
        {
            return Vector3(data[i], data[i + 4], data[i + 8]);
        }

        /*!
         * Sets this matrix to be the rotation matrix corresponding to
         * the given quaternion.
         */
        void SetOrientationAndPos(const Quaternion& q, const Vector3& pos)
        {
            data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
            data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
            data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
            data[3] = pos.x;

            data[4] = 2 * q.i * q.j - 2 * q.k * q.r;
            data[5] = 1 - (2 * q.i * q.i  + 2 * q.k * q.k);
            data[6] = 2 * q.j * q.k + 2 * q.i * q.r;
            data[7] = pos.y;

            data[8] = 2 * q.i * q.k + 2 * q.j * q.r;
            data[9] = 2 * q.j * q.k - 2 * q.i * q.r;
            data[10] = 1 - (2 * q.i * q.i  + 2 * q.j * q.j);
            data[11] = pos.z;
        }
    };

    /*!
     * Holds a 3x3 matrix, useful to represent an inertia tensor.
     */
    class Matrix3
    {
    public:

        /*!
         * Holds the tensor matrix data in array form.
         */
        float data[9];

        /*!
         * Creates a new matrix.
         */
        Matrix3()
        {
            data[0] = data[1] = data[2] = data[3] = data[4] = data[5] =
                data[6] = data[7] = data[8] = 0;
        }

        /*!
         * Creates a new matrix with the given three vectors 
		 * as columns.
         */
        Matrix3(const Vector3& compOne, const Vector3& compTwo,
            const Vector3& compThree)
        {
            SetComponents(compOne, compTwo, compThree);
        }

        /*!
         * Creates a new matrix with the given coefficients.
         */
        Matrix3(float c0, float c1, float c2, float c3, float c4, float c5,
            float c6, float c7, float c8)
        {
            data[0] = c0; data[1] = c1; data[2] = c2;
            data[3] = c3; data[4] = c4; data[5] = c5;
            data[6] = c6; data[7] = c7; data[8] = c8;
        }

		const static Matrix3 IDENTITY;

        /*!
         * Sets the matrix to be a diagonal matrix with the given
         * values along the leading diagonal.
         */
        void SetDiagonal(float a, float b, float c)
        {
            SetInertiaTensorCoeffs(a, b, c);
        }

        /*!
         * Sets the value of the matrix from inertia tensor values.
         */
        void SetInertiaTensorCoeffs(float ix, float iy, float iz,
            float ixy = 0, float ixz = 0, float iyz = 0)
        {
            data[0] = ix;
            data[1] = data[3] = -ixy;
            data[2] = data[6] = -ixz;
            data[4] = iy;
            data[5] = data[7] = -iyz;
            data[8] = iz;
        }

        /*!
         * Sets the value of the matrix as an inertia tensor of
         * a rectangular block aligned with the body's coordinate
         * system with the given axis half-sizes and mass.
         */
        void SetBlockInertiaTensor(const Vector3& halfSizes, float mass)
        {
            Vector3 squares = halfSizes.ComponentProduct(halfSizes);
            SetInertiaTensorCoeffs(	0.3f * mass * (squares.y + squares.z),
									0.3f * mass * (squares.x + squares.z),
									0.3f * mass * (squares.x + squares.y));
        }

        /*!
         * Sets the matrix to be a skew symmetric matrix based on
         * the given vector.
         */
        void SetSkewSymmetric(const Vector3 vector)
        {
            data[0] = data[4] = data[8] = 0;
            data[1] = -vector.z;
            data[2] = vector.y;
            data[3] = vector.z;
            data[5] = -vector.x;
            data[6] = -vector.y;
            data[7] = vector.x;
        }

        /*!
         * Sets the matrix values from the given three vector components.
         * These are arranged as the three columns of the vector.
         */
        void SetComponents(const Vector3& compOne, const Vector3& compTwo,
            const Vector3& compThree)
        {
            data[0] = compOne.x;
            data[1] = compTwo.x;
            data[2] = compThree.x;
            data[3] = compOne.y;
            data[4] = compTwo.y;
            data[5] = compThree.y;
            data[6] = compOne.z;
            data[7] = compTwo.z;
            data[8] = compThree.z;
        }

		/*!
         * Transform the given vector by this matrix.
         */
        Vector3 Transform(const Vector3& vector) const
        {
            return (*this) * vector;
        }

        /*!
         * Transform the given vector by this matrix.
         */
        Vector3 operator*(const Vector3& vector) const
        {
            return Vector3(
                vector.x * data[0] + vector.y * data[1] + vector.z * data[2],
                vector.x * data[3] + vector.y * data[4] + vector.z * data[5],
                vector.x * data[6] + vector.y * data[7] + vector.z * data[8]
            );
        }

        /*!
         * Transform the given vector by the transpose of this matrix.
         */
        Vector3 TransformTranspose(const Vector3& vector) const
        {
            return Vector3(
                vector.x * data[0] + vector.y * data[3] + vector.z * data[6],
                vector.x * data[1] + vector.y * data[4] + vector.z * data[7],
                vector.x * data[2] + vector.y * data[5] + vector.z * data[8]
            );
        }

        /*!
         * Gets a vector representing one row in the matrix.
         */
        Vector3 GetRowVector(size_t i) const
        {
            return Vector3(data[i * 3], data[i * 3 + 1], data[i * 3 + 2]);
        }

        /*!
         * Gets a vector representing one axis (i.e. one column) in the matrix.
         */
        Vector3 GetAxisVector(size_t i) const
        {
            return Vector3(data[i], data[i + 3], data[i + 6]);
        }

        /*!
         * Sets the matrix to be the inverse of the given matrix.
         */
        void SetInverse(const Matrix3& m)
        {
            float t4		= m.data[0] * m.data[4];
            float t6		= m.data[0] * m.data[5];
            float t8		= m.data[1] * m.data[3];
            float t10	= m.data[2] * m.data[3];
            float t12	= m.data[1] * m.data[6];
            float t14	= m.data[2] * m.data[6];

            // Calculate the determinant
            float t16 = (t4 * m.data[8] - t6 * m.data[7] - t8 * m.data[8]+
                        t10 * m.data[7] + t12 * m.data[5] - t14 * m.data[4]);

            // Make sure the determinant is non-zero.
            if (t16 == (float)0.0f) return;
            float t17 = 1 / t16;

            data[0] = (m.data[4] * m.data[8] - m.data[5] * m.data[7]) * t17;
            data[1] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7]) * t17;
            data[2] = (m.data[1] * m.data[5] - m.data[2] * m.data[4]) * t17;
            data[3] = -(m.data[3] * m.data[8]-m.data[5] * m.data[6]) * t17;
            data[4] = (m.data[0] * m.data[8] - t14) * t17;
            data[5] = -(t6 - t10) * t17;
            data[6] = (m.data[3] * m.data[7] - m.data[4] * m.data[6]) * t17;
            data[7] = -(m.data[0] * m.data[7] - t12) * t17;
            data[8] = (t4 - t8) * t17;
        }

        /*! 
		 * Returns a new matrix containing the inverse of this matrix. 
		 */
        Matrix3 Inverse() const
        {
            Matrix3 result;
            result.SetInverse(*this);
            return result;
        }

        /*!
         * Inverts the matrix.
         */
        void Invert()
        {
            SetInverse(*this);
        }

        /*!
         * Sets the matrix to be the transpose of the given matrix.
         */
        void SetTranspose(const Matrix3& m)
        {
            data[0] = m.data[0];
            data[1] = m.data[3];
            data[2] = m.data[6];
            data[3] = m.data[1];
            data[4] = m.data[4];
            data[5] = m.data[7];
            data[6] = m.data[2];
            data[7] = m.data[5];
            data[8] = m.data[8];
        }

        /*! 
		 * Returns a new matrix containing the transpose of this matrix. 
		 */
        Matrix3 Transpose() const
        {
            Matrix3 result;
            result.SetTranspose(*this);
            return result;
        }

        /*!
         * Returns a matrix which is this matrix multiplied by the given
         * other matrix.
         */
        Matrix3 operator*(const Matrix3& o) const
        {
            return Matrix3(
                data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
                data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
                data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],

                data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
                data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
                data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],

                data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
                data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
                data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8]
                );
        }

        /*!
         * Multiplies this matrix in place by the given other matrix.
         */
        void operator*=(const Matrix3& o)
        {
            float t1;
            float t2;
            float t3;

            t1 = data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6];
            t2 = data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7];
            t3 = data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8];
            data[0] = t1;
            data[1] = t2;
            data[2] = t3;

            t1 = data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6];
            t2 = data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7];
            t3 = data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8];
            data[3] = t1;
            data[4] = t2;
            data[5] = t3;

            t1 = data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6];
            t2 = data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7];
            t3 = data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8];
            data[6] = t1;
            data[7] = t2;
            data[8] = t3;
        }

        /*!
         * Multiplies this matrix in place by the given scalar.
         */
        void operator*=(const float scalar)
        {
            data[0] *= scalar; data[1] *= scalar; data[2] *= scalar;
            data[3] *= scalar; data[4] *= scalar; data[5] *= scalar;
            data[6] *= scalar; data[7] *= scalar; data[8] *= scalar;
        }

        /*!
         * Does a component-wise addition of this matrix and the given
         * matrix.
         */
        void operator+=(const Matrix3& o)
        {
            data[0] += o.data[0]; data[1] += o.data[1]; data[2] += o.data[2];
            data[3] += o.data[3]; data[4] += o.data[4]; data[5] += o.data[5];
            data[6] += o.data[6]; data[7] += o.data[7]; data[8] += o.data[8];
        }

        /*!
         * Sets this matrix to be the rotation matrix corresponding to
         * the given quaternion.
         */
        void SetOrientation(const Quaternion& q)
        {
            data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
            data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
            data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
            data[3] = 2 * q.i * q.j - 2 * q.k * q.r;
            data[4] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
            data[5] = 2 * q.j * q.k + 2 * q.i * q.r;
            data[6] = 2 * q.i * q.k + 2 * q.j * q.r;
            data[7] = 2 * q.j * q.k - 2 * q.i * q.r;
            data[8] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
        }

        /*!
         * Interpolates a couple of matrices.
         */
        static Matrix3 LinearInterpolate(const Matrix3& a, const Matrix3& b, float prop);
    };

	#pragma region Sleep epsilon

	/*!
     * Holds the value for energy under which a body will be put to
     * sleep. This is a global value for the whole solution.
     */
    extern float sleepEpsilon;

    /*!
     * Sets the current sleep epsilon value.
     */
    void SetSleepEpsilon(float value);

    /*!
     * Gets the current value of the sleep epsilon parameter.
     */
    float GetSleepEpsilon();

	#pragma endregion

	#pragma region Integration step

	/*!
     * Integration step.
     */
	extern float integrationStep;

	/*!
     * Sets the current integration step value.
     */
    void SetIntegrationStep(float value);

    /*!
     * Gets the current value of the integration step.
     */
    float GetIntegrationStep();

	#pragma endregion	
}
