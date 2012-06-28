#pragma once
#include "Core.h"


namespace physic{


class RigidBody
	{

	protected:

		float inverseMass;

		Matrix3 inverseInertiaTensor;

		float linearDamping;

		float angularDamping;

		Vector3 position;

		Quaternion orientation;

		Vector3 velocity;

		Vector3 rotation;

		Matrix3 inverseInertiaTensorWorld;


		float motion;


		bool isAwake;


		bool canSleep;

		Matrix4 transformMatrix;


		Vector3 forceAccum;


		Vector3 torqueAccum;


		Vector3 acceleration;


		Vector3 lastFrameAcceleration;


	public:

		void calculateDerivedData();

		void integrate(float duration);


		void setMass(const float mass);

		/**
		* Gets the mass of the rigid body.
		*/
		float getMass() const;

		/**
		* Sets the inverse mass of the rigid body.
		*/
		void setInverseMass(const float inverseMass);

		/**
		* Gets the inverse mass of the rigid body.
		*
		*/
		float getInverseMass() const;

		/**
		* Returns true if the mass of the body is not-infinite.
		*/
		bool hasFiniteMass() const;

		/**
		* Sets the intertia tensor for the rigid body.
		*/
		void setInertiaTensor(const Matrix3 &inertiaTensor);

		/**
		* Copies the current inertia tensor of the rigid body into
		* the given matrix.
		*
		* inertiaTensor A pointer to a matrix to hold the
		* current inertia tensor of the rigid body. The inertia
		* tensor is expressed in the rigid body's local space.
		*/
		void getInertiaTensor(Matrix3 *inertiaTensor) const;

		/**
		* Gets a copy of the current inertia tensor of the rigid body.
		*
		* return A new matrix containing the current intertia
		* tensor. The inertia tensor is expressed in the rigid body's
		* local space.
		*/
		Matrix3 getInertiaTensor() const;

		/**
		* Copies the current inertia tensor of the rigid body into
		* the given matrix.
		*
		* inertiaTensor A pointer to a matrix to hold the
		* current inertia tensor of the rigid body. The inertia
		* tensor is expressed in world space.
		*/
		void getInertiaTensorWorld(Matrix3 *inertiaTensor) const;

		/**
		* Gets a copy of the current inertia tensor of the rigid body.
		*
		* return A new matrix containing the current intertia
		* tensor. The inertia tensor is expressed in world space.
		*/
		Matrix3 getInertiaTensorWorld() const;

		/**
		* Sets the inverse intertia tensor for the rigid body.
		*/
		void setInverseInertiaTensor(const Matrix3 &inverseInertiaTensor);

		/**
		* Copies the current inverse inertia tensor of the rigid body
		* into the given matrix.
		*/
		void getInverseInertiaTensor(Matrix3 *inverseInertiaTensor) const;

		/**
		* Gets a copy of the current inverse inertia tensor of the
		* rigid body.
		*
		* return A new matrix containing the current inverse
		* intertia tensor. The inertia tensor is expressed in the
		* rigid body's local space.
		*/
		Matrix3 getInverseInertiaTensor() const;

		/**
		* Copies the current inverse inertia tensor of the rigid body
		* into the given matrix.
		*
		* inverseInertiaTensor A pointer to a matrix to hold
		* the current inverse inertia tensor of the rigid body. The
		* inertia tensor is expressed in world space.
		*/
		void getInverseInertiaTensorWorld(Matrix3 *inverseInertiaTensor) const;

		/**
		* Gets a copy of the current inverse inertia tensor of the
		* rigid body.
		*
		* return A new matrix containing the current inverse
		* intertia tensor. The inertia tensor is expressed in world
		* space.
		*/
		Matrix3 getInverseInertiaTensorWorld() const;

		/**
		* Sets both linear and angular damping in one function call.
		*/
		void setDamping(const float linearDamping, const float angularDamping);

		/**
		* Sets the linear damping for the rigid body.
		*/
		void setLinearDamping(const float linearDamping);

		/**
		* Gets the current linear damping value.
		*/
		float getLinearDamping() const;

		/**
		* Sets the angular damping for the rigid body.
		*
		*/
		void setAngularDamping(const float angularDamping);

		/**
		* Gets the current angular damping value.
		*
		* return The current angular damping value.
		*/
		float getAngularDamping() const;

		/**
		* Sets the position of the rigid body.
		*/
		void setPosition(const Vector3 &position);

		/**
		* Sets the position of the rigid body by component.
		*/
		void setPosition(const float x, const float y, const float z);

		/**
		* Fills the given vector with the position of the rigid body.
		*/
		void getPosition(Vector3 *position) const;

		/**
		* Gets the position of the rigid body.
		*
		*/
		Vector3 getPosition() const;

		/**
		* Sets the orientation of the rigid body.
		* note The given orientation does not need to be normalised,
		* and can be zero. This function automatically constructs a
		* valid rotation quaternion with (0,0,0,0) mapping to
		* (1,0,0,0).
		*/
		void setOrientation(const Quaternion &orientation);

		/**
		* Sets the orientation of the rigid body by component.
		*
		* note The given orientation does not need to be normalised,
		* and can be zero. This function automatically constructs a
		* valid rotation quaternion with (0,0,0,0) mapping to
		* (1,0,0,0).
		*/
		void setOrientation(const float r, const float i,
			const float j, const float k);

		/**
		* Fills the given quaternion with the current value of the
		* rigid body's orientation.
		*
		* orientation A pointer to a quaternion to receive the
		* orientation data.
		*/
		void getOrientation(Quaternion *orientation) const;

		/**
		* Gets the orientation of the rigid body.
		*
		* return The orientation of the rigid body.
		*/
		Quaternion getOrientation() const;

		/**
		* Fills the given matrix with a transformation representing
		* the rigid body's orientation.
		*
		* Transforming a direction vector by this matrix turns
		* it from the body's local space to world space.
		*
		* matrix A pointer to the matrix to fill.
		*/
		void getOrientation(Matrix3 *matrix) const;

		/**
		* Fills the given matrix data structure with a transformation
		* representing the rigid body's orientation.
		*
		* note Transforming a direction vector by this matrix turns
		* it from the body's local space to world space.
		*
		*  matrix A pointer to the matrix to fill.
		*/
		void getOrientation(float matrix[9]) const;

		/**
		* Fills the given matrix with a transformation representing
		* the rigid body's position and orientation.
		*
		* @note Transforming a vector by this matrix turns it from
		* the body's local space to world space.
		*
		* @param transform A pointer to the matrix to fill.
		*/
		void getTransform(Matrix4 *transform) const;

		/**
		* Fills the given matrix data structure with a
		* transformation representing the rigid body's position and
		* orientation.
		*
		* note Transforming a vector by this matrix turns it from
		* the body's local space to world space.
		*
		* matrix A pointer to the matrix to fill.
		*/
		void getTransform(float matrix[16]) const;

		/**
		* Fills the given matrix data structure with a
		* transformation representing the rigid body's position and
		* orientation. The matrix is transposed from that returned
		* by getTransform. This call returns a matrix suitable
		* for applying as an OpenGL transform.
		*
		* note Transforming a vector by this matrix turns it from
		* the body's local space to world space.
		*
		* matrix A pointer to the matrix to fill.
		*/
		void getGLTransform(float matrix[16]) const;

		/**
		* Gets a transformation representing the rigid body's
		* position and orientation.
		*/
		Matrix4 getTransform() const;

		/**
		* Converts the given point from world space into the body's
		* local space.
		*/
		Vector3 getPointInLocalSpace(const Vector3 &point) const;

		/**
		* Converts the given point from world space into the body's
		* local space.
		*/
		Vector3 getPointInWorldSpace(const Vector3 &point) const;

		/**
		* Converts the given direction from world space into the
		* body's local space.
		*/
		Vector3 getDirectionInLocalSpace(const Vector3 &direction) const;

		/**
		* Converts the given direction from world space into the
		* body's local space.
		*/
		Vector3 getDirectionInWorldSpace(const Vector3 &direction) const;

		/**
		* Sets the velocity of the rigid body.
		* 
		* velocity The new velocity of the rigid body. The
		* velocity is given in world space.
		*/
		void setVelocity(const Vector3 &velocity);

		/**
		* Sets the velocity of the rigid body by component. The
		* velocity is given in world space.
		*/
		void setVelocity(const float x, const float y, const float z);

		/**
		* Fills the given vector with the velocity of the rigid body.
		*
		* velocity A pointer to a vector into which to write
		* the velocity. The velocity is given in world local space.
		*/
		void getVelocity(Vector3 *velocity) const;

		/**
		* Gets the velocity of the rigid body.
		*
		* @return The velocity of the rigid body. The velocity is
		* given in world local space.
		*/
		Vector3 getVelocity() const;

		/**
		* Applies the given change in velocity.
		*/
		void addVelocity(const Vector3 &deltaVelocity);

		/**
		* Sets the rotation of the rigid body.
		*
		* rotation The new rotation of the rigid body. The
		* rotation is given in world space.
		*/
		void setRotation(const Vector3 &rotation);

		/**
		* Sets the rotation of the rigid body by component. The
		* rotation is given in world space.
		*/
		void setRotation(const float x, const float y, const float z);

		/**
		* Fills the given vector with the rotation of the rigid body.
		*
		* rotation A pointer to a vector into which to write
		* the rotation. The rotation is given in world local space.
		*/
		void getRotation(Vector3 *rotation) const;

		/**
		* Gets the rotation of the rigid body.
		*
		* return The rotation of the rigid body. The rotation is
		* given in world local space.
		*/
		Vector3 getRotation() const;

		/**
		* Applies the given change in rotation.
		*/
		void addRotation(const Vector3 &deltaRotation);

		/**
		* Returns true if the body is awake and responding to
		* integration.
		*/
		bool getAwake() const
		{
			return isAwake;
		}


		/**
		* Sets the awake state of the body. If the body is set to be
		* not awake, then its velocities are also cancelled, since
		* a moving body that is not awake can cause problems in the
		* simulation.
		*/
		void setAwake(const bool awake=true);


		/**
		* Returns true if the body is allowed to go to sleep at
		* any time.
		*/
		bool getCanSleep() const
		{
			return canSleep;
		}

		/**
		* Sets whether the body is ever allowed to go to sleep. Bodies
		* under the player's control, or for which the set of
		* transient forces applied each frame are not predictable,
		* should be kept awake.
		*/
		void setCanSleep(const bool canSleep=true);


		/**
		* Fills the given vector with the current accumulated value
		* for linear acceleration. The acceleration accumulators
		* are set during the integration step. They can be read to
		* determine the rigid body's acceleration over the last
		* integration step. The linear acceleration is given in world
		* space.

		*/
		void getLastFrameAcceleration(Vector3 *linearAcceleration) const;

		/**
		* Gets the current accumulated value for linear
		* acceleration. The acceleration accumulators are set during
		* the integration step. They can be read to determine the
		* rigid body's acceleration over the last integration
		* step. The linear acceleration is given in world space.
		*
		*/
		Vector3 getLastFrameAcceleration() const;

		/**
		* Clears the forces and torques in the accumulators. This will
		* be called automatically after each intergration step.
		*/
		void clearAccumulators();

		/**
		* Adds the given force to centre of mass of the rigid body.
		* The force is expressed in world-coordinates.

		*/
		void addForce(const Vector3 &force);

		/**
		* Adds the given force to the given point on the rigid body.
		* Both the force and the application point are given in world
		* space. Because the force is not applied at the centre of
		* mass, it may be split into both a force and torque.
		*/
		void addForceAtPoint(const Vector3 &force, 
			const Vector3 &point);

		/**
		* Adds the given force to the given point on the rigid body.
		* The direction of the force is given in world coordinates,
		* but the application point is given in body space. This is
		* useful for spring forces, or other forces fixed to the
		* body.
		*/
		void addForceAtBodyPoint(const Vector3 &force, 
			const Vector3 &point);


		/**
		* Adds the given torque to the rigid body.
		* The force is expressed in world-coordinates.
		*/
		void addTorque(const Vector3 &torque);

		/**
		* Sets the constant acceleration of the rigid body.
		*/
		void setAcceleration(const Vector3 &acceleration);

		/**
		* Sets the constant acceleration of the rigid body by component.
		*/
		void setAcceleration(const float x, const float y, const float z);

		/**
		* Fills the given vector with the acceleration of the rigid body.
		*/
		void getAcceleration(Vector3 *acceleration) const;

		/**
		* Gets the acceleration of the rigid body.
		*/
		Vector3 getAcceleration() const;
	};
}