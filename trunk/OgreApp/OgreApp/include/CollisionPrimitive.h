#pragma once
#include "Contacts.h"

namespace physic{

	 // Forward declarations of primitive friends
    class IntersectionTests;
    class CollisionDetector;

    /**
     * Represents a primitive to detect collisions against.
     */
    class CollisionPrimitive
    {
    public:

        friend IntersectionTests;
        friend CollisionDetector;

        RigidBody * body;

        /**
         * The offset of this primitive from the given rigid body.
         */
        Matrix4 offset;

        /**
         * Calculates the internals for the primitive.
         */
		void CollisionPrimitive::calculateInternals()
		{
			transform = body->getTransform() * offset;
		}

        /**
         * This is a convenience function to allow access to the
         * axis vectors in the transform for this primitive.
         */
        Vector3 getAxis(unsigned index) const
        {
            return transform.GetAxisVector(index);
        }

 
        const Matrix4& getTransform() const
        {
            return transform;
        }


    protected:

        Matrix4 transform;
    };



}