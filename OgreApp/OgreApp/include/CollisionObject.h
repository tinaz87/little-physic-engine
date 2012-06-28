#pragma once
#include "CollisionPrimitive.h"


namespace physic{


	class CollisionSphere : public CollisionPrimitive
    {
    public:
        /**
         * The radius of the sphere.
         */
        float radius;
    };


    class CollisionPlane
    {
    public:
        /**
         * The plane normal
         */
        Vector3 direction;

        /**
         * The distance of the plane from the origin.
         */
        float offset;
    };

    /**
     * Represents a rigid body that can be treated as an aligned bounding
     * box for collision detection.
     */
    class CollisionBox : public CollisionPrimitive
    {
    public:
        /**
         * Holds the half-sizes of the box along each of its local axes.
         */
        Vector3 halfSize;
    };

    /**
     * A wrapper class that holds fast intersection tests. These
     * can be used to drive the coarse collision detection system or
     * as an early out in the full collision tests below.
     */
    class IntersectionTests
    {
    public:


        static bool sphereAndSphere(
            const CollisionSphere &one,
            const CollisionSphere &two);

        static bool boxAndBox(
            const CollisionBox &one,
            const CollisionBox &two);

        static bool boxAndHalfSpace(
            const CollisionBox &box,
            const CollisionPlane &plane);
    };


    /**
     * A helper structure that contains information for the detector to use
     * in building its contact data.
     */
    struct CollisionData
    {
        /**
         * Holds the base of the collision data: the first contact
         * in the array. This is used so that the contact pointer (below)
         * can be incremented each time a contact is detected, while
         * this pointer points to the first contact found.
         */
        Contact *contactArray;

        /** Holds the contact array to write into. */
        Contact *contacts;

        /** Holds the maximum number of contacts the array can take. */
        int contactsLeft;

        /** Holds the number of contacts found so far. */
        unsigned contactCount;

        /** Holds the friction value to write into any collisions. */
        float friction;

        /** Holds the restitution value to write into any collisions. */
        float restitution;

        /**
         * Holds the collision tolerance, even uncolliding objects this
         * close should have collisions generated.
         */
        float tolerance;

        /**
         * Checks if there are more contacts available in the contact
         * data.
         */
        bool hasMoreContacts()
        {
            return contactsLeft > 0;
        }

        /**
         * Resets the data so that it has no used contacts recorded.
         */
        void reset(unsigned maxContacts)
        {
            contactsLeft = maxContacts;
            contactCount = 0;
            contacts = contactArray;
        }

        /**
         * Notifies the data that the given number of contacts have
         * been added.
         */
        void addContacts(unsigned count)
        {
            // Reduce the number of contacts remaining, add number used
            contactsLeft -= count;
            contactCount += count;

            // Move the array forward
            contacts += count;
        }
    };


	class CollisionDetector
    {
    public:

        static unsigned sphereAndTruePlane(
            const CollisionSphere &sphere,
            const CollisionPlane &plane,
            CollisionData *data
            );

        static unsigned sphereAndSphere(
            const CollisionSphere &one,
            const CollisionSphere &two,
            CollisionData *data
            );

        static unsigned boxAndHalfSpace(
            const CollisionBox &box,
            const CollisionPlane &plane,
            CollisionData *data
            );

        static unsigned boxAndSphere(
            const CollisionBox &box,
            const CollisionSphere &sphere,
            CollisionData *data
            );
    };

}