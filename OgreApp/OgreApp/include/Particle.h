#ifndef PARTICLE_H
#define PARTICLE_H

#include "Core.h"

namespace physic{


	class Particle{

	protected:

		float inverseMass;
		
		float damping;
		
		Vector3 position;
		
		Vector3 velocity;

		Vector3 forceAccum;

		Vector3 acceleration;

	public:

		void integrate(const float dt);

		void setMass(const float mass);

		float getMass()const;

		void setInverseMass(const float imass);

		float getInverseMass()const;

		bool hasFiniteMass() const;

        void setDamping(const float damping);

        float getDamping() const;

        void setPosition(const Vector3 &position);

        void setPosition(const float x, const float y, const float z);

        void getPosition(Vector3 *position) const;

        Vector3 getPosition() const;

        void setVelocity(const Vector3 &velocity);

        void setVelocity(const float x, const float y, const float z);

		void getVelocity(Vector3 *velocity) const;

        Vector3 getVelocity() const;

        void setAcceleration(const Vector3 &acceleration);

        void setAcceleration(const float x, const float y, const float z);

        void getAcceleration(Vector3 *acceleration) const;

        Vector3 getAcceleration() const;

        void clearAccumulator();       

        void addForce(const Vector3 &force);


	};



}


#endif