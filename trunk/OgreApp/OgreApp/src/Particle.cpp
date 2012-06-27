#include <assert.h>
#include "Particle.h"

using namespace physic;


void Particle::integrate(const float dt){

	if (inverseMass <= 0.0f) return;

	assert(dt > 0.0);

	// Update linear position.
	position.AddScaledVector(velocity, dt);
	
	// Work out the acceleration from the force
	// < ParticleAccum
	// (we'll add to this vector when we come to generate forces)
	// > ParticleAccum
	Vector3 resultingAcc = acceleration;

	// < ParticleIntegrate
	resultingAcc.AddScaledVector(forceAccum, inverseMass);

	// > ParticleIntegrate

	// Update linear velocity from the acceleration.
	velocity.AddScaledVector(resultingAcc, dt);

	// Impose drag.
	velocity *= pow(damping, dt);

	// Clear the forces.
	clearAccumulator();

}

void Particle::setMass(const float mass)
{
	assert(mass != 0);
	Particle::inverseMass = ((float)1.0)/mass;
}

float Particle::getMass() const
{
	if (inverseMass == 0) {
		return FLT_MAX;
	} else {
		return ((float)1.0)/inverseMass;
	}
}

void Particle::setInverseMass(const float inverseMass)
{
	Particle::inverseMass = inverseMass;
}

float Particle::getInverseMass() const
{
	return inverseMass;
}

bool Particle::hasFiniteMass() const
{
	return inverseMass >= 0.0f;
}

void Particle::setDamping(const float damping)
{
	Particle::damping = damping;
}

float Particle::getDamping() const
{
	return damping;
}

void Particle::setPosition(const Vector3 &position)
{
	Particle::position = position;
}

void Particle::setPosition(const float x, const float y, const float z)
{
	position.x = x;
	position.y = y;
	position.z = z;
}

void Particle::getPosition(Vector3 *position) const
{
	*position = Particle::position;
}

Vector3 Particle::getPosition() const
{
	return position;
}

void Particle::setVelocity(const Vector3 &velocity)
{
	Particle::velocity = velocity;
}

void Particle::setVelocity(const float x, const float y, const float z)
{
	velocity.x = x;
	velocity.y = y;
	velocity.z = z;
}

void Particle::getVelocity(Vector3 *velocity) const
{
	*velocity = Particle::velocity;
}

Vector3 Particle::getVelocity() const
{
	return velocity;
}

void Particle::setAcceleration(const Vector3 &acceleration)
{
	Particle::acceleration = acceleration;
}

void Particle::setAcceleration(const float x, const float y, const float z)
{
	acceleration.x = x;
	acceleration.y = y;
	acceleration.z = z;
}

void Particle::getAcceleration(Vector3 *acceleration) const
{
	*acceleration = Particle::acceleration;
}

Vector3 Particle::getAcceleration() const
{
	return acceleration;
}


void Particle::clearAccumulator()
{
	forceAccum.Clear();
}



void Particle::addForce(const Vector3 &force)
{
	forceAccum += force;
}