#include "Forces.h"

using namespace physic;

void ForceRegistry::updateForces(float duration)
{
	Registry::iterator i = registrations.begin();
	for (; i != registrations.end(); i++)
	{
		i->fg->updateForce(i->body, duration);
	}
}

void ForceRegistry::add(RigidBody *body, ForceGenerator *fg)
{
	ForceRegistry::ForceRegistration registration;
	registration.body = body;
	registration.fg = fg;
	registrations.push_back(registration);
}


Gravity::Gravity(const Vector3& gravity)
	: gravity(gravity)
{
}

void Gravity::updateForce(RigidBody* body, float duration)
{
	// Check that we do not have infinite mass
	if (!body->hasFiniteMass()) return;

	// Apply the mass-scaled force to the body
	body->addForce(gravity * body->getMass());
}