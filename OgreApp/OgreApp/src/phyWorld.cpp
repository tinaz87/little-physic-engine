#include "phyWorld.h"
#include <cstdlib>


using namespace physic;

phyWorld::phyWorld(unsigned maxContacts, unsigned iterations)
	:
firstBody(NULL),
	firstContactGen(NULL),
	resolver(iterations),
	maxContacts(maxContacts)
{
	contacts = new Contact[maxContacts];
	calculateIterations = (iterations == 0);
}

phyWorld::~phyWorld()
{
	delete[] contacts;
}

void phyWorld::startFrame()
{
	BodyRegistration *reg = firstBody;
	while (reg)
	{
		// Remove all forces from the accumulator
		reg->body->clearAccumulators();
		reg->body->calculateDerivedData();

		// Get the next registration
		reg = reg->next;
	}
}

unsigned phyWorld::generateContacts()
{
	unsigned limit = maxContacts;
	Contact *nextContact = contacts;

	ContactGenRegistration * reg = firstContactGen;
	while (reg)
	{
		unsigned used = reg->gen->addContact(nextContact, limit);
		limit -= used;
		nextContact += used;

		// We've run out of contacts to fill. This means we're missing
		// contacts.
		if (limit <= 0) break;

		reg = reg->next;
	}

	// Return the number of contacts used.
	return maxContacts - limit;
}

void phyWorld::runPhysics(float duration)
{
	// First apply the force generators
	//registry.updateForces(duration);

	// Then integrate the objects
	BodyRegistration *reg = firstBody;
	while (reg)
	{
		// Remove all forces from the accumulator
		reg->body->integrate(duration);

		// Get the next registration
		reg = reg->next;
	}

	// Generate contacts
	unsigned usedContacts = generateContacts();

	// And process them
	if (calculateIterations) resolver.setIterations(usedContacts * 4);
	resolver.resolveContacts(contacts, usedContacts, duration);
}