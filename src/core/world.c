
#include <core/world.hpp>

using namespace chaos;

World::World(unsigned maxContacts, unsigned iterations)
    : firstBody(NULL)
    , resolver(iterations)
    , firstContactGen(NULL)
    , maxContacts(maxContacts)
{
    contacts = new Contact[maxContacts];
    calculateIterations = (iterations == 0);
}

World::~World()
{
    delete[] contacts;
}

void World::startFrame()
{
    BodyRegistration* reg = firstBody;
    while (reg) {
        reg->body->clearAccumulators();
        reg->body->calculateDerivedData();

        reg = reg->next;
    }
}

unsigned World::generateContacts()
{
    unsigned limit = maxContacts;
    Contact* nextContact = contacts;

    ContactGenRegistration* reg = firstContactGen;
    while (reg) {
        unsigned used = reg->gen->addContact(nextContact, limit);
        limit -= used;
        nextContact += used;

        if (limit <= 0)
            break;

        reg = reg->next;
    }

    return maxContacts - limit;
}

void World::runPhysics(real duration)
{
    BodyRegistration* reg = firstBody;
    while (reg) {
        reg->body->integrate(duration);
        reg = reg->next;
    }

    unsigned usedContacts = generateContacts();

    if (calculateIterations)
        resolver.setIterations(usedContacts * 4);
    resolver.resolveContacts(contacts, usedContacts, duration);
}
