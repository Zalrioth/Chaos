#include <core/pworld.hpp>

using namespace chaos;

ParticleWorld::ParticleWorld(unsigned maxContacts, unsigned iterations)
    : resolver(iterations)
    , maxContacts(maxContacts)
{
    contacts = new ParticleContact[maxContacts];
    calculateIterations = (iterations == 0);
}

ParticleWorld::~ParticleWorld()
{
    delete[] contacts;
}

void ParticleWorld::startFrame()
{
    for (Particles::iterator p = particles.begin(); p != particles.end(); p++)
        (*p)->clearAccumulator();
}

unsigned ParticleWorld::generateContacts()
{
    unsigned limit = maxContacts;
    ParticleContact* nextContact = contacts;

    for (ContactGenerators::iterator g = contactGenerators.begin(); g != contactGenerators.end(); g++) {
        unsigned used = (*g)->addContact(nextContact, limit);
        limit -= used;
        nextContact += used;

        if (limit <= 0)
            break;
    }

    return maxContacts - limit;
}

void ParticleWorld::integrate(real duration)
{
    for (Particles::iterator p = particles.begin(); p != particles.end(); p++) {
        (*p)->integrate(duration);
    }
}

void ParticleWorld::runPhysics(real duration)
{
    registry.updateForces(duration);

    integrate(duration);

    unsigned usedContacts = generateContacts();

    if (usedContacts) {
        if (calculateIterations)
            resolver.setIterations(usedContacts * 2);
        resolver.resolveContacts(contacts, usedContacts, duration);
    }
}

ParticleWorld::Particles& ParticleWorld::getParticles()
{
    return particles;
}

ParticleWorld::ContactGenerators& ParticleWorld::getContactGenerators()
{
    return contactGenerators;
}

ParticleForceRegistry& ParticleWorld::getForceRegistry()
{
    return registry;
}

void GroundContacts::init(chaos::ParticleWorld::Particles* particles)
{
    GroundContacts::particles = particles;
}

unsigned GroundContacts::addContact(chaos::ParticleContact* contact, unsigned limit) const
{
    unsigned count = 0;
    for (chaos::ParticleWorld::Particles::iterator p = particles->begin();
         p != particles->end();
         p++) {
        chaos::real y = (*p)->getPosition().y;
        if (y < 0.0f) {
            contact->contactNormal = chaos::Vector3::UP;
            contact->particle[0] = *p;
            contact->particle[1] = NULL;
            contact->penetration = -y;
            contact->restitution = 0.2f;
            contact++;
            count++;
        }

        if (count >= limit)
            return count;
    }
    return count;
}
