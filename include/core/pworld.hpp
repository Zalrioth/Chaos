#pragma once
#ifndef PWORLD_HPP_
#define PWORLD_HPP_

#include "core/pfgen.hpp"
#include "core/plinks.hpp"
#include <cstddef>

namespace chaos {
class ParticleWorld {
public:
    typedef std::vector<Particle*> Particles;
    typedef std::vector<ParticleContactGenerator*> ContactGenerators;

protected:
    Particles particles;
    bool calculateIterations;
    ParticleForceRegistry registry;
    ParticleContactResolver resolver;
    ContactGenerators contactGenerators;
    ParticleContact* contacts;
    unsigned maxContacts;

public:
    ParticleWorld(unsigned maxContacts, unsigned iterations = 0);
    ~ParticleWorld();
    unsigned generateContacts();
    void integrate(real duration);
    void runPhysics(real duration);
    void startFrame();
    Particles& getParticles();
    ContactGenerators& getContactGenerators();
    ParticleForceRegistry& getForceRegistry();
};

class GroundContacts : public chaos::ParticleContactGenerator {
    chaos::ParticleWorld::Particles* particles;

public:
    void init(chaos::ParticleWorld::Particles* particles);
    virtual unsigned addContact(chaos::ParticleContact* contact, unsigned limit) const;
};
}

#endif // PWORLD_HPP_