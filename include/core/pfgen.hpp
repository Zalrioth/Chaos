#pragma once
#ifndef PFGEN_HPP_
#define PFGEN_HPP_

#include "core/core.hpp"
#include "core/particle.hpp"
#include <vector>

namespace chaos {
class ParticleForceGenerator {
public:
    virtual void updateForce(Particle* particle, real duration) = 0;
};

class ParticleForceRegistry {
protected:
    struct ParticleForceRegistration {
        Particle* particle;
        ParticleForceGenerator* fg;
    };

    typedef std::vector<ParticleForceRegistration> Registry;
    Registry registrations;

public:
    void add(Particle* particle, ParticleForceGenerator* fg);
    void remove(Particle* particle, ParticleForceGenerator* fg);
    void clear();
    void updateForces(real duration);
};

class ParticleGravity : public ParticleForceGenerator {
    Vector3 gravity;

public:
    ParticleGravity(const Vector3& gravity);
    virtual void updateForce(Particle* particle, real duration);
};

class ParticleDrag : public ParticleForceGenerator {
    real k1;
    real k2;

public:
    ParticleDrag(real k1, real k2);
    virtual void updateForce(Particle* particle, real duration);
};

class ParticleSpring : public ParticleForceGenerator {
    Particle* other;
    real springConstant;
    real restLength;

public:
    ParticleSpring(Particle* other, real springConstant, real restLength);
    virtual void updateForce(Particle* particle, real duration);
};
}

#endif // PFGEN_HPP_