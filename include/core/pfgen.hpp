#pragma once
#ifndef PFGEN_HPP_
#define PFGEN_HPP_

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

class ParticleAnchoredSpring : public ParticleForceGenerator {
protected:
    Vector3* anchor;
    real springConstant;
    real restLength;

public:
    ParticleAnchoredSpring(Vector3* anchor, real springConstant, real restLength);
    virtual void updateForce(Particle* particle, real duration);
};

class ParticleBungee : public ParticleForceGenerator {
    Particle* other;
    real springConstant;
    real restLength;

public:
    ParticleBungee(Particle* other, real springConstant, real restLength);
    virtual void updateForce(Particle* particle, real duration);
};

class ParticleBoyancy : public ParticleForceGenerator {
    real maxDepth;
    real volume;
    real waterHeigh;
    real liquidDensity;

public:
    ParticleBouyancy(real maxDepth, real volume, real waterHieght, real liquidDensity = 1000.0f);
    virtual void updateForce(Particle* particle, real duration);
};

class ParticleFakeSpring : public ParticleForceGenerator {
    Vector3* anchor;
    real springConstant;
    real damping;

public:
    ParticleFakeSpring(Vector3* anchor, real springConstant, real damping);
    virtual void updateForce(Particle* particle, real duration);
};
}

#endif // PFGEN_HPP_
