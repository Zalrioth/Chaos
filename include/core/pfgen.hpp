#ifndef PFGEN_H_
#define PFGEN_H_

#include "core/core.hpp"
#include "core/particle.hpp"
#include <vector>

namespace chaos {
class ParticleForceGenerator {
public:
    virtual void updateForce(Particle* particle, real duration) = 0;
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

class ParticleAnchoredSpring : public ParticleForceGenerator {
protected:
    Vector3* anchor;
    real springConstant;
    real restLength;

public:
    ParticleAnchoredSpring();
    ParticleAnchoredSpring(Vector3* anchor, real springConstant, real restLength);
    const Vector3* getAnchor() const { return anchor; }
    void init(Vector3* anchor, real springConstant, real restLength);
    virtual void updateForce(Particle* particle, real duration);
};

class ParticleAnchoredBungee : public ParticleAnchoredSpring {
public:
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

class ParticleSpring : public ParticleForceGenerator {
    Particle* other;
    real springConstant;
    real restLength;

public:
    ParticleSpring(Particle* other, real springConstant, real restLength);
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

class ParticleBuoyancy : public ParticleForceGenerator {
    real maxDepth;
    real volume;
    real waterHeight;
    real liquidDensity;

public:
    ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity = 1000.0f);
    virtual void updateForce(Particle* particle, real duration);
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
}

#endif // PFGEN_H_