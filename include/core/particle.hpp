#pragma once
#ifndef PARTICLE_HPP_
#define PARTICLE_HPP_

#include "core/core.hpp"
#include <assert.h>

namespace chaos {
class Particle {
protected:
    real inverseMass;
    real damping;
    Vector3 position;
    Vector3 velocity;
    Vector3 forceAccum;
    Vector3 acceleration;

public:
    void integrate(real duration);
    void setMass(const real mass);
    real getMass() const;
    void setInverseMass(const real inverseMass);
    real getInverseMass() const;
    bool hasFiniteMass() const;
    void setDamping(const real damping);
    real getDamping() const;
    void setPosition(const Vector3& position);
    void setPosition(const real x, const real y, const real z);
    void getPosition(Vector3* position) const;
    Vector3 getPosition() const;
    void setVelocity(const Vector3& velocity);
    void setVelocity(const real x, const real y, const real z);
    void getVelocity(Vector3* velocity) const;
    Vector3 getVelocity() const;
    void setAcceleration(const Vector3& acceleration);
    void setAcceleration(const real x, const real y, const real z);
    void getAcceleration(Vector3* acceleration) const;
    Vector3 getAcceleration() const;
    void clearAccumulator();
    void addForce(const Vector3& force);
};
}

#endif // PARTICLE_HPP_
