#pragma once
#ifndef PARTICLE_HPP_
#define PARTICLE_HPP_

#include "core/core.hpp"
#include <assert.h>

namespace chaos {
class Particle {
protected:
    Vector3 position;
    Vector3 velocity;
    Vector3 acceleration;
    real damping;
    real inverseMass;
    Vector3 forceAccum;

public:
    void integrate(real duration);
    void clearAccumulator();
    void addForce(const Vector3& force);
};
}

#endif // PARTICLE_HPP_