#include "core/particle.hpp"

using namespace chaos;

void Particle::integrate(real duration)
{
    if (inverseMass <= 0.0f)
        return;

    assert(duration > 0.0);

    position.addScaledVector(velocity, duration);

    Vector3 resultingAcc = acceleration;

    velocity.addScaledVector(resultingAcc, duration);

    velocity *= real_pow(damping, duration);

    clearAccumulator();
}

void Particle::clearAccumulator()
{
    forceAccum.clear();
}

void Particle::addForce(const Vector3& force)
{
    forceAccum += force;
}
