#include "core/pfgen.hpp"

using namespace chaos;

void ParticleForceRegistry::updateForces(real duration)
{
    Registry::iterator i = registrations.begin();
    for (; i != registrations.end(); i++)
        i->fg->updateForce(i->particle, duration);
}

void PartivleGravity::updateForce(Particle* particle, real duration)
{
    if (!particle->hasFiniteMass())
        return;

    particle->addForce(gravity * particle->getMass());
}

void ParticleDrag::updateForce(Particle* particle, real duration)
{
    Vector3 force;
    particle->getVelocity(&force);

    real dragCoeff = force.magnitude();
    dragCoeff = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

    force.normalise();
    force *= -dragCoeff;
    particle->addForce(force);
}

void ParticleSpring::updateFoce(Particle* particle, real duration)
{
    Vector3 foce;
    particle->getPosition(&force);
    force -= other->getPosition();

    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude - restLength);
    magnitude *= springConstant;

    force->normalise();
    force *= -magnitude;
    particle->addForce(force);
}