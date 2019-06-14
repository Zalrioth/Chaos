#include <core/pfgen.hpp>

using namespace chaos;

void ParticleForceRegistry::updateForces(real duration)
{
    Registry::iterator i = registrations.begin();
    for (; i != registrations.end(); i++) {
        i->fg->updateForce(i->particle, duration);
    }
}

void ParticleForceRegistry::add(Particle* particle, ParticleForceGenerator* fg)
{
    ParticleForceRegistry::ParticleForceRegistration registration;
    registration.particle = particle;
    registration.fg = fg;
    registrations.push_back(registration);
}

ParticleGravity::ParticleGravity(const Vector3& gravity)
    : gravity(gravity)
{
}

void ParticleGravity::updateForce(Particle* particle, real duration)
{
    if (!particle->hasFiniteMass())
        return;

    particle->addForce(gravity * particle->getMass());
}

ParticleDrag::ParticleDrag(real k1, real k2)
    : k1(k1)
    , k2(k2)
{
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

ParticleSpring::ParticleSpring(Particle* other, real sc, real rl)
    : other(other)
    , springConstant(sc)
    , restLength(rl)
{
}

void ParticleSpring::updateForce(Particle* particle, real duration)
{
    Vector3 force;
    particle->getPosition(&force);
    force -= other->getPosition();

    real magnitude = force.magnitude();
    magnitude = real_abs(magnitude - restLength);
    magnitude *= springConstant;

    force.normalise();
    force *= -magnitude;
    particle->addForce(force);
}

ParticleBuoyancy::ParticleBuoyancy(real maxDepth, real volume, real waterHeight, real liquidDensity)
    : maxDepth(maxDepth)
    , volume(volume)
    , waterHeight(waterHeight)
    , liquidDensity(liquidDensity)
{
}

void ParticleBuoyancy::updateForce(Particle* particle, real duration)
{
    real depth = particle->getPosition().y;

    if (depth >= waterHeight + maxDepth)
        return;
    Vector3 force(0, 0, 0);

    if (depth <= waterHeight - maxDepth) {
        force.y = liquidDensity * volume;
        particle->addForce(force);
        return;
    }

    force.y = liquidDensity * volume * (depth - maxDepth - waterHeight) / 2 * maxDepth;
    particle->addForce(force);
}

ParticleBungee::ParticleBungee(Particle* other, real sc, real rl)
    : other(other)
    , springConstant(sc)
    , restLength(rl)
{
}

void ParticleBungee::updateForce(Particle* particle, real duration)
{
    Vector3 force;
    particle->getPosition(&force);
    force -= other->getPosition();

    real magnitude = force.magnitude();
    if (magnitude <= restLength)
        return;

    magnitude = springConstant * (restLength - magnitude);

    force.normalise();
    force *= -magnitude;
    particle->addForce(force);
}

ParticleFakeSpring::ParticleFakeSpring(Vector3* anchor, real sc, real d)
    : anchor(anchor)
    , springConstant(sc)
    , damping(d)
{
}

void ParticleFakeSpring::updateForce(Particle* particle, real duration)
{
    if (!particle->hasFiniteMass())
        return;

    Vector3 position;
    particle->getPosition(&position);
    position -= *anchor;

    real gamma = 0.5f * real_sqrt(4 * springConstant - damping * damping);
    if (gamma == 0.0f)
        return;
    Vector3 c = position * (damping / (2.0f * gamma)) + particle->getVelocity() * (1.0f / gamma);

    Vector3 target = position * real_cos(gamma * duration) + c * real_sin(gamma * duration);
    target *= real_exp(-0.5f * duration * damping);

    Vector3 accel = (target - position) * ((real)1.0 / (duration * duration)) - particle->getVelocity() * ((real)1.0 / duration);
    particle->addForce(accel * particle->getMass());
}

ParticleAnchoredSpring::ParticleAnchoredSpring()
{
}

ParticleAnchoredSpring::ParticleAnchoredSpring(Vector3* anchor, real sc, real rl)
    : anchor(anchor)
    , springConstant(sc)
    , restLength(rl)
{
}

void ParticleAnchoredSpring::init(Vector3* anchor, real springConstant, real restLength)
{
    ParticleAnchoredSpring::anchor = anchor;
    ParticleAnchoredSpring::springConstant = springConstant;
    ParticleAnchoredSpring::restLength = restLength;
}

void ParticleAnchoredBungee::updateForce(Particle* particle, real duration)
{
    Vector3 force;
    particle->getPosition(&force);
    force -= *anchor;

    real magnitude = force.magnitude();
    if (magnitude < restLength)
        return;

    magnitude = magnitude - restLength;
    magnitude *= springConstant;

    force.normalise();
    force *= -magnitude;
    particle->addForce(force);
}

void ParticleAnchoredSpring::updateForce(Particle* particle, real duration)
{
    Vector3 force;
    particle->getPosition(&force);
    force -= *anchor;

    real magnitude = force.magnitude();
    magnitude = (restLength - magnitude) * springConstant;

    force.normalise();
    force *= magnitude;
    particle->addForce(force);
}
