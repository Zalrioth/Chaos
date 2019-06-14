#include "core/pcontacts.hpp"

using namespace chaos;

void ParticleContact::resolve(real duration)
{
    resolveVelocity(duration);
    resolveInterpenetration(duration);
}

real ParticleContact::calculateSeparatingVelocity() const
{
    Vector3 relativeVelocity = particle[0]->getVelocity();
    if (particle[1])
        relativeVelocity -= particle[1]->getVelocity();
    return relativeVelocity * contactNormal;
}

void ParticleContact::resolveVelocity(real duration)
{
    real separatingVelocity = calculateSeparatingVelocity();
    if (separatingVelocity > 0)
        return;

    real newSepVelocity = -separatingVelocity * restitution;

    Vector3 accCausedVelocity = particle[0]->getAcceleration();
    if (particle[1])
        accCausedVelocity -= particle[1]->getAcceleration();
    real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;

    if (accCausedSepVelocity < 0) {
        newSepVelocity += restitution * accCausedSepVelocity;

        if (newSepVelocity < 0)
            newSepVelocity = 0;
    }

    real deltaVelocity = newSepVelocity - separatingVelocity;

    real totalInverseMass = particle[0]->getInverseMass();
    if (particle[1])
        totalInverseMass += particle[1]->getInverseMass();

    if (totalInverseMass <= 0)
        return;

    real impulse = deltaVelocity / totalInverseMass;

    Vector3 impulsePerIMass = contactNormal * impulse;

    particle[0]->setVelocity(particle[0]->getVelocity() + impulsePerIMass * particle[0]->getInverseMass());
    if (particle[1])
        particle[1]->setVelocity(particle[1]->getVelocity() + impulsePerIMass * -particle[1]->getInverseMass());
}

void ParticleContact::resolveInterpenetration(real duration)
{
    if (penetration <= 0)
        return;

    real totalInverseMass = particle[0]->getInverseMass();
    if (particle[1])
        totalInverseMass += particle[1]->getInverseMass();

    if (totalInverseMass <= 0)
        return;

    Vector3 movePerIMass = contactNormal * (penetration / totalInverseMass);

    particleMovement[0] = movePerIMass * particle[0]->getInverseMass();
    if (particle[1]) {
        particleMovement[1] = movePerIMass * -particle[1]->getInverseMass();
    } else {
        particleMovement[1].clear();
    }

    particle[0]->setPosition(particle[0]->getPosition() + particleMovement[0]);
    if (particle[1]) {
        particle[1]->setPosition(particle[1]->getPosition() + particleMovement[1]);
    }
}

ParticleContactResolver::ParticleContactResolver(unsigned iterations)
    : iterations(iterations)
{
}

void ParticleContactResolver::setIterations(unsigned iterations)
{
    ParticleContactResolver::iterations = iterations;
}

void ParticleContactResolver::resolveContacts(ParticleContact* contactArray, unsigned numContacts, real duration)
{
    unsigned i;

    iterationsUsed = 0;
    while (iterationsUsed < iterations) {
        real max = REAL_MAX;
        unsigned maxIndex = numContacts;
        for (i = 0; i < numContacts; i++) {
            real sepVel = contactArray[i].calculateSeparatingVelocity();
            if (sepVel < max && (sepVel < 0 || contactArray[i].penetration > 0)) {
                max = sepVel;
                maxIndex = i;
            }
        }

        if (maxIndex == numContacts)
            break;

        contactArray[maxIndex].resolve(duration);

        Vector3* move = contactArray[maxIndex].particleMovement;
        for (i = 0; i < numContacts; i++) {
            if (contactArray[i].particle[0] == contactArray[maxIndex].particle[0]) {
                contactArray[i].penetration -= move[0] * contactArray[i].contactNormal;
            } else if (contactArray[i].particle[0] == contactArray[maxIndex].particle[1]) {
                contactArray[i].penetration -= move[1] * contactArray[i].contactNormal;
            }
            if (contactArray[i].particle[1]) {
                if (contactArray[i].particle[1] == contactArray[maxIndex].particle[0]) {
                    contactArray[i].penetration += move[0] * contactArray[i].contactNormal;
                } else if (contactArray[i].particle[1] == contactArray[maxIndex].particle[1]) {
                    contactArray[i].penetration += move[1] * contactArray[i].contactNormal;
                }
            }
        }

        iterationsUsed++;
    }
}
