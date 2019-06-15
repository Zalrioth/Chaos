#ifndef CONTACTS_HPP_
#define CONTACTS_HPP_

#include "core/body.hpp"
#include <assert.h>
#include <memory.h>

namespace chaos {
class ContactResolver;
class Contact {
    friend class ContactResolver;

public:
    RigidBody* body[2];
    real friction;
    real restitution;
    Vector3 contactPoint;
    Vector3 contactNormal;
    real penetration;
    void setBodyData(RigidBody* one, RigidBody* two, real friction, real restitution);

protected:
    Matrix3 contactToWorld;
    Vector3 contactVelocity;
    real desiredDeltaVelocity;
    Vector3 relativeContactPosition[2];

protected:
    void calculateInternals(real duration);
    void swapBodies();
    void matchAwakeState();
    void calculateDesiredDeltaVelocity(real duration);
    Vector3 calculateLocalVelocity(unsigned bodyIndex, real duration);
    void calculateContactBasis();
    void applyImpulse(const Vector3& impulse, RigidBody* body, Vector3* velocityChange, Vector3* rotationChange);
    void applyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2]);
    void applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real penetration);
    Vector3 calculateFrictionlessImpulse(Matrix3* inverseInertiaTensor);
    Vector3 calculateFrictionImpulse(Matrix3* inverseInertiaTensor);
};

class ContactResolver {
protected:
    unsigned velocityIterations;
    unsigned positionIterations;
    real velocityEpsilon;
    real positionEpsilon;

public:
    unsigned velocityIterationsUsed;
    unsigned positionIterationsUsed;

private:
    //bool validSettings;

public:
    ContactResolver(unsigned iterations, real velocityEpsilon = (real)0.01, real positionEpsilon = (real)0.01);
    ContactResolver(unsigned velocityIterations, unsigned positionIterations, real velocityEpsilon = (real)0.01, real positionEpsilon = (real)0.01);
    bool isValid()
    {
        return (velocityIterations > 0) && (positionIterations > 0) && (positionEpsilon >= 0.0f) && (positionEpsilon >= 0.0f);
    }
    void setIterations(unsigned velocityIterations, unsigned positionIterations);
    void setIterations(unsigned iterations);
    void setEpsilon(real velocityEpsilon, real positionEpsilon);
    void resolveContacts(Contact* contactArray, unsigned numContacts, real duration);

protected:
    void prepareContacts(Contact* contactArray, unsigned numContacts, real duration);
    void adjustVelocities(Contact* contactArray, unsigned numContacts, real duration);
    void adjustPositions(Contact* contacts, unsigned numContacts, real duration);
};

class ContactGenerator {
public:
    virtual unsigned addContact(Contact* contact, unsigned limit) const = 0;
};
}

#endif // CONTACTS_HPP_