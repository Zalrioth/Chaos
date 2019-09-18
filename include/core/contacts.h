#ifndef CONTACTS_H
#define CONTACTS_H

#include <assert.h>
#include <memory.h>
#include "core/body.h"

struct Contact {
  struct RigidBody* body[2];
  real friction;
  real restitution;
  vec3 contact_point;
  vec3 contact_normal;
  real penetration;
  mat3 contact_to_world;
  vec3 contact_velocity;
  real desired_delta_velocity;
  vec3 relative_contact_position[2];
  void setBodyData(RigidBody* one, RigidBody* two, real friction, real restitution);
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

struct ContactResolver {
  unsigned velocityIterations;
  unsigned positionIterations;
  real velocityEpsilon;
  real positionEpsilon;
  unsigned velocityIterationsUsed;
  unsigned positionIterationsUsed;
  ContactResolver(unsigned iterations, real velocityEpsilon = (real)0.01, real positionEpsilon = (real)0.01);
  ContactResolver(unsigned velocityIterations, unsigned positionIterations, real velocityEpsilon = (real)0.01, real positionEpsilon = (real)0.01);
  bool isValid() {
    return (velocityIterations > 0) && (positionIterations > 0) && (positionEpsilon >= 0.0f) && (positionEpsilon >= 0.0f);
  }
  void setIterations(unsigned velocityIterations, unsigned positionIterations);
  void setIterations(unsigned iterations);
  void setEpsilon(real velocityEpsilon, real positionEpsilon);
  void resolveContacts(Contact* contactArray, unsigned numContacts, real duration);
  void prepareContacts(Contact* contactArray, unsigned numContacts, real duration);
  void adjustVelocities(Contact* contactArray, unsigned numContacts, real duration);
  void adjustPositions(Contact* contacts, unsigned numContacts, real duration);
};

struct ContactGenerator {
  void (*add_contact)(struct Contact*, unsigned int);
  //virtual unsigned addContact(Contact* contact, unsigned limit) const = 0;
};

#endif  // CONTACTS_H