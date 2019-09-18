#include "core/contacts.h"

static inline void contact_set_body_data(struct Contact* contact, struct RigidBody* one, struct RigidBody* two, real friction, real restitution) {
  contact->body[0] = one;
  contact->body[1] = two;
  contact->friction = friction;
  contact->restitution = restitution;
}

static inline void contacnt_match_awake_state(struct Contact* contact) {
  if (!contact->body[1])
    return;

  bool body0_awake = body_get_awake(contact->body[0]);
  bool body1_awake = body_get_awake(contact->body[1]);

  if (body0_awake ^ body1_awake) {
    if (body0_awake)
      body_set_awake(body[1], true);
    else
      body_set_awake(body[0], true);
  }
}

static inline void contact_swap_bodies(struct Contact* contact) {
  vec3_clone(contact->contact_normal, vec3_invert(contact->contact_normal));

  struct RigidBody* temp = body[0];
  body[0] = body[1];
  body[1] = temp;
}

static inline void contact_calculate_contact_basis(struct Contact* contact) {
  vec3 contact_tangent[2];

  real* contact_normal = contact->contact_normal;

  if (real_abs(contact_normal[0]) > real_abs(contact_normal[1])) {
    const real s = (real)1.0f / real_sqrt(contact_normal[2] * contact_normal[2] + contact_normal[0] * contact_normal[0]);

    contact_tangent[0][0] = contact_normal[2] * s;
    contact_tangent[0][1] = 0;
    contact_tangent[0][2] = -contact_normal[0] * s;

    contact_tangent[1][0] = contact_normal[1] * contact_tangent[0][0];
    contact_tangent[1][1] = contact_normal[2] * contact_tangent[0][0] - contact_normal[0] * contact_tangent[0][2];
    contact_tangent[1][2] = -contact_normal[1] * contact_tangent[0][0];
  } else {
    const real s = (real)1.0 / real_sqrt(contact_normal[2] * contact_normal[2] + contact_normal[1] * contact_normal[1]);

    contact_tangent[0][0] = 0;
    contact_tangent[0][1] = -contact_normal[2] * s;
    contact_tangent[0][2] = contact_normal[1] * s;

    contact_tangent[1][0] = contact_normal[1] * contact_tangent[0][2] - contact_normal[2] * contact_tangent[0][2];
    contact_tangent[1][1] = -contact_normal[0] * contact_tangent[0][2];
    contact_tangent[1][2] = contact_normal[0] * contact_tangent[0][1];
  }
  mat3_clone(contact->contact_to_world, mat3_set_components(contact_normal, contact_tangent[0], contact_tangent[1]));
}

static inline real* contact_calculate_local_velocity(struct Contact* contact, unsigned int body_index, real duration) {
  struct RigidBody* this_body = body[body_index];

  real* velocity;
  vec3_clone(velocity, vec3_cross(rigid_body_get_rotation(this_body), contact->relative_contact_position[body_index]));
  vec3_clone(velocity, vec3_add(velocity, rigid_body_get_velocity(this_body)));

  vec3 contact_velocity;
  vec3_clone(contact_velocity, mat3_transform_transpose(contact->contact_to_world, velocity));

  vec3 acc_velocity;
  vec3_clone(acc_velocity, vec3_mul_scalar(rigid_body_get_last_frame_acceleration(this_body), duration));
  vec3_clone(acc_velocity, mat3_transform_transpose(contact->contact_to_world, acc_velocity));
  acc_velocity[0] = 0;

  vec3_clone(contact_velocity, vec3_add(contact_velocity, acc_velocity));
  return contact_velocity;
}

void Contact::calculateDesiredDeltaVelocity(real duration) {
  const static real velocityLimit = (real)0.25f;
  real velocityFromAcc = 0;

  if (body[0]->getAwake())
    velocityFromAcc += body[0]->getLastFrameAcceleration() * duration * contactNormal;

  if (body[1] && body[1]->getAwake())
    velocityFromAcc -= body[1]->getLastFrameAcceleration() * duration * contactNormal;

  real thisRestitution = restitution;
  if (real_abs(contactVelocity.x) < velocityLimit)
    thisRestitution = (real)0.0f;

  desiredDeltaVelocity = -contactVelocity.x - thisRestitution * (contactVelocity.x - velocityFromAcc);
}

void Contact::calculateInternals(real duration) {
  if (!body[0])
    swapBodies();
  assert(body[0]);

  calculateContactBasis();

  relativeContactPosition[0] = contactPoint - body[0]->getPosition();
  if (body[1])
    relativeContactPosition[1] = contactPoint - body[1]->getPosition();

  contactVelocity = calculateLocalVelocity(0, duration);
  if (body[1])
    contactVelocity -= calculateLocalVelocity(1, duration);

  calculateDesiredDeltaVelocity(duration);
}

void Contact::applyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2]) {
  Matrix3 inverseInertiaTensor[2];
  body[0]->getInverseInertiaTensorWorld(&inverseInertiaTensor[0]);
  if (body[1])
    body[1]->getInverseInertiaTensorWorld(&inverseInertiaTensor[1]);

  Vector3 impulseContact;

  if (friction == (real)0.0)
    impulseContact = calculateFrictionlessImpulse(inverseInertiaTensor);
  else
    impulseContact = calculateFrictionImpulse(inverseInertiaTensor);

  Vector3 impulse = contactToWorld.transform(impulseContact);
  Vector3 impulsiveTorque = relativeContactPosition[0] % impulse;
  rotationChange[0] = inverseInertiaTensor[0].transform(impulsiveTorque);
  velocityChange[0].clear();
  velocityChange[0].addScaledVector(impulse, body[0]->getInverseMass());

  body[0]->addVelocity(velocityChange[0]);
  body[0]->addRotation(rotationChange[0]);

  if (body[1]) {
    Vector3 impulsiveTorque = impulse % relativeContactPosition[1];
    rotationChange[1] = inverseInertiaTensor[1].transform(impulsiveTorque);
    velocityChange[1].clear();
    velocityChange[1].addScaledVector(impulse, -body[1]->getInverseMass());

    body[1]->addVelocity(velocityChange[1]);
    body[1]->addRotation(rotationChange[1]);
  }
}

inline Vector3 Contact::calculateFrictionlessImpulse(Matrix3* inverseInertiaTensor) {
  Vector3 impulseContact;
  Vector3 deltaVelWorld = relativeContactPosition[0] % contactNormal;
  deltaVelWorld = inverseInertiaTensor[0].transform(deltaVelWorld);
  deltaVelWorld = deltaVelWorld % relativeContactPosition[0];
  real deltaVelocity = deltaVelWorld * contactNormal;
  deltaVelocity += body[0]->getInverseMass();

  if (body[1]) {
    Vector3 deltaVelWorld = relativeContactPosition[1] % contactNormal;
    deltaVelWorld = inverseInertiaTensor[1].transform(deltaVelWorld);
    deltaVelWorld = deltaVelWorld % relativeContactPosition[1];
    deltaVelocity += deltaVelWorld * contactNormal;
    deltaVelocity += body[1]->getInverseMass();
  }

  impulseContact.x = desiredDeltaVelocity / deltaVelocity;
  impulseContact.y = 0;
  impulseContact.z = 0;
  return impulseContact;
}

inline Vector3 Contact::calculateFrictionImpulse(Matrix3* inverseInertiaTensor) {
  Vector3 impulseContact;
  real inverseMass = body[0]->getInverseMass();

  Matrix3 impulseToTorque;
  impulseToTorque.setSkewSymmetric(relativeContactPosition[0]);

  Matrix3 deltaVelWorld = impulseToTorque;
  deltaVelWorld *= inverseInertiaTensor[0];
  deltaVelWorld *= impulseToTorque;
  deltaVelWorld *= -1;

  if (body[1]) {
    impulseToTorque.setSkewSymmetric(relativeContactPosition[1]);

    Matrix3 deltaVelWorld2 = impulseToTorque;
    deltaVelWorld2 *= inverseInertiaTensor[1];
    deltaVelWorld2 *= impulseToTorque;
    deltaVelWorld2 *= -1;

    deltaVelWorld += deltaVelWorld2;

    inverseMass += body[1]->getInverseMass();
  }

  Matrix3 deltaVelocity = contactToWorld.transpose();
  deltaVelocity *= deltaVelWorld;
  deltaVelocity *= contactToWorld;

  deltaVelocity.data[0] += inverseMass;
  deltaVelocity.data[4] += inverseMass;
  deltaVelocity.data[8] += inverseMass;

  Matrix3 impulseMatrix = deltaVelocity.inverse();
  Vector3 velKill(desiredDeltaVelocity, -contactVelocity.y, -contactVelocity.z);
  impulseContact = impulseMatrix.transform(velKill);

  real planarImpulse = real_sqrt(
      impulseContact.y * impulseContact.y + impulseContact.z * impulseContact.z);
  if (planarImpulse > impulseContact.x * friction) {
    impulseContact.y /= planarImpulse;
    impulseContact.z /= planarImpulse;

    impulseContact.x = deltaVelocity.data[0] + deltaVelocity.data[1] * friction * impulseContact.y + deltaVelocity.data[2] * friction * impulseContact.z;
    impulseContact.x = desiredDeltaVelocity / impulseContact.x;
    impulseContact.y *= friction * impulseContact.x;
    impulseContact.z *= friction * impulseContact.x;
  }
  return impulseContact;
}

void Contact::applyPositionChange(Vector3 linearChange[2], Vector3 angularChange[2], real penetration) {
  const real angularLimit = (real)0.2f;
  real angularMove[2];
  real linearMove[2];

  real totalInertia = 0;
  real linearInertia[2];
  real angularInertia[2];

  for (unsigned i = 0; i < 2; i++)
    if (body[i]) {
      Matrix3 inverseInertiaTensor;
      body[i]->getInverseInertiaTensorWorld(&inverseInertiaTensor);

      Vector3 angularInertiaWorld = relativeContactPosition[i] % contactNormal;
      angularInertiaWorld = inverseInertiaTensor.transform(angularInertiaWorld);
      angularInertiaWorld = angularInertiaWorld % relativeContactPosition[i];
      angularInertia[i] = angularInertiaWorld * contactNormal;

      linearInertia[i] = body[i]->getInverseMass();

      totalInertia += linearInertia[i] + angularInertia[i];
    }

  for (unsigned i = 0; i < 2; i++)
    if (body[i]) {
      real sign = (i == 0) ? 1 : -1;
      angularMove[i] = sign * penetration * (angularInertia[i] / totalInertia);
      linearMove[i] = sign * penetration * (linearInertia[i] / totalInertia);

      Vector3 projection = relativeContactPosition[i];
      projection.addScaledVector(contactNormal, -relativeContactPosition[i].scalarProduct(contactNormal));

      real maxMagnitude = angularLimit * projection.magnitude();

      if (angularMove[i] < -maxMagnitude) {
        real totalMove = angularMove[i] + linearMove[i];
        angularMove[i] = -maxMagnitude;
        linearMove[i] = totalMove - angularMove[i];
      } else if (angularMove[i] > maxMagnitude) {
        real totalMove = angularMove[i] + linearMove[i];
        angularMove[i] = maxMagnitude;
        linearMove[i] = totalMove - angularMove[i];
      }

      if (angularMove[i] == 0)
        angularChange[i].clear();
      else {
        Vector3 targetAngularDirection = relativeContactPosition[i].vectorProduct(contactNormal);

        Matrix3 inverseInertiaTensor;
        body[i]->getInverseInertiaTensorWorld(&inverseInertiaTensor);

        angularChange[i] = inverseInertiaTensor.transform(targetAngularDirection) * (angularMove[i] / angularInertia[i]);
      }

      linearChange[i] = contactNormal * linearMove[i];

      Vector3 pos;
      body[i]->getPosition(&pos);
      pos.addScaledVector(contactNormal, linearMove[i]);
      body[i]->setPosition(pos);

      Quaternion q;
      body[i]->getOrientation(&q);
      q.addScaledVector(angularChange[i], ((real)1.0));
      body[i]->setOrientation(q);

      if (!body[i]->getAwake())
        body[i]->calculateDerivedData();
    }
}

ContactResolver::ContactResolver(unsigned iterations,
                                 real velocityEpsilon,
                                 real positionEpsilon) {
  setIterations(iterations, iterations);
  setEpsilon(velocityEpsilon, positionEpsilon);
}

ContactResolver::ContactResolver(unsigned velocityIterations,
                                 unsigned positionIterations,
                                 real velocityEpsilon,
                                 real positionEpsilon) {
  setIterations(velocityIterations);
  setEpsilon(velocityEpsilon, positionEpsilon);
}

void ContactResolver::setIterations(unsigned iterations) {
  setIterations(iterations, iterations);
}

void ContactResolver::setIterations(unsigned velocityIterations,
                                    unsigned positionIterations) {
  ContactResolver::velocityIterations = velocityIterations;
  ContactResolver::positionIterations = positionIterations;
}

void ContactResolver::setEpsilon(real velocityEpsilon,
                                 real positionEpsilon) {
  ContactResolver::velocityEpsilon = velocityEpsilon;
  ContactResolver::positionEpsilon = positionEpsilon;
}

void ContactResolver::resolveContacts(Contact* contacts, unsigned numContacts, real duration) {
  if (numContacts == 0)
    return;
  if (!isValid())
    return;

  prepareContacts(contacts, numContacts, duration);
  adjustPositions(contacts, numContacts, duration);
  adjustVelocities(contacts, numContacts, duration);
}

void ContactResolver::prepareContacts(Contact* contacts, unsigned numContacts, real duration) {
  Contact* lastContact = contacts + numContacts;
  for (Contact* contact = contacts; contact < lastContact; contact++) {
    contact->calculateInternals(duration);
  }
}

void ContactResolver::adjustVelocities(Contact* c, unsigned numContacts, real duration) {
  Vector3 velocityChange[2], rotationChange[2];
  Vector3 deltaVel;

  velocityIterationsUsed = 0;
  while (velocityIterationsUsed < velocityIterations) {
    real max = velocityEpsilon;
    unsigned index = numContacts;
    for (unsigned i = 0; i < numContacts; i++) {
      if (c[i].desiredDeltaVelocity > max) {
        max = c[i].desiredDeltaVelocity;
        index = i;
      }
    }
    if (index == numContacts)
      break;

    c[index].matchAwakeState();
    c[index].applyVelocityChange(velocityChange, rotationChange);

    for (unsigned i = 0; i < numContacts; i++) {
      for (unsigned b = 0; b < 2; b++)
        if (c[i].body[b]) {
          for (unsigned d = 0; d < 2; d++) {
            if (c[i].body[b] == c[index].body[d]) {
              deltaVel = velocityChange[d] + rotationChange[d].vectorProduct(c[i].relativeContactPosition[b]);
              c[i].contactVelocity += c[i].contactToWorld.transformTranspose(deltaVel) * (b ? -1 : 1);
              c[i].calculateDesiredDeltaVelocity(duration);
            }
          }
        }
    }
    velocityIterationsUsed++;
  }
}

void ContactResolver::adjustPositions(Contact* c, unsigned numContacts, real duration) {
  unsigned i, index;
  Vector3 linearChange[2], angularChange[2];
  real max;
  Vector3 deltaPosition;

  positionIterationsUsed = 0;
  while (positionIterationsUsed < positionIterations) {
    max = positionEpsilon;
    index = numContacts;
    for (i = 0; i < numContacts; i++) {
      if (c[i].penetration > max) {
        max = c[i].penetration;
        index = i;
      }
    }
    if (index == numContacts)
      break;

    c[index].matchAwakeState();
    c[index].applyPositionChange(linearChange, angularChange, max);

    for (i = 0; i < numContacts; i++) {
      for (unsigned b = 0; b < 2; b++)
        if (c[i].body[b]) {
          for (unsigned d = 0; d < 2; d++) {
            if (c[i].body[b] == c[index].body[d]) {
              deltaPosition = linearChange[d] + angularChange[d].vectorProduct(c[i].relativeContactPosition[b]);
              c[i].penetration += deltaPosition.scalarProduct(c[i].contactNormal) * (b ? 1 : -1);
            }
          }
        }
    }
    positionIterationsUsed++;
  }
}
