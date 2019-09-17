#pragma once
#ifndef WORLD_H_
#define WORLD_H_

#include <cstdlib>
#include "core/body.hpp"
#include "core/contacts.hpp"

// TODO: Add this
//real chaos::sleepEpsilon = ((real)0.3);
//extern real sleepEpsilon;
//void setSleepEpsilon(real value);
//real getSleepEpsilon();
//void chaos::setSleepEpsilon(real value) {
//    chaos::sleepEpsilon = value;
//}
//real chaos::getSleepEpsilon() {
//    return chaos::sleepEpsilon;
//}
namespace chaos {
class World {
  bool calculateIterations;
  struct BodyRegistration {
    RigidBody* body;
    BodyRegistration* next;
  };
  BodyRegistration* firstBody;
  ContactResolver resolver;
  struct ContactGenRegistration {
    ContactGenerator* gen;
    ContactGenRegistration* next;
  };
  ContactGenRegistration* firstContactGen;
  Contact* contacts;
  unsigned maxContacts;

 public:
  World(unsigned maxContacts, unsigned iterations = 0);
  ~World();
  unsigned generateContacts();
  void runPhysics(real duration);
  void startFrame();
};
}  // namespace chaos

#endif  // WORLD_H_
