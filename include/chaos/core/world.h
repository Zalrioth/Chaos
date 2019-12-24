#pragma once
#ifndef WORLD_H
#define WORLD_H

#include <stdlib.h>
#include "chaos/core/body.h"
#include "chaos/core/contacts.h"

// TODO: Add this
//const static real velocityLimit = (real)0.25f;
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
struct World {
  bool calculate_iterations;
  struct BodyRegistration* first_body;
  struct ContactResolver resolver;
  struct ContactGenRegistration* first_contact_gen;
  struct Contact* contacts;
  unsigned int max_contacts;
};

static inline void world_init(struct World* world, unsigned int max_contacts, unsigned int iterations);
static inline void world_delete(struct World* world);
static inline void world_start_frame(struct World* world);
static inline unsigned int world_generate_contacts(struct World* world);
static inline void world_run_physics(struct World* world, real duration);

#endif  // WORLD_H
