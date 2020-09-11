#pragma once
#ifndef JOINTS_H
#define JOINTS_H

#include <ubermath/ubermath.h>

#include "chaos/core/contacts.h"

struct Joint {
  struct ContactGenerator contact_generator;
  struct RigidBody* body[2];
  vec3 position[2];
  float error;
};

unsigned int joint_add_contact(struct Joint* joint, struct Contact* contact, unsigned int limit);
void joint_set(struct Joint* joint, struct RigidBody* a, vec3 a_pos, struct RigidBody* b, vec3 b_pos, float error);

#endif  // JOINTS_H
