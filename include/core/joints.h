#pragma once
#ifndef JOINTS_H
#define JOINTS_H

#include "core/contacts.h"

struct Joint {
  struct ContactGenerator contact_generator;
  struct RigidBody* body[2];
  vec3 position[2];
  real error;
};

static inline void joint_set(struct Joint* joint, struct RigidBody* a, real* a_pos, struct RigidBody* b, real* b_pos, real error);
static inline unsigned int joint_add_contact(struct Joint* joint, struct Contact* contact, unsigned limit);

#endif  // JOINTS_H
