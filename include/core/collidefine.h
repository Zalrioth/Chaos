#pragma once
#ifndef COLLIDE_FINE_H
#define COLLIDE_FINE_H

#include <memory.h>
#include <cstdio>
#include <cstdlib>
#include "core/contacts.h"

struct CollisionPrimitive {
  struct RigidBody* body;
  mat4 offset;
  mat4 transform;
};

void collision_primitive_calculate_internals(struct CollisionPrimitive* collision_primitive);
real* collision_primitive_get_axis(struct CollisionPrimitive* collision_primitive, unsigned int index);
real* collision_primitive_get_transform(struct CollisionPrimitive* collision_primitive);

struct CollisionSphere {
  struct CollisionPrimitive collision_primitive;
  real radius;
};

struct CollisionPlane {
  vec3 direction;
  real offset;
};

struct CollisionBox {
  struct CollisionPrimitive collision_primitive;
  vec3 half_size;
};

static inline bool intersection_test_sphere_and_half_space(struct CollisionSphere* sphere, struct CollisionPlane* plane);
static inline bool intersection_test_sphere_and_sphere(struct CollisionSphere* one, struct CollisionSphere* two);
static inline bool intersection_test_box_and_box(struct CollisionBox* one, struct CollisionBox* two);
static inline bool intersection_test_box_and_half_space(struct CollisionBox* box, struct CollisionPlane* plane);

struct CollisionData {
  struct Contact* contact_array;
  struct Contact* contacts;
  int contacts_left;
  unsigned int contact_count;
  real friction;
  real restitution;
  real tolerance;
};

static inline bool collision_data_has_more_contacts(struct CollisionData* collision_data);
static inline void collision_data_reset(struct CollisionData* collision_data, unsigned int max_contacts);
static inline void collision_data_add_contacts(struct CollisionData* collision_data, unsigned int count);
static inline unsigned int collision_detector_sphere_and_half_space(struct CollisionSphere* sphere, struct CollisionPlane* plane, struct CollisionData* data);
static inline unsigned int collision_detector_sphere_and_true_plane(struct CollisionSphere* sphere, struct CollisionPlane* plane, struct CollisionData* data);
static inline unsigned int collision_detector_sphere_and_sphere(struct CollisionSphere* one, struct CollisionSphere* two, struct CollisionData* data);
static inline unsigned int collision_detector_box_and_half_space(struct CollisionBox* box, struct CollisionPlane* plane, struct CollisionData* data);
static inline unsigned int collision_detector_box_and_box(struct CollisionBox* one, struct CollisionBox* two, struct CollisionData* data);
static inline unsigned int collision_detector_box_and_point(struct CollisionBox* box, struct Vector3* point, struct CollisionData* data);
static inline unsigned int collision_detector_box_and_sphere(struct CollisionBox* box, struct CollisionSphere* sphere, struct CollisionData* data);

#endif  // COLLIDE_FINE_H