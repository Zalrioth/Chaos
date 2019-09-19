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
};

static inline void contact_set_body_data(struct Contact* contact, struct RigidBody* one, struct RigidBody* two, real friction, real restitution);
static inline void contact_match_awake_state(struct Contact* contact);
static inline void contact_swap_bodies(struct Contact* contact);
static inline void contact_calculate_contact_basis(struct Contact* contact);
static inline real* contact_calculate_local_velocity(struct Contact* contact, unsigned int body_index, real duration);
static inline void contact_calculate_desired_delta_velocity(struct Contact* contact, real duration);
static inline contact_calculate_internals(struct Contact* contact, real duration);
static inline void contact_apply_velocity_change(struct Contact* contact, vec3* velocity_change, vec3* rotation_change);
static inline real* contact_calculate_frictionless_impulse(struct Contact* contact, mat3* inverse_inertia_tensor);
static inline real* contact_calculate_friction_impulse(struct Contact* contact, mat3* inverse_inertia_tensor);
static inline void contact_apply_position_change(struct Contact* contact, vec3* linear_change, vec3* angular_change, real penetration);

struct ContactResolver {
  unsigned int velocity_iterations;
  unsigned int position_iterations;
  real velocity_epsilon;
  real position_epsilon;
  unsigned int velocity_iterations_used;
  unsigned int position_iterations_used;
  ContactResolver(unsigned iterations, real velocityEpsilon = (real)0.01, real positionEpsilon = (real)0.01);
  ContactResolver(unsigned velocityIterations, unsigned positionIterations, real velocityEpsilon = (real)0.01, real positionEpsilon = (real)0.01);
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