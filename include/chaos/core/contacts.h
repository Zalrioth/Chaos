#pragma once
#ifndef CONTACTS_H
#define CONTACTS_H

#include <assert.h>
#include <memory.h>
#include "chaos/core/body.h"

// TODO: velocity_limit will need to changed
static real velocity_limit = (real)0.25f;

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

void contact_set_body_data(struct Contact* contact, struct RigidBody* one, struct RigidBody* two, real friction, real restitution);
void contact_match_awake_state(struct Contact* contact);
void contact_swap_bodies(struct Contact* contact);
void contact_calculate_contact_basis(struct Contact* contact);
real* contact_calculate_local_velocity(struct Contact* contact, unsigned int body_index, real duration);
void contact_calculate_desired_delta_velocity(struct Contact* contact, real duration);
void contact_calculate_internals(struct Contact* contact, real duration);
void contact_apply_velocity_change(struct Contact* contact, vec3* velocity_change, vec3* rotation_change);
real* contact_calculate_frictionless_impulse(struct Contact* contact, mat3* inverse_inertia_tensor);
real* contact_calculate_friction_impulse(struct Contact* contact, mat3* inverse_inertia_tensor);
void contact_apply_position_change(struct Contact* contact, vec3* linear_change, vec3* angular_change, real penetration);

struct ContactResolver {
  unsigned int velocity_iterations;
  unsigned int position_iterations;
  real velocity_epsilon;
  real position_epsilon;
  unsigned int velocity_iterations_used;
  unsigned int position_iterations_used;
};

void contact_resolver_init(struct ContactResolver* contact_resolver, unsigned int velocity_iterations, unsigned int position_iterations, real velocity_epsilon, real position_epsilon);
bool contact_resolver_is_valid(struct ContactResolver* contact_resolver);
void contact_resolver_set_iterations(struct ContactResolver* contact_resolver, unsigned int velocity_iterations, unsigned int position_iterations);
void contact_resolver_set_epsilon(struct ContactResolver* contact_resolver, real velocity_epsilon, real position_epsilon);
void contact_resolver_resolve_contacts(struct ContactResolver* contact_resolver, struct Contact* contacts, unsigned int num_contacts, real duration);
void contact_resolver_prepare_contacts(struct ContactResolver* contact_resolver, struct Contact* contacts, unsigned int num_contacts, real duration);
void contact_resolver_adjust_velocities(struct ContactResolver* contact_resolver, struct Contact* contact, unsigned num_contacts, real duration);
void contact_resolver_adjust_positions(struct ContactResolver* contact_resolver, struct Contact* contact, unsigned num_contacts, real duration);

struct ContactGenerator {
  unsigned int (*add_contact)(struct Contact*, unsigned int);
  //virtual unsigned addContact(Contact* contact, unsigned limit) const = 0;
};

struct ContactGenRegistration {
  struct ContactGenerator* gen;
  struct ContactGenRegistration* next;
};

#endif  // CONTACTS_H