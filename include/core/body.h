#pragma once
#ifndef BODY_H
#define BODY_H

#include <memory.h>
#include "core/chaos.h"

struct RigidBody {
  real inverse_mass;
  mat3 inverse_inertia_tensor;
  real linear_damping;
  real angular_damping;
  vec3 position;
  quaternion orientation;
  vec3 velocity;
  vec3 rotation;
  mat3 inverse_inertia_tensor_world;
  real motion;
  bool is_awake;
  bool can_sleep;
  mat4 transform_matrix;
  vec3 force_accum;
  vec3 torque_accum;
  vec3 acceleration;
  vec3 last_frame_acceleration;
};

static inline void rigid_body_transform_inertia_tensor(real* iit_world, real* q, real* iit_body, real* rotmat);
static inline void rigid_body_calculate_transform_matrix(real* transform_matrix, real* position, real* orientation);
static inline void rigid_body_calculate_derived_data(struct RigidBody* rigid_body);
static inline void rigid_body_integrate(struct RigidBody* rigid_body, real duration);
static inline void rigid_body_set_mass(struct RigidBody* rigid_body, real mass);
static inline real rigid_body_get_mass(struct RigidBody* rigid_body);
static inline void rigid_body_set_inverse_mass(struct RigidBody* rigid_body, real inverse_mass);
static inline real rigid_body_get_inverse_mass(struct RigidBody* rigid_body);
static inline bool rigid_body_has_finite_mass(struct RigidBody* rigid_body);
static inline void rigid_body_set_inertia_tensor(struct RigidBody* rigid_body, real* inertia_tensor);
static inline real* rigid_body_get_inertia_tensor(struct RigidBody* rigid_body);
static inline real* rigid_body_get_inertia_tensor_world(struct RigidBody* rigid_body);
static inline void rigid_body_set_inverse_inertia_tensor(struct RigidBody* rigid_body, real* inverse_inertia_tensor);
static inline real* rigid_body_get_inverse_inertia_tensor(struct RigidBody* rigid_body);
static inline real* rigid_body_get_inverse_inertia_tensor_world(struct RigidBody* rigid_body);
static inline void rigid_body_set_damping(struct RigidBody* rigid_body, real linear_damping, real angular_damping);
static inline void rigid_body_set_linear_damping(struct RigidBody* rigid_body, real linear_damping);
static inline real rigid_body_get_linear_damping(struct RigidBody* rigid_body);
static inline void rigid_body_set_angular_damping(struct RigidBody* rigid_body, real angular_damping);
static inline real rigid_body_get_angular_damping(struct RigidBody* rigid_body);
static inline void rigid_body_set_position(struct RigidBody* rigid_body, real* position);
static inline void rigid_body_set_position_xyz(struct RigidBody* rigid_body, real x, real y, real z);
static inline real* rigid_body_get_position(struct RigidBody* rigid_body);
static inline void rigid_body_set_orientation(struct RigidBody* rigid_body, real* orientation);
static inline void rigid_body_set_orientation_rijk(struct RigidBody* rigid_body, real r, real i, real j, real k);
static inline real* rigid_body_get_orientation(struct RigidBody* rigid_body);
static inline void rigid_body_get_transform(struct RigidBody* rigid_body);
static inline void rigid_body_get_transform_4x4(struct RigidBody* rigid_body, real* matrix);
static inline void rigid_body_get_transform_gl4x4(struct RigidBody* rigid_body, real* matrix);
static inline real* rigid_body_get_point_in_local_space(struct RigidBody* rigid_body, real* point);
static inline real* rigid_body_get_point_in_world_space(struct RigidBody* rigid_body, real* point);
static inline real* rigid_body_get_direction_in_local_space(struct RigidBody* rigid_body, real* direction);
static inline real* rigid_body_get_direction_in_world_space(struct RigidBody* rigid_body, real* direction);
static inline void rigid_body_set_velocity(struct RigidBody* rigid_body, real* velocity);
static inline void rigid_body_set_velocity_xyz(struct RigidBody* rigid_body, real x, real y, real z);
static inline real* rigid_body_get_velocity(struct RigidBody* rigid_body);
static inline void rigid_body_add_velocity(struct RigidBody* rigid_body, real* delta_velocity);
static inline void rigid_body_set_rotation(struct RigidBody* rigid_body, real* rotation);
static inline void rigid_body_set_rotation_xyz(struct RigidBody* rigid_body, real x, real y, real z);
static inline real* rigid_body_get_rotation(struct RigidBody* rigid_body);
static inline void rigid_body_add_rotation(struct RigidBody* rigid_body, real* delta_rotation);
static inline void rigid_body_set_awake(struct RigidBody* rigid_body, bool awake);
static inline void rigid_body_set_can_sleep(struct RigidBody* rigid_body, bool can_sleep);
static inline real* rigid_body_get_last_frame_acceleration(struct RigidBody* rigid_body);
static inline void rigid_body_clear_accumulators(struct RigidBody* rigid_body);
static inline void rigid_body_add_force(struct RigidBody* rigid_body, real* force);
static inline void rigid_body_add_force_at_body_point(struct RigidBody* rigid_body, real* force, real* point);
static inline void rigid_body_add_force_at_point(struct RigidBody* rigid_body, real* force, real* point);
static inline void rigid_body_add_torque(struct RigidBody* rigid_body, real* torque);
static inline void rigid_body_set_acceleration(struct RigidBody* rigid_body, real* acceleration);
static inline void rigid_body_set_acceleration_xyz(struct RigidBody* rigid_body, real x, real y, real z);
static inline real* rigid_body_get_acceleration(struct RigidBody* rigid_body);
static inline bool rigid_body_get_is_awake(struct RigidBody* rigid_body);
static inline bool rigid_body_get_can_sleep(struct RigidBody* rigid_body);

#endif  // BODY_H
