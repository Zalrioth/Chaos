#pragma once
#ifndef BODY_H
#define BODY_H

#include <float.h>
#include <memory.h>
#include <ubermath/ubermath.h>

#define SLEEP_EPSILON 0.3f

struct RigidBody {
  float inverse_mass;
  mat3 inverse_inertia_tensor;
  float linear_damping;
  float angular_damping;
  vec3 position;
  quat orientation;
  vec3 velocity;
  vec3 rotation;
  mat3 inverse_inertia_tensor_world;
  float motion;
  bool is_awake;
  bool can_sleep;
  mat4 transform_matrix;
  vec3 force_accum;
  vec3 torque_accum;
  vec3 acceleration;
  vec3 last_frame_acceleration;
};

mat3 rigid_body_transform_inertia_tensor(mat3 iit_body, mat4 rotmat);
mat4 rigid_body_calculate_transform_matrix(vec3 position, quat orientation);
void rigid_body_calculate_derived_data(struct RigidBody* rigid_body);
void rigid_body_integrate(struct RigidBody* rigid_body, float duration);
void rigid_body_set_mass(struct RigidBody* rigid_body, float mass);
float rigid_body_get_mass(struct RigidBody* rigid_body);
void rigid_body_set_inverse_mass(struct RigidBody* rigid_body, float inverse_mass);
bool rigid_body_has_finite_mass(struct RigidBody* rigid_body);
void rigid_body_set_inertia_tensor(struct RigidBody* rigid_body, mat3 inertia_tensor);
mat3 rigid_body_get_inertia_tensor(struct RigidBody* rigid_body);
mat3 rigid_body_get_inertia_tensor_world(struct RigidBody* rigid_body);
void rigid_body_set_damping(struct RigidBody* rigid_body, float linear_damping, float angular_damping);
void rigid_body_set_position_xyz(struct RigidBody* rigid_body, float x, float y, float z);
void rigid_body_set_orientation(struct RigidBody* rigid_body, quat orientation);
void rigid_body_set_orientation_rijk(struct RigidBody* rigid_body, float r, float i, float j, float k);
void rigid_body_get_transform_4x4(struct RigidBody* rigid_body, mat4 matrix);
void rigid_body_get_transform_gl4x4(struct RigidBody* rigid_body, mat4 matrix);
vec3 rigid_body_get_point_in_local_space(struct RigidBody* rigid_body, vec3 point);
vec3 rigid_body_get_point_in_world_space(struct RigidBody* rigid_body, vec3 point);
vec3 rigid_body_get_direction_in_local_space(struct RigidBody* rigid_body, vec3 direction);
vec3 rigid_body_get_direction_in_world_space(struct RigidBody* rigid_body, vec3 direction);
void rigid_body_set_velocity_xyz(struct RigidBody* rigid_body, float x, float y, float z);
void rigid_body_add_velocity(struct RigidBody* rigid_body, vec3 delta_velocity);
void rigid_body_set_rotation_xyz(struct RigidBody* rigid_body, float x, float y, float z);
void rigid_body_add_rotation(struct RigidBody* rigid_body, vec3 delta_rotation);
void rigid_body_set_awake(struct RigidBody* rigid_body, bool awake);
void rigid_body_set_can_sleep(struct RigidBody* rigid_body, bool can_sleep);
void rigid_body_clear_accumulators(struct RigidBody* rigid_body);
void rigid_body_add_force(struct RigidBody* rigid_body, vec3 force);
void rigid_body_add_force_at_body_point(struct RigidBody* rigid_body, vec3 force, vec3 point);
void rigid_body_add_force_at_point(struct RigidBody* rigid_body, vec3 force, vec3 point);
void rigid_body_add_torque(struct RigidBody* rigid_body, vec3 torque);
void rigid_body_set_acceleration_xyz(struct RigidBody* rigid_body, float x, float y, float z);

struct BodyRegistration {
  struct RigidBody* body;
  struct BodyRegistration* next;
};

#endif  // BODY_H
