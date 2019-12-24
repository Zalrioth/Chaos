#include "chaos/core/body.h"

void rigid_body_transform_inertia_tensor(real* iit_world, real* q, real* iit_body, real* rotmat) {
  real t4 = rotmat[0] * iit_body[0] + rotmat[1] * iit_body[3] + rotmat[2] * iit_body[6];
  real t9 = rotmat[0] * iit_body[1] + rotmat[1] * iit_body[4] + rotmat[2] * iit_body[7];
  real t14 = rotmat[0] * iit_body[2] + rotmat[1] * iit_body[5] + rotmat[2] * iit_body[8];
  real t28 = rotmat[4] * iit_body[0] + rotmat[5] * iit_body[3] + rotmat[6] * iit_body[6];
  real t33 = rotmat[4] * iit_body[1] + rotmat[5] * iit_body[4] + rotmat[6] * iit_body[7];
  real t38 = rotmat[4] * iit_body[2] + rotmat[5] * iit_body[5] + rotmat[6] * iit_body[8];
  real t52 = rotmat[8] * iit_body[0] + rotmat[9] * iit_body[3] + rotmat[10] * iit_body[6];
  real t57 = rotmat[8] * iit_body[1] + rotmat[9] * iit_body[4] + rotmat[10] * iit_body[7];
  real t62 = rotmat[8] * iit_body[2] + rotmat[9] * iit_body[5] + rotmat[10] * iit_body[8];

  iit_world[0] = t4 * rotmat[0] + t9 * rotmat[1] + t14 * rotmat[2];
  iit_world[1] = t4 * rotmat[4] + t9 * rotmat[5] + t14 * rotmat[6];
  iit_world[2] = t4 * rotmat[8] + t9 * rotmat[9] + t14 * rotmat[10];
  iit_world[3] = t28 * rotmat[0] + t33 * rotmat[1] + t38 * rotmat[2];
  iit_world[4] = t28 * rotmat[4] + t33 * rotmat[5] + t38 * rotmat[6];
  iit_world[5] = t28 * rotmat[8] + t33 * rotmat[9] + t38 * rotmat[10];
  iit_world[6] = t52 * rotmat[0] + t57 * rotmat[1] + t62 * rotmat[2];
  iit_world[7] = t52 * rotmat[4] + t57 * rotmat[5] + t62 * rotmat[6];
  iit_world[8] = t52 * rotmat[8] + t57 * rotmat[9] + t62 * rotmat[10];
}

void rigid_body_calculate_transform_matrix(real* transform_matrix, real* position, real* orientation) {
  transform_matrix[0] = 1 - 2 * orientation[2] * orientation[2] - 2 * orientation[3] * orientation[3];
  transform_matrix[1] = 2 * orientation[1] * orientation[2] - 2 * orientation[0] * orientation[3];
  transform_matrix[2] = 2 * orientation[1] * orientation[3] + 2 * orientation[0] * orientation[2];
  transform_matrix[3] = position[0];

  transform_matrix[4] = 2 * orientation[1] * orientation[2] + 2 * orientation[0] * orientation[3];
  transform_matrix[5] = 1 - 2 * orientation[1] * orientation[1] - 2 * orientation[3] * orientation[3];
  transform_matrix[6] = 2 * orientation[2] * orientation[3] - 2 * orientation[0] * orientation[1];
  transform_matrix[7] = position[1];

  transform_matrix[8] = 2 * orientation[1] * orientation[3] - 2 * orientation[0] * orientation[2];
  transform_matrix[9] = 2 * orientation[2] * orientation[3] + 2 * orientation[0] * orientation[1];
  transform_matrix[10] = 1 - 2 * orientation[1] * orientation[1] - 2 * orientation[2] * orientation[2];
  transform_matrix[11] = position[2];
}

void rigid_body_calculate_derived_data(struct RigidBody* rigid_body) {
  quaternion_normalise(rigid_body->orientation);
  rigid_body_calculate_transform_matrix(rigid_body->transform_matrix, rigid_body->position, rigid_body->orientation);
  rigid_body_transform_inertia_tensor(rigid_body->inverse_inertia_tensor_world, rigid_body->orientation, rigid_body->inverse_inertia_tensor, rigid_body->transform_matrix);
}

void rigid_body_integrate(struct RigidBody* rigid_body, real duration) {
  if (!rigid_body->is_awake)
    return;

  vec3_copy(rigid_body->last_frame_acceleration, rigid_body->acceleration);
  vec3_copy(rigid_body->last_frame_acceleration, vec3_add_scaled_vector(rigid_body->last_frame_acceleration, rigid_body->force_accum, rigid_body->inverse_mass));

  real* angular_acceleration = mat3_transform(rigid_body->inverse_inertia_tensor_world, rigid_body->torque_accum);

  vec3_copy(rigid_body->velocity, vec3_add_scaled_vector(rigid_body->velocity, rigid_body->last_frame_acceleration, duration));
  vec3_copy(rigid_body->rotation, vec3_add_scaled_vector(rigid_body->rotation, angular_acceleration, duration));

  vec3_copy(rigid_body->velocity, vec3_mul_scalar(rigid_body->velocity, real_pow(rigid_body->linear_damping, duration)));
  vec3_copy(rigid_body->rotation, vec3_mul_scalar(rigid_body->velocity, real_pow(rigid_body->angular_damping, duration)));

  vec3_copy(rigid_body->position, vec3_add_scaled_vector(rigid_body->position, rigid_body->velocity, duration));
  vec3_copy(rigid_body->orientation, vec3_add_scaled_vector(rigid_body->rotation, rigid_body->velocity, duration));

  rigid_body_calculate_derived_data(rigid_body);
  rigid_body_clear_accumulators(rigid_body);

  if (rigid_body->can_sleep) {
    real current_motion = vec3_scalar_product(rigid_body->velocity, rigid_body->velocity) + vec3_scalar_product(rigid_body->rotation, rigid_body->rotation);
    real bias = real_pow(0.5, duration);
    rigid_body->motion = bias * rigid_body->motion + (1 - bias) * current_motion;

    if (rigid_body->motion < sleep_epsilon)
      rigid_body_set_awake(rigid_body, false);
    else if (rigid_body->motion > 10 * sleep_epsilon)
      rigid_body->motion = 10 * sleep_epsilon;
  }
}

void rigid_body_set_mass(struct RigidBody* rigid_body, real mass) {
  rigid_body->inverse_mass = ((real)1.0) / mass;
}

real rigid_body_get_mass(struct RigidBody* rigid_body) {
  if (rigid_body->inverse_mass == 0)
    return REAL_MAX;
  else
    return ((real)1.0) / rigid_body->inverse_mass;
}

void rigid_body_set_inverse_mass(struct RigidBody* rigid_body, real inverse_mass) {
  rigid_body->inverse_mass = inverse_mass;
}

real rigid_body_get_inverse_mass(struct RigidBody* rigid_body) {
  return rigid_body->inverse_mass;
}

bool rigid_body_has_finite_mass(struct RigidBody* rigid_body) {
  return rigid_body->inverse_mass >= 0.0f;
}

void rigid_body_set_inertia_tensor(struct RigidBody* rigid_body, real* inertia_tensor) {
  mat3_copy(rigid_body->inverse_inertia_tensor, mat3_inverse(inertia_tensor));
}

real* rigid_body_get_inertia_tensor(struct RigidBody* rigid_body) {
  return mat3_inverse(rigid_body->inverse_inertia_tensor);
}

real* rigid_body_get_inertia_tensor_world(struct RigidBody* rigid_body) {
  return mat3_inverse(rigid_body->inverse_inertia_tensor_world);
}

void rigid_body_set_inverse_inertia_tensor(struct RigidBody* rigid_body, real* inverse_inertia_tensor) {
  mat3_copy(rigid_body->inverse_inertia_tensor, inverse_inertia_tensor);
}

real* rigid_body_get_inverse_inertia_tensor(struct RigidBody* rigid_body) {
  return rigid_body->inverse_inertia_tensor;
}

real* rigid_body_get_inverse_inertia_tensor_world(struct RigidBody* rigid_body) {
  return rigid_body->inverse_inertia_tensor_world;
}

void rigid_body_set_damping(struct RigidBody* rigid_body, real linear_damping, real angular_damping) {
  rigid_body->linear_damping = linear_damping;
  rigid_body->angular_damping = angular_damping;
}

void rigid_body_set_linear_damping(struct RigidBody* rigid_body, real linear_damping) {
  rigid_body->linear_damping = linear_damping;
}

real rigid_body_get_linear_damping(struct RigidBody* rigid_body) {
  return rigid_body->linear_damping;
}

void rigid_body_set_angular_damping(struct RigidBody* rigid_body, real angular_damping) {
  rigid_body->angular_damping = angular_damping;
}

real rigid_body_get_angular_damping(struct RigidBody* rigid_body) {
  return rigid_body->angular_damping;
}

void rigid_body_set_position(struct RigidBody* rigid_body, real* position) {
  vec3_copy(rigid_body->position, position);
}

void rigid_body_set_position_xyz(struct RigidBody* rigid_body, real x, real y, real z) {
  rigid_body->position[0] = x;
  rigid_body->position[1] = y;
  rigid_body->position[2] = z;
}

real* rigid_body_get_position(struct RigidBody* rigid_body) {
  return rigid_body->position;
}

void rigid_body_set_orientation(struct RigidBody* rigid_body, real* orientation) {
  quaternion_copy(rigid_body->orientation, quaternion_normalise(orientation));
}

void rigid_body_set_orientation_rijk(struct RigidBody* rigid_body, real r, real i, real j, real k) {
  quaternion_copy(rigid_body->orientation, quaternion_normalise((quaternion){r, i, j, k}));
}

real* rigid_body_get_orientation(struct RigidBody* rigid_body) {
  return rigid_body->orientation;
}

real* rigid_body_get_transform(struct RigidBody* rigid_body) {
  return rigid_body->transform_matrix;
}

void rigid_body_get_transform_4x4(struct RigidBody* rigid_body, real* matrix) {
  memcpy(matrix, rigid_body->transform_matrix, sizeof(mat4));
  matrix[12] = matrix[13] = matrix[14] = 0;
  matrix[15] = 1;
}

void rigid_body_get_transform_gl4x4(struct RigidBody* rigid_body, real* matrix) {
  matrix[0] = (float)rigid_body->transform_matrix[0];
  matrix[1] = (float)rigid_body->transform_matrix[4];
  matrix[2] = (float)rigid_body->transform_matrix[8];
  matrix[3] = 0;

  matrix[4] = (float)rigid_body->transform_matrix[1];
  matrix[5] = (float)rigid_body->transform_matrix[5];
  matrix[6] = (float)rigid_body->transform_matrix[9];
  matrix[7] = 0;

  matrix[8] = (float)rigid_body->transform_matrix[2];
  matrix[9] = (float)rigid_body->transform_matrix[6];
  matrix[10] = (float)rigid_body->transform_matrix[10];
  matrix[11] = 0;

  matrix[12] = (float)rigid_body->transform_matrix[3];
  matrix[13] = (float)rigid_body->transform_matrix[7];
  matrix[14] = (float)rigid_body->transform_matrix[11];
  matrix[15] = 1;
}

real* rigid_body_get_point_in_local_space(struct RigidBody* rigid_body, real* point) {
  return mat4_transform_inverse(rigid_body->transform_matrix, point);
}

real* rigid_body_get_point_in_world_space(struct RigidBody* rigid_body, real* point) {
  return mat4_transform(rigid_body->transform_matrix, point);
}

real* rigid_body_get_direction_in_local_space(struct RigidBody* rigid_body, real* direction) {
  return mat4_transform_inverse_direction(rigid_body->transform_matrix, direction);
}

real* rigid_body_get_direction_in_world_space(struct RigidBody* rigid_body, real* direction) {
  return mat4_transform_direction(rigid_body->transform_matrix, direction);
}

void rigid_body_set_velocity(struct RigidBody* rigid_body, real* velocity) {
  vec3_copy(rigid_body->velocity, velocity);
}

void rigid_body_set_velocity_xyz(struct RigidBody* rigid_body, real x, real y, real z) {
  vec3_copy(rigid_body->velocity, (vec3){x, y, z});
}

real* rigid_body_get_velocity(struct RigidBody* rigid_body) {
  return rigid_body->velocity;
}

void rigid_body_add_velocity(struct RigidBody* rigid_body, real* delta_velocity) {
  vec3_copy(rigid_body->velocity, vec3_add(rigid_body->velocity, delta_velocity));
}

void rigid_body_set_rotation(struct RigidBody* rigid_body, real* rotation) {
  vec3_copy(rigid_body->rotation, rotation);
}

void rigid_body_set_rotation_xyz(struct RigidBody* rigid_body, real x, real y, real z) {
  vec3_copy(rigid_body->rotation, (vec3){x, y, z});
}

real* rigid_body_get_rotation(struct RigidBody* rigid_body) {
  return rigid_body->rotation;
}

void rigid_body_add_rotation(struct RigidBody* rigid_body, real* delta_rotation) {
  vec3_copy(rigid_body->rotation, vec3_add(rigid_body->rotation, delta_rotation));
}

void rigid_body_set_awake(struct RigidBody* rigid_body, bool awake) {
  if (awake) {
    rigid_body->is_awake = true;
    rigid_body->motion = sleep_epsilon * 2.0f;
  } else {
    rigid_body->is_awake = false;
    vec3_clear(rigid_body->velocity);
    vec3_clear(rigid_body->rotation);
  }
}

void rigid_body_set_can_sleep(struct RigidBody* rigid_body, bool can_sleep) {
  rigid_body->can_sleep = can_sleep;

  if (!rigid_body->can_sleep && !rigid_body->is_awake)
    rigid_body_set_awake(rigid_body, true);
}

real* rigid_body_get_last_frame_acceleration(struct RigidBody* rigid_body) {
  return rigid_body->last_frame_acceleration;
}

void rigid_body_clear_accumulators(struct RigidBody* rigid_body) {
  vec3_clear(rigid_body->force_accum);
  vec3_clear(rigid_body->torque_accum);
}

void rigid_body_add_force(struct RigidBody* rigid_body, real* force) {
  vec3_copy(rigid_body->force_accum, vec3_add(rigid_body->force_accum, force));
  rigid_body->is_awake = true;
}

void rigid_body_add_force_at_body_point(struct RigidBody* rigid_body, real* force, real* point) {
  real* pt = rigid_body_get_point_in_world_space(rigid_body, point);
  rigid_body_add_force_at_point(rigid_body, force, pt);
}

void rigid_body_add_force_at_point(struct RigidBody* rigid_body, real* force, real* point) {
  real* pt = point;
  vec3_copy(pt, vec3_sub(pt, rigid_body->position));

  vec3_copy(rigid_body->force_accum, vec3_add(rigid_body->force_accum, force));
  vec3_copy(rigid_body->torque_accum, vec3_add(rigid_body->torque_accum, vec3_cross_product(pt, force)));

  rigid_body->is_awake = true;
}

void rigid_body_add_torque(struct RigidBody* rigid_body, real* torque) {
  vec3_copy(rigid_body->torque_accum, vec3_add(rigid_body->torque_accum, torque));
  rigid_body->is_awake = true;
}

void rigid_body_set_acceleration(struct RigidBody* rigid_body, real* acceleration) {
  vec3_copy(rigid_body->acceleration, acceleration);
}

void rigid_body_set_acceleration_xyz(struct RigidBody* rigid_body, real x, real y, real z) {
  vec3_copy(rigid_body->acceleration, (vec3){x, y, z});
}

real* rigid_body_get_acceleration(struct RigidBody* rigid_body) {
  return rigid_body->acceleration;
}

bool rigid_body_get_is_awake(struct RigidBody* rigid_body) {
  return rigid_body->is_awake;
}

bool rigid_body_get_can_sleep(struct RigidBody* rigid_body) {
  return rigid_body->can_sleep;
}
