#include "chaos/core/body.h"

mat3 rigid_body_transform_inertia_tensor(mat3 iit_body, mat4 rotmat) {
  float t4 = rotmat.data[0] * iit_body.data[0] + rotmat.data[1] * iit_body.data[3] + rotmat.data[2] * iit_body.data[6];
  float t9 = rotmat.data[0] * iit_body.data[1] + rotmat.data[1] * iit_body.data[4] + rotmat.data[2] * iit_body.data[7];
  float t14 = rotmat.data[0] * iit_body.data[2] + rotmat.data[1] * iit_body.data[5] + rotmat.data[2] * iit_body.data[8];
  float t28 = rotmat.data[4] * iit_body.data[0] + rotmat.data[5] * iit_body.data[3] + rotmat.data[6] * iit_body.data[6];
  float t33 = rotmat.data[4] * iit_body.data[1] + rotmat.data[5] * iit_body.data[4] + rotmat.data[6] * iit_body.data[7];
  float t38 = rotmat.data[4] * iit_body.data[2] + rotmat.data[5] * iit_body.data[5] + rotmat.data[6] * iit_body.data[8];
  float t52 = rotmat.data[8] * iit_body.data[0] + rotmat.data[9] * iit_body.data[3] + rotmat.data[10] * iit_body.data[6];
  float t57 = rotmat.data[8] * iit_body.data[1] + rotmat.data[9] * iit_body.data[4] + rotmat.data[10] * iit_body.data[7];
  float t62 = rotmat.data[8] * iit_body.data[2] + rotmat.data[9] * iit_body.data[5] + rotmat.data[10] * iit_body.data[8];

  return (mat3){.data[0] = t4 * rotmat.data[0] + t9 * rotmat.data[1] + t14 * rotmat.data[2],
                .data[1] = t4 * rotmat.data[4] + t9 * rotmat.data[5] + t14 * rotmat.data[6],
                .data[2] = t4 * rotmat.data[8] + t9 * rotmat.data[9] + t14 * rotmat.data[10],
                .data[3] = t28 * rotmat.data[0] + t33 * rotmat.data[1] + t38 * rotmat.data[2],
                .data[4] = t28 * rotmat.data[4] + t33 * rotmat.data[5] + t38 * rotmat.data[6],
                .data[5] = t28 * rotmat.data[8] + t33 * rotmat.data[9] + t38 * rotmat.data[10],
                .data[6] = t52 * rotmat.data[0] + t57 * rotmat.data[1] + t62 * rotmat.data[2],
                .data[7] = t52 * rotmat.data[4] + t57 * rotmat.data[5] + t62 * rotmat.data[6],
                .data[8] = t52 * rotmat.data[8] + t57 * rotmat.data[9] + t62 * rotmat.data[10]};
}

mat4 rigid_body_calculate_transform_matrix(vec3 position, quat orientation) {
  return (mat4){
      .data[0] = 1 - 2 * orientation.data[1] * orientation.data[1] - 2 * orientation.data[2] * orientation.data[2],
      .data[1] = 2 * orientation.data[0] * orientation.data[1] - 2 * orientation.data[3] * orientation.data[2],
      .data[2] = 2 * orientation.data[0] * orientation.data[2] + 2 * orientation.data[3] * orientation.data[1],
      .data[3] = position.data[0],

      .data[4] = 2 * orientation.data[0] * orientation.data[1] + 2 * orientation.data[3] * orientation.data[2],
      .data[5] = 1 - 2 * orientation.data[0] * orientation.data[0] - 2 * orientation.data[2] * orientation.data[2],
      .data[6] = 2 * orientation.data[1] * orientation.data[2] - 2 * orientation.data[3] * orientation.data[0],
      .data[7] = position.data[1],

      .data[8] = 2 * orientation.data[0] * orientation.data[2] - 2 * orientation.data[3] * orientation.data[1],
      .data[9] = 2 * orientation.data[1] * orientation.data[2] + 2 * orientation.data[3] * orientation.data[0],
      .data[10] = 1 - 2 * orientation.data[0] * orientation.data[0] - 2 * orientation.data[1] * orientation.data[1],
      .data[11] = position.data[2],

      .data[12] = 0.0f,
      .data[13] = 0.0f,
      .data[14] = 0.0f,
      .data[15] = 1.0f};
}

void rigid_body_calculate_derived_data(struct RigidBody* rigid_body) {
  rigid_body->orientation = quaternion_normalise(rigid_body->orientation);
  rigid_body->transform_matrix = rigid_body_calculate_transform_matrix(rigid_body->position, rigid_body->orientation);
  rigid_body->inverse_inertia_tensor_world = rigid_body_transform_inertia_tensor(rigid_body->inverse_inertia_tensor, rigid_body->transform_matrix);
}

void rigid_body_integrate(struct RigidBody* rigid_body, float duration) {
  if (!rigid_body->is_awake)
    return;

  rigid_body->last_frame_acceleration = rigid_body->acceleration;
  rigid_body->last_frame_acceleration = vec3_add_scaled_vector(rigid_body->last_frame_acceleration, rigid_body->force_accum, rigid_body->inverse_mass);

  vec3 angular_acceleration = mat3_transform(rigid_body->inverse_inertia_tensor_world, rigid_body->torque_accum);

  rigid_body->velocity = vec3_add_scaled_vector(rigid_body->velocity, rigid_body->last_frame_acceleration, duration);
  rigid_body->rotation = vec3_add_scaled_vector(rigid_body->rotation, angular_acceleration, duration);

  rigid_body->velocity = vec3_scale(rigid_body->velocity, powf(rigid_body->linear_damping, duration));
  rigid_body->rotation = vec3_scale(rigid_body->rotation, powf(rigid_body->angular_damping, duration));

  rigid_body->position = vec3_add_scaled_vector(rigid_body->position, rigid_body->velocity, duration);
  rigid_body->orientation = quaternion_add_scaled_vector(rigid_body->orientation, rigid_body->rotation, duration);

  rigid_body_calculate_derived_data(rigid_body);
  rigid_body_clear_accumulators(rigid_body);

  if (rigid_body->can_sleep) {
    float current_motion = vec3_magnitude(rigid_body->velocity) + vec3_magnitude(rigid_body->rotation);
    float bias = powf(0.5, duration);
    rigid_body->motion = bias * rigid_body->motion + (1 - bias) * current_motion;

    if (rigid_body->motion < SLEEP_EPSILON)
      rigid_body_set_awake(rigid_body, false);
    else if (rigid_body->motion > 10 * SLEEP_EPSILON)
      rigid_body->motion = 10 * SLEEP_EPSILON;
  }
}

void rigid_body_set_mass(struct RigidBody* rigid_body, float mass) {
  rigid_body->inverse_mass = 1.0f / mass;
}

float rigid_body_get_mass(struct RigidBody* rigid_body) {
  if (rigid_body->inverse_mass == 0)
    return FLT_MAX;
  else
    return 1.0f / rigid_body->inverse_mass;
}

void rigid_body_set_inverse_mass(struct RigidBody* rigid_body, float inverse_mass) {
  rigid_body->inverse_mass = inverse_mass;
}

bool rigid_body_has_finite_mass(struct RigidBody* rigid_body) {
  return rigid_body->inverse_mass >= 0.0f;
}

void rigid_body_set_inertia_tensor(struct RigidBody* rigid_body, mat3 inertia_tensor) {
  rigid_body->inverse_inertia_tensor = mat3_inverse(inertia_tensor);
}

mat3 rigid_body_get_inertia_tensor(struct RigidBody* rigid_body) {
  return mat3_inverse(rigid_body->inverse_inertia_tensor);
}

void rigid_body_set_damping(struct RigidBody* rigid_body, float linear_damping, float angular_damping) {
  rigid_body->linear_damping = linear_damping;
  rigid_body->angular_damping = angular_damping;
}

void rigid_body_set_position_xyz(struct RigidBody* rigid_body, float x, float y, float z) {
  rigid_body->position.x = x;
  rigid_body->position.y = y;
  rigid_body->position.z = z;
}

void rigid_body_set_orientation_rijk(struct RigidBody* rigid_body, float r, float i, float j, float k) {
  rigid_body->orientation = quaternion_normalise((quat){.data[0] = r, .data[1] = i, .data[2] = j, .data[3] = k});
}

void rigid_body_get_transform_4x4(struct RigidBody* rigid_body, mat4 matrix) {
  rigid_body->transform_matrix = matrix;
  matrix.data[12] = matrix.data[13] = matrix.data[14] = 0;
  matrix.data[15] = 1;
}

void rigid_body_get_transform_gl4x4(struct RigidBody* rigid_body, mat4 matrix) {
  matrix.data[0] = (float)rigid_body->transform_matrix.data[0];
  matrix.data[1] = (float)rigid_body->transform_matrix.data[4];
  matrix.data[2] = (float)rigid_body->transform_matrix.data[8];
  matrix.data[3] = 0;

  matrix.data[4] = (float)rigid_body->transform_matrix.data[1];
  matrix.data[5] = (float)rigid_body->transform_matrix.data[5];
  matrix.data[6] = (float)rigid_body->transform_matrix.data[9];
  matrix.data[7] = 0;

  matrix.data[8] = (float)rigid_body->transform_matrix.data[2];
  matrix.data[9] = (float)rigid_body->transform_matrix.data[6];
  matrix.data[10] = (float)rigid_body->transform_matrix.data[10];
  matrix.data[11] = 0;

  matrix.data[12] = (float)rigid_body->transform_matrix.data[3];
  matrix.data[13] = (float)rigid_body->transform_matrix.data[7];
  matrix.data[14] = (float)rigid_body->transform_matrix.data[11];
  matrix.data[15] = 1;
}

vec3 rigid_body_get_point_in_local_space(struct RigidBody* rigid_body, vec3 point) {
  return mat4_transform_inverse(rigid_body->transform_matrix, point);
}

vec3 rigid_body_get_point_in_world_space(struct RigidBody* rigid_body, vec3 point) {
  return mat4_transform(rigid_body->transform_matrix, point);
}

vec3 rigid_body_get_direction_in_local_space(struct RigidBody* rigid_body, vec3 direction) {
  return mat4_transform_inverse_direction(rigid_body->transform_matrix, direction);
}

vec3 rigid_body_get_direction_in_world_space(struct RigidBody* rigid_body, vec3 direction) {
  return mat4_transform_direction(rigid_body->transform_matrix, direction);
}

void rigid_body_set_velocity_xyz(struct RigidBody* rigid_body, float x, float y, float z) {
  rigid_body->velocity = (vec3){.x = x, .y = y, .z = z};
}

void rigid_body_add_velocity(struct RigidBody* rigid_body, vec3 delta_velocity) {
  rigid_body->velocity = vec3_add(rigid_body->velocity, delta_velocity);
}

void rigid_body_set_rotation_xyz(struct RigidBody* rigid_body, float x, float y, float z) {
  rigid_body->rotation = (vec3){.x = x, .y = y, .z = z};
}

void rigid_body_add_rotation(struct RigidBody* rigid_body, vec3 delta_rotation) {
  rigid_body->rotation = vec3_add(rigid_body->rotation, delta_rotation);
}

void rigid_body_set_awake(struct RigidBody* rigid_body, bool awake) {
  if (awake) {
    rigid_body->is_awake = true;
    rigid_body->motion = SLEEP_EPSILON * 2.0f;
  } else {
    rigid_body->is_awake = false;
    rigid_body->velocity = VEC3_ZERO;
    rigid_body->rotation = VEC3_ZERO;
  }
}

void rigid_body_set_can_sleep(struct RigidBody* rigid_body, bool can_sleep) {
  rigid_body->can_sleep = can_sleep;

  if (!rigid_body->can_sleep && !rigid_body->is_awake)
    rigid_body_set_awake(rigid_body, true);
}

void rigid_body_clear_accumulators(struct RigidBody* rigid_body) {
  rigid_body->force_accum = VEC3_ZERO;
  rigid_body->torque_accum = VEC3_ZERO;
}

void rigid_body_add_force(struct RigidBody* rigid_body, vec3 force) {
  rigid_body->force_accum = vec3_add(rigid_body->force_accum, force);
  rigid_body->is_awake = true;
}

void rigid_body_add_force_at_body_point(struct RigidBody* rigid_body, vec3 force, vec3 point) {
  vec3 pt = rigid_body_get_point_in_world_space(rigid_body, point);
  rigid_body_add_force_at_point(rigid_body, force, pt);
}

void rigid_body_add_force_at_point(struct RigidBody* rigid_body, vec3 force, vec3 point) {
  vec3 pt = point;
  pt = vec3_sub(pt, rigid_body->position);

  rigid_body->force_accum = vec3_add(rigid_body->force_accum, force);
  rigid_body->torque_accum = vec3_add(rigid_body->torque_accum, vec3_cross_product(pt, force));

  rigid_body->is_awake = true;
}

void rigid_body_add_torque(struct RigidBody* rigid_body, vec3 torque) {
  rigid_body->torque_accum = vec3_add(rigid_body->torque_accum, torque);
  rigid_body->is_awake = true;
}

void rigid_body_set_acceleration_xyz(struct RigidBody* rigid_body, float x, float y, float z) {
  rigid_body->acceleration = (vec3){.data[0] = x, .data[1] = y, .data[2] = z};
}
