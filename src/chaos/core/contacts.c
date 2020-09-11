#include "chaos/core/contacts.h"

void contact_set_body_data(struct Contact* contact, struct RigidBody* one, struct RigidBody* two, float friction, float restitution) {
  contact->body[0] = one;
  contact->body[1] = two;
  contact->friction = friction;
  contact->restitution = restitution;
}

void contact_match_awake_state(struct Contact* contact) {
  if (!contact->body[1])
    return;

  bool body0_awake = contact->body[0]->is_awake;
  bool body1_awake = contact->body[1]->is_awake;

  if (body0_awake ^ body1_awake) {
    if (body0_awake)
      rigid_body_set_awake(contact->body[1], true);
    else
      rigid_body_set_awake(contact->body[0], true);
  }
}

void contact_swap_bodies(struct Contact* contact) {
  contact->contact_normal = vec3_invert(contact->contact_normal);

  struct RigidBody* temp = contact->body[0];
  contact->body[0] = contact->body[1];
  contact->body[1] = temp;
}

void contact_calculate_contact_basis(struct Contact* contact) {
  vec3 contact_tangent[2];

  vec3 contact_normal = contact->contact_normal;

  if (sqrtf(contact_normal.data[0]) > sqrtf(contact_normal.data[1])) {
    const float s = 1.0f / sqrtf(contact_normal.data[2] * contact_normal.data[2] + contact_normal.data[0] * contact_normal.data[0]);

    contact_tangent[0].data[0] = contact_normal.data[2] * s;
    contact_tangent[0].data[1] = 0;
    contact_tangent[0].data[2] = -contact_normal.data[0] * s;

    contact_tangent[1].data[0] = contact_normal.data[1] * contact_tangent[0].data[0];
    contact_tangent[1].data[1] = contact_normal.data[2] * contact_tangent[0].data[0] - contact_normal.data[0] * contact_tangent[0].data[2];
    contact_tangent[1].data[2] = -contact_normal.data[1] * contact_tangent[0].data[0];
  } else {
    const float s = 1.0f / sqrtf(contact_normal.data[2] * contact_normal.data[2] + contact_normal.data[1] * contact_normal.data[1]);

    contact_tangent[0].data[0] = 0;
    contact_tangent[0].data[1] = -contact_normal.data[2] * s;
    contact_tangent[0].data[2] = contact_normal.data[1] * s;

    contact_tangent[1].data[0] = contact_normal.data[1] * contact_tangent[0].data[2] - contact_normal.data[2] * contact_tangent[0].data[2];
    contact_tangent[1].data[1] = -contact_normal.data[0] * contact_tangent[0].data[2];
    contact_tangent[1].data[2] = contact_normal.data[0] * contact_tangent[0].data[1];
  }
  contact->contact_to_world = (mat3){
      .data[0] = contact_normal.data[0],
      .data[1] = contact_tangent[0].data[0],
      .data[2] = contact_tangent[1].data[0],

      .data[3] = contact_normal.data[1],
      .data[4] = contact_tangent[0].data[1],
      .data[5] = contact_tangent[1].data[1],

      .data[6] = contact_normal.data[2],
      .data[7] = contact_tangent[0].data[2],
      .data[8] = contact_tangent[1].data[2],
  };
  //contact->contact_to_world = (mat3){.m00 = contact_normal.data[0], .m01 = contact_normal.data[1], .m02 = contact_normal.data[2], .m10 = contact_tangent[0].data[0], .m11 = contact_tangent[0].data[1], .m12 = contact_tangent[0].data[2], .m20 = contact_tangent[1].data[0], .m21 = contact_tangent[1].data[1], .m22 = contact_tangent[1].data[2]};
  //contact->contact_to_world = (mat3){.vecs[0] = contact_normal, .vecs[1] = contact_tangent[0], .vecs[2] = contact_tangent[1]};
}

vec3 contact_calculate_local_velocity(struct Contact* contact, unsigned int body_index, float duration) {
  struct RigidBody* this_body = contact->body[body_index];

  vec3 velocity = vec3_cross_product(this_body->rotation, contact->relative_contact_position[body_index]);
  velocity = vec3_add(velocity, this_body->velocity);

  vec3 contact_velocity = mat3_transform_transpose(contact->contact_to_world, velocity);

  vec3 acc_velocity = vec3_scale(this_body->last_frame_acceleration, duration);
  acc_velocity = mat3_transform_transpose(contact->contact_to_world, acc_velocity);
  acc_velocity.data[0] = 0;

  return vec3_add(contact_velocity, acc_velocity);
  //contact_velocity = vec3_add(contact_velocity, acc_velocity);
  //return (vec3){.data[0] = contact_velocity.data[0], .data[1] = contact_velocity.data[1], .data[2] = contact_velocity.data[2]};
}

void contact_calculate_desired_delta_velocity(struct Contact* contact, float duration) {
  float velocity_from_acc = 0;

  if (contact->body[0]->is_awake)
    velocity_from_acc += vec3_dot(vec3_scale(contact->body[0]->last_frame_acceleration, duration), contact->contact_normal);
  //velocity_from_acc += vec3_magnitude(vec3_component_product(vec3_scale(contact->body[0]->last_frame_acceleration, duration), contact->contact_normal));

  if (contact->body[1] && contact->body[1]->is_awake)
    velocity_from_acc -= vec3_dot(vec3_scale(contact->body[1]->last_frame_acceleration, duration), contact->contact_normal);

  float this_restitution = contact->restitution;
  if (fabsf(contact->contact_velocity.data[0]) < VELOCITY_LIMIT)
    this_restitution = 0.0f;

  contact->desired_delta_velocity = -contact->contact_velocity.data[0] - this_restitution * (contact->contact_velocity.data[0] - velocity_from_acc);
}

void contact_calculate_internals(struct Contact* contact, float duration) {
  if (!contact->body[0])
    contact_swap_bodies(contact);

  contact_calculate_contact_basis(contact);
  contact->relative_contact_position[0] = vec3_sub(contact->contact_point, contact->body[0]->position);

  if (contact->body[1])
    contact->relative_contact_position[1] = vec3_sub(contact->contact_point, contact->body[1]->position);

  contact->contact_velocity = contact_calculate_local_velocity(contact, 0, duration);
  if (contact->body[1])
    contact->contact_velocity = vec3_sub(contact->contact_velocity, contact_calculate_local_velocity(contact, 1, duration));

  contact_calculate_desired_delta_velocity(contact, duration);
}

// NOTE: Watch out for this typedef pointer
void contact_apply_velocity_change(struct Contact* contact, vec3 velocity_change[2], vec3 rotation_change[2]) {
  mat3 inverse_inertia_tensor[2];
  inverse_inertia_tensor[0] = contact->body[0]->inverse_inertia_tensor_world;
  if (contact->body[1])
    inverse_inertia_tensor[1] = contact->body[1]->inverse_inertia_tensor_world;

  vec3 impulse_contact;
  if (contact->friction == 0.0f)
    impulse_contact = contact_calculate_frictionless_impulse(contact, inverse_inertia_tensor);
  else
    impulse_contact = contact_calculate_friction_impulse(contact, inverse_inertia_tensor);

  vec3 impulse = mat3_transform(contact->contact_to_world, impulse_contact);
  vec3 impulsive_torque = vec3_cross_product(contact->relative_contact_position[0], impulse);

  rotation_change[0] = mat3_transform(inverse_inertia_tensor[0], impulsive_torque);
  velocity_change[0] = VEC3_ZERO;
  velocity_change[0] = vec3_add_scaled_vector(velocity_change[0], impulse, contact->body[0]->inverse_mass);

  rigid_body_add_velocity(contact->body[0], velocity_change[0]);
  rigid_body_add_rotation(contact->body[0], rotation_change[0]);

  if (contact->body[1]) {
    vec3 impulsive_torque = vec3_cross_product(impulse, contact->relative_contact_position[1]);

    rotation_change[1] = mat3_transform(inverse_inertia_tensor[1], impulsive_torque);
    velocity_change[1] = VEC3_ZERO;
    velocity_change[1] = vec3_add_scaled_vector(velocity_change[1], impulse, -contact->body[1]->inverse_mass);

    rigid_body_add_velocity(contact->body[1], velocity_change[1]);
    rigid_body_add_rotation(contact->body[1], rotation_change[1]);
  }
}

vec3 contact_calculate_frictionless_impulse(struct Contact* contact, mat3 inverse_inertia_tensor[2]) {
  vec3 delta_vel_world = vec3_cross_product(contact->relative_contact_position[0], contact->contact_normal);
  delta_vel_world = mat3_transform(inverse_inertia_tensor[0], delta_vel_world);
  delta_vel_world = vec3_cross_product(delta_vel_world, contact->relative_contact_position[0]);
  float delta_velocity = vec3_magnitude(vec3_component_product(delta_vel_world, contact->contact_normal));
  delta_velocity += contact->body[0]->inverse_mass;

  if (contact->body[1]) {
    vec3 delta_vel_world_other = vec3_cross_product(contact->relative_contact_position[1], contact->contact_normal);
    delta_vel_world_other = mat3_transform(inverse_inertia_tensor[1], delta_vel_world_other);
    delta_vel_world_other = vec3_cross_product(delta_vel_world_other, contact->relative_contact_position[1]);
    delta_velocity += vec3_magnitude(vec3_component_product(delta_vel_world_other, contact->contact_normal));
    delta_velocity += contact->body[1]->inverse_mass;
  }

  return (vec3){.data[0] = contact->desired_delta_velocity / delta_velocity, .data[1] = 0, .data[2] = 0};
}

vec3 contact_calculate_friction_impulse(struct Contact* contact, mat3 inverse_inertia_tensor[2]) {
  float inverse_mass = contact->body[0]->inverse_mass;

  mat3 impulse_to_torque = MAT3_ZERO;
  impulse_to_torque = mat3_skew_symmetric(impulse_to_torque, contact->relative_contact_position[0]);

  mat3 delta_vel_world = impulse_to_torque;
  delta_vel_world = mat3_mul_mat3(delta_vel_world, inverse_inertia_tensor[0]);
  delta_vel_world = mat3_mul_mat3(delta_vel_world, impulse_to_torque);
  delta_vel_world = mat3_mul_scalar(delta_vel_world, -1);

  if (contact->body[1]) {
    impulse_to_torque = mat3_skew_symmetric(impulse_to_torque, contact->relative_contact_position[1]);

    mat3 delta_vel_world_other = impulse_to_torque;
    delta_vel_world_other = mat3_mul_mat3(delta_vel_world, inverse_inertia_tensor[1]);
    delta_vel_world_other = mat3_mul_mat3(delta_vel_world, impulse_to_torque);
    delta_vel_world_other = mat3_mul_scalar(delta_vel_world, -1);

    delta_vel_world = mat3_add_mat3(delta_vel_world, delta_vel_world_other);

    inverse_mass += contact->body[1]->inverse_mass;
  }

  mat3 delta_velocity = mat3_transpose(contact->contact_to_world);
  delta_velocity = mat3_mul_mat3(delta_velocity, delta_vel_world);
  delta_velocity = mat3_mul_mat3(delta_velocity, contact->contact_to_world);

  delta_velocity.data[0] += inverse_mass;
  delta_velocity.data[4] += inverse_mass;
  delta_velocity.data[8] += inverse_mass;

  mat3 impulse_matrix = mat3_inverse(delta_velocity);

  vec3 vel_kill = (vec3){.data[0] = contact->desired_delta_velocity, .data[1] = -contact->contact_velocity.data[1], .data[2] = -contact->contact_velocity.data[2]};

  vec3 impulse_contact = mat3_transform(impulse_matrix, vel_kill);

  float planar_impulse = sqrtf(impulse_contact.data[1] * impulse_contact.data[1] + impulse_contact.data[2] * impulse_contact.data[2]);
  if (planar_impulse > impulse_contact.data[0] * contact->friction) {
    impulse_contact.data[1] /= planar_impulse;
    impulse_contact.data[2] /= planar_impulse;

    impulse_contact.data[0] = delta_velocity.data[0] + delta_velocity.data[1] * contact->friction * impulse_contact.data[1] + delta_velocity.data[2] * contact->friction * impulse_contact.data[2];
    impulse_contact.data[0] = contact->desired_delta_velocity / impulse_contact.data[0];
    impulse_contact.data[1] *= contact->friction * impulse_contact.data[0];
    impulse_contact.data[2] *= contact->friction * impulse_contact.data[0];
  }
  return impulse_contact;
  //return (vec3){.data[0] = impulse_contact.data[0], .data[1] = impulse_contact.data[1], .data[2] = impulse_contact.data[0]};
}

void contact_apply_position_change(struct Contact* contact, vec3 linear_change[2], vec3 angular_change[2], float penetration) {
  float angular_limit = 0.2f;
  float angular_move[2];
  float linear_move[2];

  float total_inertia = 0;
  float linear_inertia[2];
  float angular_inertia[2];

  for (unsigned int i = 0; i < 2; i++)
    if (contact->body[i]) {
      mat3 inverse_inertia_tensor = contact->body[i]->inverse_inertia_tensor_world;

      vec3 angular_inertia_world = vec3_cross_product(contact->relative_contact_position[i], contact->contact_normal);
      angular_inertia_world = mat3_transform(inverse_inertia_tensor, angular_inertia_world);
      angular_inertia_world = vec3_cross_product(angular_inertia_world, contact->relative_contact_position[i]);

      angular_inertia[i] = vec3_dot(angular_inertia_world, contact->contact_normal);
      linear_inertia[i] = contact->body[i]->inverse_mass;
      total_inertia += linear_inertia[i] + angular_inertia[i];
    }

  for (unsigned int i = 0; i < 2; i++)
    if (contact->body[i]) {
      float sign = (i == 0) ? 1 : -1;
      angular_move[i] = sign * penetration * (angular_inertia[i] / total_inertia);
      linear_move[i] = sign * penetration * (linear_inertia[i] / total_inertia);

      vec3 projection = contact->relative_contact_position[i];
      projection = vec3_add_scaled_vector(projection, contact->contact_normal, -vec3_dot(contact->relative_contact_position[i], contact->contact_normal));

      float max_magnitude = angular_limit * vec3_magnitude(projection);

      if (angular_move[i] < -max_magnitude) {
        float totalMove = angular_move[i] + linear_move[i];
        angular_move[i] = -max_magnitude;
        linear_move[i] = totalMove - angular_move[i];
      } else if (angular_move[i] > max_magnitude) {
        float totalMove = angular_move[i] + linear_move[i];
        angular_move[i] = max_magnitude;
        linear_move[i] = totalMove - angular_move[i];
      }

      if (angular_move[i] == 0)
        angular_change[i] = VEC3_ZERO;
      else {
        vec3 target_angular_direction = vec3_cross_product(contact->relative_contact_position[i], contact->contact_normal);
        mat3 inverse_inertia_tensor = rigid_body_get_inertia_tensor_world(contact->body[i]);
        angular_change[i] = vec3_scale(mat3_transform(inverse_inertia_tensor, target_angular_direction), (angular_move[i] / angular_inertia[i]));
      }

      linear_change[i] = vec3_scale(contact->contact_normal, linear_move[i]);

      contact->body[i]->position = vec3_add_scaled_vector(contact->body[i]->position, contact->contact_normal, linear_move[i]);

      rigid_body_set_orientation(contact->body[i], quaternion_add_scaled_vector(contact->body[i]->orientation, angular_change[i], 1.0f));

      if (!contact->body[i]->is_awake)
        rigid_body_calculate_derived_data(contact->body[i]);
    }
}

////////////////////////////////////////////////////////////////////////////////////////

void contact_resolver_init(struct ContactResolver* contact_resolver, unsigned int velocity_iterations, unsigned int position_iterations, float velocity_epsilon, float position_epsilon) {
  contact_resolver_set_iterations(contact_resolver, velocity_iterations, position_iterations);
  contact_resolver_set_epsilon(contact_resolver, velocity_epsilon, position_epsilon);
}

bool contact_resolver_is_valid(struct ContactResolver* contact_resolver) {
  return (contact_resolver->velocity_iterations > 0) && (contact_resolver->position_iterations > 0) && (contact_resolver->velocity_epsilon >= 0.0f) && (contact_resolver->position_epsilon >= 0.0f);
}

void contact_resolver_set_iterations(struct ContactResolver* contact_resolver, unsigned int velocity_iterations, unsigned int position_iterations) {
  contact_resolver->velocity_iterations = velocity_iterations;
  contact_resolver->position_iterations = position_iterations;
}

void contact_resolver_set_epsilon(struct ContactResolver* contact_resolver, float velocity_epsilon, float position_epsilon) {
  contact_resolver->velocity_epsilon = velocity_epsilon;
  contact_resolver->position_epsilon = position_epsilon;
}

void contact_resolver_resolve_contacts(struct ContactResolver* contact_resolver, struct Contact* contacts, unsigned int num_contacts, float duration) {
  if (num_contacts == 0)
    return;
  if (!contact_resolver_is_valid(contact_resolver))
    return;

  contact_resolver_prepare_contacts(contact_resolver, contacts, num_contacts, duration);
  contact_resolver_adjust_positions(contact_resolver, contacts, num_contacts, duration);
  contact_resolver_adjust_velocities(contact_resolver, contacts, num_contacts, duration);
}

void contact_resolver_prepare_contacts(struct ContactResolver* contact_resolver, struct Contact* contacts, unsigned int num_contacts, float duration) {
  struct Contact* last_contact = contacts + num_contacts;
  for (struct Contact* contact = contacts; contact < last_contact; contact++) {
    contact_calculate_internals(contact, duration);
  }
}

void contact_resolver_adjust_velocities(struct ContactResolver* contact_resolver, struct Contact* contact, unsigned int num_contacts, float duration) {
  vec3 velocity_change[2], rotation_change[2];
  vec3 delta_vel;

  contact_resolver->velocity_iterations_used = 0;
  while (contact_resolver->velocity_iterations_used < contact_resolver->velocity_iterations) {
    float max = contact_resolver->velocity_epsilon;
    unsigned index = num_contacts;
    for (unsigned i = 0; i < num_contacts; i++) {
      if (contact[i].desired_delta_velocity > max) {
        max = contact[i].desired_delta_velocity;
        index = i;
      }
    }
    if (index == num_contacts)
      break;

    contact_match_awake_state(&contact[index]);
    contact_apply_velocity_change(&contact[index], velocity_change, rotation_change);

    for (unsigned int i = 0; i < num_contacts; i++) {
      for (unsigned int b = 0; b < 2; b++)
        if (contact[i].body[b]) {
          for (unsigned int d = 0; d < 2; d++) {
            if (contact[i].body[b] == contact[index].body[d]) {
              delta_vel = vec3_add(velocity_change[d], vec3_cross_product(rotation_change[d], contact[i].relative_contact_position[b]));
              contact[i].contact_velocity = vec3_add(contact[i].contact_velocity, vec3_scale(mat3_transform_transpose(contact[i].contact_to_world, delta_vel), (b ? -1 : 1)));
              contact_calculate_desired_delta_velocity(&contact[i], duration);
            }
          }
        }
    }
    contact_resolver->velocity_iterations_used++;
  }
}

void contact_resolver_adjust_positions(struct ContactResolver* contact_resolver, struct Contact* contact, unsigned int num_contacts, float duration) {
  unsigned int i, index;
  vec3 linear_change[2], angular_change[2];
  float max;
  vec3 delta_position;

  contact_resolver->position_iterations_used = 0;
  while (contact_resolver->position_iterations_used < contact_resolver->position_iterations) {
    max = contact_resolver->position_epsilon;
    index = num_contacts;
    for (i = 0; i < num_contacts; i++) {
      if (contact[i].penetration > max) {
        max = contact[i].penetration;
        index = i;
      }
    }
    if (index == num_contacts)
      break;

    contact_match_awake_state(&contact[index]);
    contact_apply_position_change(&contact[index], linear_change, angular_change, max);

    for (i = 0; i < num_contacts; i++) {
      for (unsigned int b = 0; b < 2; b++)
        if (contact[i].body[b]) {
          for (unsigned int d = 0; d < 2; d++) {
            if (contact[i].body[b] == contact[index].body[d]) {
              delta_position = vec3_add(linear_change[d], vec3_cross_product(angular_change[d], contact[i].relative_contact_position[b]));
              contact[i].penetration += vec3_dot(delta_position, contact[i].contact_normal) * (b ? 1 : -1);
            }
          }
        }
    }
    contact_resolver->position_iterations_used++;
  }
}
