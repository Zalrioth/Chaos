#include "chaos/core/contacts.h"

void contact_set_body_data(struct Contact* contact, struct RigidBody* one, struct RigidBody* two, real friction, real restitution) {
  contact->body[0] = one;
  contact->body[1] = two;
  contact->friction = friction;
  contact->restitution = restitution;
}

void contact_match_awake_state(struct Contact* contact) {
  if (!contact->body[1])
    return;

  bool body0_awake = rigid_body_get_is_awake(contact->body[0]);
  bool body1_awake = rigid_body_get_is_awake(contact->body[1]);

  if (body0_awake ^ body1_awake) {
    if (body0_awake)
      rigid_body_set_awake(contact->body[1], true);
    else
      rigid_body_set_awake(contact->body[0], true);
  }
}

void contact_swap_bodies(struct Contact* contact) {
  vec3_copy(contact->contact_normal, vec3_invert(contact->contact_normal));

  struct RigidBody* temp = contact->body[0];
  contact->body[0] = contact->body[1];
  contact->body[1] = temp;
}

void contact_calculate_contact_basis(struct Contact* contact) {
  vec3 contact_tangent[2];

  real* contact_normal = contact->contact_normal;

  if (real_abs(contact_normal[0]) > real_abs(contact_normal[1])) {
    const real s = (real)1.0f / real_sqrt(contact_normal[2] * contact_normal[2] + contact_normal[0] * contact_normal[0]);

    contact_tangent[0][0] = contact_normal[2] * s;
    contact_tangent[0][1] = 0;
    contact_tangent[0][2] = -contact_normal[0] * s;

    contact_tangent[1][0] = contact_normal[1] * contact_tangent[0][0];
    contact_tangent[1][1] = contact_normal[2] * contact_tangent[0][0] - contact_normal[0] * contact_tangent[0][2];
    contact_tangent[1][2] = -contact_normal[1] * contact_tangent[0][0];
  } else {
    const real s = (real)1.0 / real_sqrt(contact_normal[2] * contact_normal[2] + contact_normal[1] * contact_normal[1]);

    contact_tangent[0][0] = 0;
    contact_tangent[0][1] = -contact_normal[2] * s;
    contact_tangent[0][2] = contact_normal[1] * s;

    contact_tangent[1][0] = contact_normal[1] * contact_tangent[0][2] - contact_normal[2] * contact_tangent[0][2];
    contact_tangent[1][1] = -contact_normal[0] * contact_tangent[0][2];
    contact_tangent[1][2] = contact_normal[0] * contact_tangent[0][1];
  }
  mat3_copy(contact->contact_to_world, mat3_set_components(contact_normal, contact_tangent[0], contact_tangent[1]));
}

real* contact_calculate_local_velocity(struct Contact* contact, unsigned int body_index, real duration) {
  struct RigidBody* this_body = contact->body[body_index];

  vec3 velocity;
  vec3_copy(velocity, vec3_cross_product(rigid_body_get_rotation(this_body), contact->relative_contact_position[body_index]));
  vec3_copy(velocity, vec3_add(velocity, rigid_body_get_velocity(this_body)));

  vec3 contact_velocity;
  vec3_copy(contact_velocity, mat3_transform_transpose(contact->contact_to_world, velocity));

  vec3 acc_velocity;
  vec3_copy(acc_velocity, vec3_mul_scalar(rigid_body_get_last_frame_acceleration(this_body), duration));
  vec3_copy(acc_velocity, mat3_transform_transpose(contact->contact_to_world, acc_velocity));
  acc_velocity[0] = 0;

  vec3_copy(contact_velocity, vec3_add(contact_velocity, acc_velocity));
  return (vec3){contact_velocity[0], contact_velocity[1], contact_velocity[2]};
}

void contact_calculate_desired_delta_velocity(struct Contact* contact, real duration) {
  real velocity_from_acc = 0;

  if (rigid_body_get_is_awake(contact->body[0]))
    velocity_from_acc += vec3_magnitude(vec3_component_product(vec3_mul_scalar(rigid_body_get_last_frame_acceleration(contact->body[0]), duration), contact->contact_normal));

  if (contact->body[1] && rigid_body_get_is_awake(contact->body[1]))
    velocity_from_acc -= vec3_magnitude(vec3_component_product(vec3_mul_scalar(rigid_body_get_last_frame_acceleration(contact->body[1]), duration), contact->contact_normal));

  real this_restitution = contact->restitution;
  if (real_abs(contact->contact_velocity[0]) < velocity_limit)
    this_restitution = (real)0.0f;

  contact->desired_delta_velocity = -contact->contact_velocity[0] - this_restitution * (contact->contact_velocity[0] - velocity_from_acc);
}

void contact_calculate_internals(struct Contact* contact, real duration) {
  if (!contact->body[0])
    contact_swap_bodies(contact);

  contact_calculate_contact_basis(contact);
  vec3_copy(contact->relative_contact_position[0], vec3_sub(contact->contact_point, rigid_body_get_position(contact->body[0])));

  if (contact->body[1])
    vec3_copy(contact->relative_contact_position[1], vec3_sub(contact->contact_point, rigid_body_get_position(contact->body[1])));

  vec3_copy(contact->contact_velocity, contact_calculate_local_velocity(contact, 0, duration));
  if (contact->body[1])
    vec3_copy(contact->contact_velocity, vec3_sub(contact->contact_velocity, contact_calculate_local_velocity(contact, 1, duration)));

  contact_calculate_desired_delta_velocity(contact, duration);
}

// NOTE: Watch out for this typedef pointer
void contact_apply_velocity_change(struct Contact* contact, vec3* velocity_change, vec3* rotation_change) {
  mat3 inverse_inertia_tensor[2];
  mat3_copy(inverse_inertia_tensor[0], rigid_body_get_inverse_inertia_tensor_world(contact->body[0]));
  if (contact->body[1])
    mat3_copy(inverse_inertia_tensor[1], rigid_body_get_inverse_inertia_tensor_world(contact->body[1]));

  vec3 impulse_contact;
  if (contact->friction == (real)0.0)
    vec3_copy(impulse_contact, contact_calculate_frictionless_impulse(contact, inverse_inertia_tensor));
  else
    vec3_copy(impulse_contact, contact_calculate_friction_impulse(contact, inverse_inertia_tensor));

  vec3 impulse;
  vec3_copy(impulse, mat3_transform(contact->contact_to_world, impulse_contact));

  vec3 impulsive_torque;
  vec3_copy(impulsive_torque, vec3_cross_product(contact->relative_contact_position[0], impulse));

  vec3_copy(rotation_change[0], mat3_transform(inverse_inertia_tensor[0], impulsive_torque));
  vec3_clear(velocity_change[0]);

  vec3_copy(velocity_change[0], vec3_add_scaled_vector(velocity_change[0], impulse, rigid_body_get_inverse_mass(contact->body[0])));

  rigid_body_add_velocity(contact->body[0], velocity_change[0]);
  rigid_body_add_rotation(contact->body[0], rotation_change[0]);

  if (contact->body[1]) {
    vec3 impulsive_torque;
    vec3_copy(impulsive_torque, vec3_cross_product(impulse, contact->relative_contact_position[1]));

    vec3_copy(rotation_change[1], mat3_transform(inverse_inertia_tensor[1], impulsive_torque));
    vec3_clear(velocity_change[1]);
    vec3_copy(velocity_change[1], vec3_add_scaled_vector(velocity_change[1], impulse, -rigid_body_get_inverse_mass(contact->body[1])));

    rigid_body_add_velocity(contact->body[1], velocity_change[1]);
    rigid_body_add_rotation(contact->body[1], rotation_change[1]);
  }
}

real* contact_calculate_frictionless_impulse(struct Contact* contact, mat3* inverse_inertia_tensor) {
  vec3 delta_vel_world;
  vec3_copy(delta_vel_world, vec3_cross_product(contact->relative_contact_position[0], contact->contact_normal));
  vec3_copy(delta_vel_world, mat3_transform(inverse_inertia_tensor[0], delta_vel_world));
  vec3_copy(delta_vel_world, vec3_cross_product(delta_vel_world, contact->relative_contact_position[0]));
  real delta_velocity = vec3_magnitude(vec3_component_product(delta_vel_world, contact->contact_normal));
  delta_velocity += rigid_body_get_inverse_mass(contact->body[0]);

  if (contact->body[1]) {
    vec3 delta_vel_world_other;
    vec3_copy(delta_vel_world_other, vec3_cross_product(contact->relative_contact_position[1], contact->contact_normal));
    vec3_copy(delta_vel_world_other, mat3_transform(inverse_inertia_tensor[1], delta_vel_world_other));
    vec3_copy(delta_vel_world_other, vec3_cross_product(delta_vel_world_other, contact->relative_contact_position[1]));
    delta_velocity += vec3_magnitude(vec3_component_product(delta_vel_world_other, contact->contact_normal));
    delta_velocity += rigid_body_get_inverse_mass(contact->body[1]);
  }

  return (vec3){contact->desired_delta_velocity / delta_velocity, 0, 0};
}

real* contact_calculate_friction_impulse(struct Contact* contact, mat3* inverse_inertia_tensor) {
  vec3 impulse_contact;
  real inverse_mass = rigid_body_get_inverse_mass(contact->body[0]);

  mat3 impulse_to_torque;
  mat3_set_skew_symmetric(impulse_to_torque, contact->relative_contact_position[0]);

  mat3 delta_vel_world;
  mat3_copy(delta_vel_world, impulse_to_torque);
  mat3_copy(delta_vel_world, mat3_mul_mat3(delta_vel_world, inverse_inertia_tensor[0]));
  mat3_copy(delta_vel_world, mat3_mul_mat3(delta_vel_world, impulse_to_torque));
  mat3_copy(delta_vel_world, mat3_mul_scalar(delta_vel_world, -1));

  if (contact->body[1]) {
    mat3_set_skew_symmetric(impulse_to_torque, contact->relative_contact_position[1]);

    mat3 delta_vel_world_other;
    mat3_copy(delta_vel_world_other, impulse_to_torque);
    mat3_copy(delta_vel_world_other, mat3_mul_mat3(delta_vel_world, inverse_inertia_tensor[1]));
    mat3_copy(delta_vel_world_other, mat3_mul_mat3(delta_vel_world, impulse_to_torque));
    mat3_copy(delta_vel_world_other, mat3_mul_scalar(delta_vel_world, -1));

    mat3_copy(delta_vel_world, mat3_add_mat3(delta_vel_world, delta_vel_world_other));

    inverse_mass += rigid_body_get_inverse_mass(contact->body[1]);
  }

  mat3 delta_velocity;
  mat3_copy(delta_velocity, mat3_transpose(contact->contact_to_world));
  mat3_copy(delta_velocity, mat3_mul_mat3(delta_velocity, delta_vel_world));
  mat3_copy(delta_velocity, mat3_mul_mat3(delta_velocity, contact->contact_to_world));

  delta_velocity[0] += inverse_mass;
  delta_velocity[4] += inverse_mass;
  delta_velocity[8] += inverse_mass;

  mat3 impulse_matrix;
  mat3_copy(impulse_matrix, mat3_inverse(delta_velocity));

  vec3 vel_kill = {contact->desired_delta_velocity, -contact->contact_velocity[1], -contact->contact_velocity[2]};

  vec3_copy(impulse_contact, mat3_transform(impulse_matrix, vel_kill));

  real planarImpulse = real_sqrt(impulse_contact[1] * impulse_contact[1] + impulse_contact[2] * impulse_contact[2]);
  if (planarImpulse > impulse_contact[0] * contact->friction) {
    impulse_contact[1] /= planarImpulse;
    impulse_contact[2] /= planarImpulse;

    impulse_contact[0] = delta_velocity[0] + delta_velocity[1] * contact->friction * impulse_contact[1] + delta_velocity[2] * contact->friction * impulse_contact[2];
    impulse_contact[0] = contact->desired_delta_velocity / impulse_contact[0];
    impulse_contact[1] *= contact->friction * impulse_contact[0];
    impulse_contact[2] *= contact->friction * impulse_contact[0];
  }
  return (vec3){impulse_contact[0], impulse_contact[1], impulse_contact[0]};
}

void contact_apply_position_change(struct Contact* contact, vec3* linear_change, vec3* angular_change, real penetration) {
  real angular_limit = (real)0.2f;
  real angular_move[2];
  real linear_move[2];

  real total_inertia = 0;
  real linear_inertia[2];
  real angular_inertia[2];

  for (unsigned int i = 0; i < 2; i++)
    if (contact->body[i]) {
      mat3 inverse_inertia_tensor;
      mat3_copy(inverse_inertia_tensor, rigid_body_get_inverse_inertia_tensor_world(contact->body[i]));

      vec3 angular_inertia_world;
      vec3_copy(angular_inertia_world, vec3_cross_product(contact->relative_contact_position[i], contact->contact_normal));
      vec3_copy(angular_inertia_world, mat3_transform(inverse_inertia_tensor, angular_inertia_world));
      vec3_copy(angular_inertia_world, vec3_cross_product(angular_inertia_world, contact->relative_contact_position[i]));

      angular_inertia[i] = vec3_magnitude(vec3_component_product(angular_inertia_world, contact->contact_normal));
      linear_inertia[i] = rigid_body_get_inverse_mass(contact->body[i]);
      total_inertia += linear_inertia[i] + angular_inertia[i];
    }

  for (unsigned int i = 0; i < 2; i++)
    if (contact->body[i]) {
      real sign = (i == 0) ? 1 : -1;
      angular_move[i] = sign * penetration * (angular_inertia[i] / total_inertia);
      linear_move[i] = sign * penetration * (linear_inertia[i] / total_inertia);

      vec3 projection;
      vec3_copy(projection, contact->relative_contact_position[i]);
      vec3_copy(projection, vec3_add_scaled_vector(projection, contact->contact_normal, -vec3_scalar_product(contact->relative_contact_position[i], contact->contact_normal)));

      real max_magnitude = angular_limit * vec3_magnitude(projection);

      if (angular_move[i] < -max_magnitude) {
        real totalMove = angular_move[i] + linear_move[i];
        angular_move[i] = -max_magnitude;
        linear_move[i] = totalMove - angular_move[i];
      } else if (angular_move[i] > max_magnitude) {
        real totalMove = angular_move[i] + linear_move[i];
        angular_move[i] = max_magnitude;
        linear_move[i] = totalMove - angular_move[i];
      }

      if (angular_move[i] == 0)
        vec3_clear(angular_change[i]);
      else {
        vec3 target_angular_direction;
        vec3_copy(target_angular_direction, vec3_cross_product(contact->relative_contact_position[i], contact->contact_normal));

        mat3 inverse_inertia_tensor;
        mat3_copy(inverse_inertia_tensor, rigid_body_get_inertia_tensor_world(contact->body[i]));

        vec3_copy(angular_change[i], vec3_mul_scalar(mat3_transform(inverse_inertia_tensor, target_angular_direction), (angular_move[i] / angular_inertia[i])));
      }

      vec3_copy(linear_change[i], vec3_mul_scalar(contact->contact_normal, linear_move[i]));

      vec3 pos;
      vec3_copy(pos, rigid_body_get_position(contact->body[i]));
      vec3_copy(pos, vec3_add_scaled_vector(pos, contact->contact_normal, linear_move[i]));
      rigid_body_set_position(contact->body[i], pos);

      quaternion q;
      quaternion_copy(q, rigid_body_get_orientation(contact->body[i]));
      quaternion_add_scaled_vector(q, angular_change[i], ((real)1.0));
      rigid_body_set_orientation(contact->body[i], q);

      if (!rigid_body_get_is_awake(contact->body[i]))
        rigid_body_calculate_derived_data(contact->body[i]);
    }
}

void contact_resolver_init(struct ContactResolver* contact_resolver, unsigned int velocity_iterations, unsigned int position_iterations, real velocity_epsilon, real position_epsilon) {
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

void contact_resolver_set_epsilon(struct ContactResolver* contact_resolver, real velocity_epsilon, real position_epsilon) {
  contact_resolver->velocity_epsilon = velocity_epsilon;
  contact_resolver->position_epsilon = position_epsilon;
}

void contact_resolver_resolve_contacts(struct ContactResolver* contact_resolver, struct Contact* contacts, unsigned int num_contacts, real duration) {
  if (num_contacts == 0)
    return;
  if (!contact_resolver_is_valid(contact_resolver))
    return;

  contact_resolver_prepare_contacts(contact_resolver, contacts, num_contacts, duration);
  contact_resolver_adjust_positions(contact_resolver, contacts, num_contacts, duration);
  contact_resolver_adjust_velocities(contact_resolver, contacts, num_contacts, duration);
}

void contact_resolver_prepare_contacts(struct ContactResolver* contact_resolver, struct Contact* contacts, unsigned int num_contacts, real duration) {
  struct Contact* last_contact = contacts + num_contacts;
  for (struct Contact* contact = contacts; contact < last_contact; contact++) {
    contact_calculate_internals(contact, duration);
  }
}

void contact_resolver_adjust_velocities(struct ContactResolver* contact_resolver, struct Contact* contact, unsigned num_contacts, real duration) {
  vec3 velocity_change[2], rotation_change[2];
  vec3 delta_vel;

  contact_resolver->velocity_iterations_used = 0;
  while (contact_resolver->velocity_iterations_used < contact_resolver->velocity_iterations) {
    real max = contact_resolver->velocity_epsilon;
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
              vec3_copy(delta_vel, vec3_add(velocity_change[d], vec3_cross_product(rotation_change[d], contact[i].relative_contact_position[b])));
              // NOTE: Double check
              vec3_copy(contact[i].contact_velocity, vec3_add(contact[i].contact_velocity, vec3_mul_scalar(mat3_transform_transpose(contact[i].contact_to_world, delta_vel), (b ? -1 : 1))));
              contact_calculate_desired_delta_velocity(&contact[i], duration);
            }
          }
        }
    }
    contact_resolver->velocity_iterations_used++;
  }
}

void contact_resolver_adjust_positions(struct ContactResolver* contact_resolver, struct Contact* contact, unsigned num_contacts, real duration) {
  unsigned int i, index;
  vec3 linear_change[2], angular_change[2];
  real max;
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
              vec3_copy(delta_position, vec3_add(linear_change[d], vec3_cross_product(angular_change[d], contact[i].relative_contact_position[b])));
              contact[i].penetration += vec3_scalar_product(delta_position, contact[i].contact_normal) * (b ? 1 : -1);
            }
          }
        }
    }
    contact_resolver->position_iterations_used++;
  }
}
