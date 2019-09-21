#include "core/joints.h"

static inline unsigned int joint_add_contact(struct Joint* joint, struct Contact* contact, unsigned limit) {
  vec3 a_pos_world;
  vec3_copy(a_pos_world, rigid_body_get_point_in_world_space(joint->body[0], joint->position[0]));
  vec3 b_pos_world;
  vec3_copy(a_pos_world, rigid_body_get_point_in_world_space(joint->body[1], joint->position[1]));

  vec3 a_to_b;
  vec3_copy(a_to_b, vec3_sub(b_pos_world, a_pos_world));

  vec3 normal;
  vec3_copy(normal, vec3_normalise(a_to_b));

  real length = vec3_magnitude(a_to_b);
  if (real_abs(length) > joint->error) {
    contact->body[0] = joint->body[0];
    contact->body[1] = joint->body[1];
    vec3_copy(contact->contact_normal, normal);
    vec3_copy(contact->contact_point, vec3_mul_scalar(vec3_add(a_pos_world, b_pos_world), 0.5f));
    contact->penetration = length - joint->error;
    contact->friction = 1.0f;
    contact->restitution = 0;

    return 1;
  }

  return 0;
}

static inline void joint_set(struct Joint* joint, struct RigidBody* a, real* a_pos, struct RigidBody* b, real* b_pos, real error) {
  joint->body[0] = a;
  joint->body[1] = b;

  vec3_copy(joint->position[0], a_pos);
  vec3_copy(joint->position[1], b_pos);

  joint->error = error;
}
