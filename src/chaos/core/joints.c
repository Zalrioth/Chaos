#include "chaos/core/joints.h"

unsigned int joint_add_contact(struct Joint* joint, struct Contact* contact, unsigned int limit) {
  vec3 a_pos_world = rigid_body_get_point_in_world_space(joint->body[0], joint->position[0]);
  vec3 b_pos_world = rigid_body_get_point_in_world_space(joint->body[1], joint->position[1]);

  vec3 a_to_b = vec3_sub(b_pos_world, a_pos_world);

  vec3 normal = vec3_normalise(a_to_b);

  float length = vec3_magnitude(a_to_b);
  if (fabsf(length) > joint->error) {
    contact->body[0] = joint->body[0];
    contact->body[1] = joint->body[1];
    contact->contact_normal = normal;
    contact->contact_point = vec3_scale(vec3_add(a_pos_world, b_pos_world), 0.5f);
    contact->penetration = length - joint->error;
    contact->friction = 1.0f;
    contact->restitution = 0;

    return 1;
  }

  return 0;
}

void joint_set(struct Joint* joint, struct RigidBody* a, vec3 a_pos, struct RigidBody* b, vec3 b_pos, float error) {
  joint->body[0] = a;
  joint->body[1] = b;

  joint->position[0] = a_pos;
  joint->position[1] = b_pos;

  joint->error = error;
}
