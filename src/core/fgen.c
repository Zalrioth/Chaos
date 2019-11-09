#include "core/fgen.h"

void gravity_init(struct Gravity* gravity, real* gravity_direction) {
  memcpy(gravity->gravity_direction, gravity_direction, sizeof(vec3));
}

void gravity_update_force(struct Gravity* gravity, struct RigidBody* body, real duration) {
  if (!rigid_body_has_finite_mass(body))
    return;

  rigid_body_add_force(body, vec3_mul_scalar(gravity->gravity_direction, rigid_body_get_mass(body)));
}

void spring_init(struct Spring* spring, real* local_connection_point, struct RigidBody* other, vec3 other_connection_point, real spring_constant, real rest_length) {
  memcpy(spring->connection_point, local_connection_point, sizeof(vec3));
  memcpy(spring->other_connection_point, other_connection_point, sizeof(vec3));
  spring->other = other;
  spring->spring_constant = spring_constant;
  spring->rest_length = rest_length;
}

void spring_update_force(struct Spring* spring, struct RigidBody* body, real duration) {
  real* lws = rigid_body_get_point_in_world_space(body, spring->connection_point);
  real* ows = rigid_body_get_point_in_world_space(spring->other, spring->other_connection_point);
  real* force = vec3_sub(lws, ows);

  real magnitude = vec3_magnitude(force);
  magnitude = real_abs(magnitude - spring->rest_length);
  magnitude *= spring->spring_constant;

  vec3_copy(force, vec3_normalise(force));
  vec3_copy(force, vec3_mul_scalar(force, -magnitude));
  rigid_body_add_force_at_point(body, force, lws);
}

//void explosion_init(struct Explosion* explosion) {
//}

void aero_init(struct Aero* aero, real* tensor, real* position, real* wind_speed) {
  memcpy(aero->tensor, tensor, sizeof(mat3));
  memcpy(aero->position, position, sizeof(vec3));
  aero->wind_speed = wind_speed;
}

void aero_update_force(struct Aero* aero, struct RigidBody* body, real duration) {
  aero_update_force_from_tensor(aero, body, duration, aero->tensor);
}

void aero_update_force_from_tensor(struct Aero* aero, struct RigidBody* body, real duration, real* tensor) {
  real* velocity = rigid_body_get_velocity(body);
  vec3_copy(velocity, vec3_add(velocity, aero->wind_speed));

  real* body_vel = mat4_transform_inverse_direction(rigid_body_get_transform(body), velocity);
  real* body_force = mat3_transform(tensor, body_vel);
  real* force = mat4_transform_direction(rigid_body_get_transform(body), body_force);

  rigid_body_add_force_at_body_point(body, force, aero->position);
}

void aero_control_init(struct AeroControl* aero_control, real* base, real* min, real* max, real* position, real* wind_speed) {
  aero_control = calloc(1, sizeof(struct Aero));
  aero_init(&aero_control->aero, base, position, wind_speed);

  mat3_copy(aero_control->min_tensor, min);
  mat3_copy(aero_control->max_tensor, max);
  aero_control->control_setting = 0.0f;
}

void aero_control_delete(struct AeroControl* aero_control) {
  free(&aero_control->aero);
}

real* aero_control_get_tensor(struct AeroControl* aero_control) {
  if (aero_control->control_setting <= -1.0f)
    return aero_control->min_tensor;
  else if (aero_control->control_setting >= 1.0f)
    return aero_control->max_tensor;
  else if (aero_control->control_setting < 0) {
    return mat3_liner_interpolate(aero_control->min_tensor, aero_control->aero.tensor, aero_control->control_setting + 1.0f);
  } else if (aero_control->control_setting > 0) {
    return mat3_liner_interpolate(aero_control->aero.tensor, aero_control->max_tensor, aero_control->control_setting);
  } else
    return aero_control->aero.tensor;
}

void aero_control_set_control(struct AeroControl* aero_control, real value) {
  aero_control->control_setting = value;
}

void aero_control_update_force(struct AeroControl* aero_control, struct RigidBody* body, real duration) {
  real* tensor = aero_control_get_tensor(aero_control);
  aero_update_force_from_tensor(&aero_control->aero, body, duration, tensor);
}

// Note: Default liquid density is 1000.0f
void buoyancy_init(struct Buoyancy* buoyancy, real* c_of_b, real max_depth, real volume, real water_height, real liquid_density) {
  vec3_copy(buoyancy->centre_of_bouyancy, c_of_b);
  buoyancy->liquid_density = liquid_density;
  buoyancy->max_depth = max_depth;
  buoyancy->volume = volume;
  buoyancy->water_height = water_height;
}

void buoyancy_update_force(struct Buoyancy* buoyancy, struct RigidBody* body, real duration) {
  real* point_in_world = rigid_body_get_point_in_world_space(body, buoyancy->centre_of_bouyancy);
  real depth = point_in_world[1];

  if (depth >= buoyancy->water_height + buoyancy->max_depth)
    return;

  vec3 force = {0.0, 0.0, 0.0};

  if (depth <= buoyancy->water_height - buoyancy->max_depth) {
    force[1] = buoyancy->liquid_density * buoyancy->volume;
    rigid_body_add_force_at_body_point(body, force, buoyancy->centre_of_bouyancy);
    return;
  }

  force[1] = buoyancy->liquid_density * buoyancy->volume * (depth - buoyancy->max_depth - buoyancy->water_height) / 2 * buoyancy->max_depth;
  rigid_body_add_force_at_body_point(body, force, buoyancy->centre_of_bouyancy);
}

void force_registry_update_forces(struct ForceRegistry* force_registry, real duration) {
  for (int iterate_num = 0; iterate_num < force_vector_size(&force_registry->registrations); iterate_num++) {
    struct ForceRegistration* force_registration = (struct ForceRegistration*)(force_registry + (iterate_num * sizeof(struct ForceRegistration)));

    force_registration->fg->update_force(&force_registration->fg->force, force_registration->body, duration);

    /*switch (force_registration->fg->force_type) {
      case GRAVITY:
        break;
      case SPRING:
        break;
      case EXPLOSION:
        break;
      case AERO:
        break;
      case AERO_CONTROL:
        break;
      case ANGLED_AERO:
        break;
      case BUOYANCY:
        break;
    }*/
  }
}

void force_registry_add(struct ForceRegistry* force_registry, struct RigidBody* body, struct ForceGenerator* fg) {
  force_vector_push_back(&force_registry->registrations, &(struct ForceRegistration){body, fg});
}

void force_registry_remove(struct ForceRegistry* force_registry, struct RigidBody* body, struct ForceGenerator* fg) {
  for (int iterate_num = 0; iterate_num < force_vector_size(&force_registry->registrations); iterate_num++) {
    struct ForceRegistration* force_registration = (struct ForceRegistration*)(force_registry + (iterate_num * sizeof(struct ForceRegistration)));

    if (force_registration->body == body && force_registration->fg == fg) {
      force_vector_remove(&force_registry->registrations, iterate_num);
      return;
    }
  }
}

void force_registry_clear(struct ForceRegistry* force_registry) {
  force_vector_clear(&force_registry->registrations);
}