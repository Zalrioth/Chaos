#include "chaos/core/fgen.h"

void gravity_init(struct Gravity* gravity, vec3 gravity_direction) {
  gravity->gravity_direction = gravity_direction;
}

void gravity_update_force(struct Gravity* gravity, struct RigidBody* body, float duration) {
  if (!rigid_body_has_finite_mass(body))
    return;

  rigid_body_add_force(body, vec3_scale(gravity->gravity_direction, rigid_body_get_mass(body)));
}

void spring_init(struct Spring* spring, vec3 local_connection_point, struct RigidBody* other, vec3 other_connection_point, float spring_constant, float rest_length) {
  spring->connection_point = local_connection_point;
  spring->other_connection_point = other_connection_point;
  spring->other = other;
  spring->spring_constant = spring_constant;
  spring->rest_length = rest_length;
}

void spring_update_force(struct Spring* spring, struct RigidBody* body, float duration) {
  vec3 lws = rigid_body_get_point_in_world_space(body, spring->connection_point);
  vec3 ows = rigid_body_get_point_in_world_space(spring->other, spring->other_connection_point);
  vec3 force = vec3_sub(lws, ows);

  float magnitude = vec3_magnitude(force);
  magnitude = fabsf(magnitude - spring->rest_length);
  magnitude *= spring->spring_constant;

  force = vec3_normalise(force);
  force = vec3_scale(force, -magnitude);
  rigid_body_add_force_at_point(body, force, lws);
}

//void explosion_init(struct Explosion* explosion) {
//}

void aero_init(struct Aero* aero, mat3 tensor, vec3 position, vec3* wind_speed) {
  aero->tensor = tensor;
  aero->position = position;
  aero->wind_speed = wind_speed;
}

void aero_update_force(struct Aero* aero, struct RigidBody* body, float duration) {
  aero_update_force_from_tensor(aero, body, duration, aero->tensor);
}

void aero_update_force_from_tensor(struct Aero* aero, struct RigidBody* body, float duration, mat3 tensor) {
  vec3 velocity = body->velocity;
  velocity = vec3_add(velocity, *aero->wind_speed);

  vec3 body_vel = mat4_transform_inverse_direction(body->transform_matrix, velocity);
  vec3 body_force = mat3_transform(tensor, body_vel);
  vec3 force = mat4_transform_direction(body->transform_matrix, body_force);

  rigid_body_add_force_at_body_point(body, force, aero->position);
}

void aero_control_init(struct AeroControl* aero_control, mat3 base, mat3 min, mat3 max, vec3 position, vec3* wind_speed) {
  aero_control = calloc(1, sizeof(struct Aero));
  aero_init(&aero_control->aero, base, position, wind_speed);

  aero_control->min_tensor = min;
  aero_control->max_tensor = max;
  aero_control->control_setting = 0.0f;
}

void aero_control_delete(struct AeroControl* aero_control) {
  free(&aero_control->aero);
}

mat3 aero_control_get_tensor(struct AeroControl* aero_control) {
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

void aero_control_set_control(struct AeroControl* aero_control, float value) {
  aero_control->control_setting = value;
}

void aero_control_update_force(struct AeroControl* aero_control, struct RigidBody* body, float duration) {
  mat3 tensor = aero_control_get_tensor(aero_control);
  aero_update_force_from_tensor(&aero_control->aero, body, duration, tensor);
}

// Note: Default liquid density is 1000.0f
void buoyancy_init(struct Buoyancy* buoyancy, vec3 c_of_b, float max_depth, float volume, float water_height, float liquid_density) {
  buoyancy->centre_of_bouyancy = c_of_b;
  buoyancy->liquid_density = liquid_density;
  buoyancy->max_depth = max_depth;
  buoyancy->volume = volume;
  buoyancy->water_height = water_height;
}

void buoyancy_update_force(struct Buoyancy* buoyancy, struct RigidBody* body, float duration) {
  vec3 point_in_world = rigid_body_get_point_in_world_space(body, buoyancy->centre_of_bouyancy);
  float depth = point_in_world.data[1];

  if (depth >= buoyancy->water_height + buoyancy->max_depth)
    return;

  vec3 force = (vec3){.data[0] = 0.0, .data[1] = 0.0, .data[2] = 0.0};

  if (depth <= buoyancy->water_height - buoyancy->max_depth) {
    force.data[1] = buoyancy->liquid_density * buoyancy->volume;
    rigid_body_add_force_at_body_point(body, force, buoyancy->centre_of_bouyancy);
    return;
  }

  force.data[1] = buoyancy->liquid_density * buoyancy->volume * (depth - buoyancy->max_depth - buoyancy->water_height) / 2 * buoyancy->max_depth;
  rigid_body_add_force_at_body_point(body, force, buoyancy->centre_of_bouyancy);
}

void force_registry_update_forces(struct ForceRegistry* force_registry, float duration) {
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
