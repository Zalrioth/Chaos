
#include "core/world.h"

static inline void world_init(struct World* world, unsigned int max_contacts, unsigned int iterations) {
  world->first_body = NULL;
  contact_resolver_init(&world->resolver, iterations, iterations, (real)0.01, (real)0.01);
  world->first_contact_gen = NULL;
  world->max_contacts = max_contacts;
  world->contacts = calloc(max_contacts, sizeof(struct Contact));
  world->calculate_iterations = (iterations == 0);
}

// NOTE: Might need to iterate free
static inline void world_delete(struct World* world) {
  free(world->contacts);
}

static inline void world_start_frame(struct World* world) {
  struct BodyRegistration* reg = world->first_body;
  while (reg) {
    rigid_body_clear_accumulators(reg->body);
    rigid_body_calculate_derived_data(reg->body);
    reg = reg->next;
  }
}

// NOTE: Watch for functional programming here
static inline unsigned int world_generate_contacts(struct World* world) {
  unsigned int limit = world->max_contacts;
  struct Contact* next_contact = world->contacts;

  struct ContactGenRegistration* reg = world->first_contact_gen;
  while (reg) {
    unsigned int used = reg->gen->add_contact(next_contact, limit);
    limit -= used;
    next_contact += used;

    if (limit <= 0)
      break;

    reg = reg->next;
  }

  return world->max_contacts - limit;
}

static inline void world_run_physics(struct World* world, real duration) {
  struct BodyRegistration* reg = world->first_body;
  while (reg) {
    rigid_body_integrate(reg->body, duration);
    reg = reg->next;
  }

  unsigned used_contacts = world_generate_contacts(world);

  if (world->calculate_iterations)
    contact_resolver_set_iterations(&world->resolver, used_contacts * 4, used_contacts * 4);
  contact_resolver_resolve_contacts(&world->resolver, world->contacts, used_contacts, duration);
}
