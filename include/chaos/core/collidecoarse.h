#ifndef COLLIDE_COARSE_H
#define COLLIDE_COARSE_H

#include <math.h>
#include <stddef.h>

#include "chaos/core/contacts.h"

struct BoundingSphere {
  vec3 centre;
  float radius;
};

static inline void bounding_sphere_init(struct BoundingSphere* bounding_sphere, vec3 centre, float radius);
static inline void bounding_sphere_init_two(struct BoundingSphere* bounding_sphere, struct BoundingSphere* one, struct BoundingSphere* two);
static inline bool bounding_sphere_overlaps(struct BoundingSphere* bounding_sphere, struct BoundingSphere* other);
static inline float bounding_sphere_get_growth(struct BoundingSphere* bounding_sphere, struct BoundingSphere* other);
static inline float bounding_sphere_get_size(struct BoundingSphere* bounding_sphere);

struct PotentialContact {
  struct RigidBody* body[2];
};

struct BVHNode {
  struct BVHNode* children[2];
  struct BoundingSphere* volume;
  struct RigidBody* body;
  struct BVHNode* parent;
};

static inline void bvh_node_init(struct BVHNode* bvh_node, struct BVHNode* parent, struct BoundingSphere* volume, struct RigidBody* body);
static inline void bvh_node_delete(struct BVHNode* bvh_node);
static inline bool bvh_node_is_leaf(struct BVHNode* bvh_node);
static inline bool bvh_node_overlaps(struct BVHNode* bvh_node, struct BVHNode* other);
static inline void bvh_node_insert(struct BVHNode* bvh_node, struct RigidBody* new_body, struct BoundingSphere* new_volume);
static inline void bvh_node_recalculate_bounding_volume(struct BVHNode* bvh_node, bool recurse);
static inline unsigned int bvh_node_get_potential_contacts(struct BVHNode* bvh_node, struct PotentialContact* contacts, unsigned int limit);
static inline unsigned int bvh_node_get_potential_contacts_with(struct BVHNode* bvh_node, struct BVHNode* other, struct PotentialContact* contacts, unsigned int limit);

#endif  // COLLIDE_COARSE_H
