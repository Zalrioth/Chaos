#include <core/collidecoarse.h>

static inline void bounding_sphere_init(struct BoundingSphere* bounding_sphere, real* centre, real radius) {
  vec3_copy(bounding_sphere->centre, centre);
  radius = radius;
}

static inline void bounding_sphere_init_two(struct BoundingSphere* bounding_sphere, struct BoundingSphere* one, struct BoundingSphere* two) {
  vec3 centre_offset;
  vec3_copy(centre_offset, vec3_sub(two->centre, one->centre));
  real distance = vec3_square_magnitude(centre_offset);
  real radius_diff = two->radius - one->radius;

  if (radius_diff * radius_diff >= distance) {
    if (one->radius > two->radius) {
      vec3_copy(bounding_sphere->centre, one->centre);
      bounding_sphere->radius = one->radius;
    } else {
      vec3_copy(bounding_sphere->centre, two->centre);
      bounding_sphere->radius = two->radius;
    }
  }

  else {
    distance = real_sqrt(distance);
    bounding_sphere->radius = (distance + one->radius + two->radius) * ((real)0.5);

    vec3_copy(bounding_sphere->centre, one->centre);
    if (distance > 0)
      vec3_copy(bounding_sphere->centre, vec3_add(bounding_sphere->centre, vec3_mul_scalar(centre_offset, ((bounding_sphere->radius - one->radius) / distance))));
  }
}

static inline bool bounding_sphere_overlaps(struct BoundingSphere* bounding_sphere, struct BoundingSphere* other) {
  real distance_squared = vec3_square_magnitude(vec3_sub(bounding_sphere->centre, other->centre));
  return distance_squared < (bounding_sphere->radius + other->radius) * (bounding_sphere->radius + other->radius);
}

static inline real bounding_sphere_get_growth(struct BoundingSphere* bounding_sphere, struct BoundingSphere* other) {
  struct BoundingSphere new_sphere;
  bounding_sphere_init_two(&new_sphere, bounding_sphere, other);

  return new_sphere.radius * new_sphere.radius - bounding_sphere->radius * bounding_sphere->radius;
}

static inline real bounding_sphere_get_size(struct BoundingSphere* bounding_sphere) {
  return ((real)1.333333) * R_PI * bounding_sphere->radius * bounding_sphere->radius * bounding_sphere->radius;
}

static inline void bvh_node_init(struct BVHNode* bvh_node, struct BVHNode* parent, struct BoundingSphere* volume, struct RigidBody* body) {
  bvh_node->children[0] = bvh_node->children[1] = NULL;
  bvh_node->parent = parent;
  bvh_node->volume = volume;
  bvh_node->body = body;
}

// Note: Check deletion on this function
static inline void bvh_node_delete(struct BVHNode* bvh_node) {
  if (bvh_node->parent) {
    struct BVHNode* sibling;
    if (bvh_node->parent->children[0] == bvh_node)
      sibling = bvh_node->parent->children[1];
    else
      sibling = bvh_node->parent->children[0];

    bvh_node->parent->volume = sibling->volume;
    bvh_node->parent->body = sibling->body;
    bvh_node->parent->children[0] = sibling->children[0];
    bvh_node->parent->children[1] = sibling->children[1];

    sibling->parent = NULL;
    sibling->body = NULL;
    sibling->children[0] = NULL;
    sibling->children[1] = NULL;
    bvh_node_delete(sibling);

    bvh_node_recalculate_bounding_volume(bvh_node->parent, true);
  }

  if (bvh_node->children[0]) {
    bvh_node->children[0]->parent = NULL;
    bvh_node_delete(bvh_node->children[0]);
  }
  if (bvh_node->children[1]) {
    bvh_node->children[1]->parent = NULL;
    bvh_node_delete(bvh_node->children[1]);
  }

  free(bvh_node);
}

static inline bool bvh_node_is_leaf(struct BVHNode* bvh_node) {
  return (bvh_node->body != NULL);
}

static inline bool bvh_node_overlaps(struct BVHNode* bvh_node, struct BVHNode* other) {
  bounding_sphere_overlaps(bvh_node->volume, other->volume);
}

static inline void bvh_node_insert(struct BVHNode* bvh_node, struct RigidBody* new_body, struct BoundingSphere* new_volume) {
  if (bvh_node_is_leaf(bvh_node)) {
    bvh_node->children[0] = calloc(1, sizeof(struct BVHNode));
    bvh_node_init(bvh_node->children[0], bvh_node, new_volume, new_body);

    bvh_node->children[1] = calloc(1, sizeof(struct BVHNode));
    bvh_node_init(bvh_node->children[1], bvh_node, new_volume, new_body);

    bvh_node->body = NULL;

    bvh_node_recalculate_bounding_volume(bvh_node->parent, true);
  }

  else {
    if (bounding_sphere_get_growth(bvh_node->children[0]->volume, new_volume) < bounding_sphere_get_growth(bvh_node->children[1]->volume, new_volume))
      bvh_node_insert(bvh_node->children[0], new_body, new_volume);
    else
      bvh_node_insert(bvh_node->children[1], new_body, new_volume);
  }
}

static inline void bvh_node_recalculate_bounding_volume(struct BVHNode* bvh_node, bool recurse) {
  if (bvh_node_is_leaf(bvh_node))
    return;

  bounding_sphere_init_two(bvh_node->volume, bvh_node->children[0]->volume, bvh_node->children[1]->volume);

  if (bvh_node->parent)
    bvh_node_recalculate_bounding_volume(bvh_node->parent, true);
}

static inline unsigned int bvh_node_get_potential_contacts(struct BVHNode* bvh_node, struct PotentialContact* contacts, unsigned int limit) {
  if (bvh_node_is_leaf(bvh_node) || limit == 0)
    return 0;

  return bvh_node_get_potential_contacts_with(bvh_node->children[0], bvh_node->children[1], contacts, limit);
}

static inline unsigned int bvh_node_get_potential_contacts_with(struct BVHNode* bvh_node, struct BVHNode* other, struct PotentialContact* contacts, unsigned int limit) {
  if (!bvh_node_overlaps(bvh_node, other) || limit == 0)
    return 0;

  if (bvh_node_is_leaf(bvh_node) && bvh_node_is_leaf(other)) {
    contacts->body[0] = bvh_node->body;
    contacts->body[1] = other->body;
    return 1;
  }

  if (bvh_node_is_leaf(other) || (!bvh_node_is_leaf(bvh_node) && bounding_sphere_get_size(bvh_node->volume) >= othbounding_sphere_get_size(other->volume))) {
    unsigned int count = bvh_node_get_potential_contacts_with(bvh_node->children[0], other, contacts, limit);

    if (limit > count)
      return count + bvh_node_get_potential_contacts_with(bvh_node->children[1], other, contacts + count, limit - count);
    else
      return count;
  } else {
    unsigned int count = bvh_node_get_potential_contacts_with(bvh_node, other->children[0], contacts, limit);

    if (limit > count)
      return count + bvh_node_get_potential_contacts_with(bvh_node, other->children[1], contacts + count, limit - count);
    else
      return count;
  }
}
