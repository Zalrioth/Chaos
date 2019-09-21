#ifndef COLLIDE_COARSE_H
#define COLLIDE_COARSE_H

#include <stddef.h>
#include "core/contacts.h"

struct BoundingSphere {
  vec3 centre;
  real radius;
};

static inline void bounding_sphere_init(struct BoundingSphere* bounding_sphere, real* centre, real radius);
static inline void bounding_sphere_init_two(struct BoundingSphere* bounding_sphere, struct BoundingSphere* one, struct BoundingSphere* two);
static inline bool bounding_sphere_overlaps(struct BoundingSphere* bounding_sphere, struct BoundingSphere* other);
static inline real bounding_sphere_get_growth(struct BoundingSphere* bounding_sphere, struct BoundingSphere* other);
static inline real bounding_sphere_get_size(struct BoundingSphere* bounding_sphere);

struct PotentialContact {
  struct RigidBody* body[2];
};

union BoundingVolumeClass {
  int placeholder;
};

struct BVHNode {
  struct BVHNode* children[2];
  union BoundingVolumeClass* volume;
  struct RigidBody* body;
  struct BVHNode* parent;
};

static inline void bvh_node_init(struct BVHNode* bvh_node, struct BVHNode* parent, union BoundingVolumeClass* volume, struct RigidBody* body) {
  bvh_node->children[0] = bvh_node->children[1] = NULL;
  bvh_node->parent = parent;
  bvh_node->volume = volume;
  bvh_node->body = body;
}

static inline bool bvh_node_is_leaf(struct BVHNode* bvh_node) {
  return (bvh_node->body != NULL);
}

unsigned getPotentialContacts(PotentialContact* contacts, unsigned limit) const;
void insert(RigidBody* body, const BoundingVolumeClass& volume);
~BVHNode();
bool overlaps(const BVHNode<BoundingVolumeClass>* other) const;
unsigned getPotentialContactsWith(const BVHNode<BoundingVolumeClass>* other, PotentialContact* contacts, unsigned limit) const;
void recalculateBoundingVolume(bool recurse = true);

template <class BoundingVolumeClass>
bool BVHNode<BoundingVolumeClass>::overlaps(const BVHNode<BoundingVolumeClass>* other) const {
  return volume->overlaps(other->volume);
}

template <class BoundingVolumeClass>
void BVHNode<BoundingVolumeClass>::insert(RigidBody* newBody, const BoundingVolumeClass& newVolume) {
  if (isLeaf()) {
    children[0] = new BVHNode<BoundingVolumeClass>(this, volume, body);
    children[1] = new BVHNode<BoundingVolumeClass>(this, newVolume, newBody);

    this->body = NULL;

    recalculateBoundingVolume();
  }

  else {
    if (children[0]->volume.getGrowth(newVolume) < children[1]->volume.getGrowth(newVolume)) {
      children[0]->insert(newBody, newVolume);
    } else {
      children[1]->insert(newBody, newVolume);
    }
  }
}

template <class BoundingVolumeClass>
BVHNode<BoundingVolumeClass>::~BVHNode() {
  if (parent) {
    BVHNode<BoundingVolumeClass>* sibling;
    if (parent->children[0] == this)
      sibling = parent->children[1];
    else
      sibling = parent->children[0];

    parent->volume = sibling->volume;
    parent->body = sibling->body;
    parent->children[0] = sibling->children[0];
    parent->children[1] = sibling->children[1];

    sibling->parent = NULL;
    sibling->body = NULL;
    sibling->children[0] = NULL;
    sibling->children[1] = NULL;
    delete sibling;

    parent->recalculateBoundingVolume();
  }

  if (children[0]) {
    children[0]->parent = NULL;
    delete children[0];
  }
  if (children[1]) {
    children[1]->parent = NULL;
    delete children[1];
  }
}

template <class BoundingVolumeClass>
void BVHNode<BoundingVolumeClass>::recalculateBoundingVolume(
    bool recurse) {
  if (isLeaf())
    return;

  volume = BVHNode(
      children[0]->volume,
      children[1]->volume);

  if (parent)
    parent->recalculateBoundingVolume(true);
}

template <class BoundingVolumeClass>
unsigned BVHNode<BoundingVolumeClass>::getPotentialContacts(PotentialContact* contacts, unsigned limit) const {
  if (isLeaf() || limit == 0)
    return 0;

  return children[0]->getPotentialContactsWith(
      children[1], contacts, limit);
}

template <class BoundingVolumeClass>
unsigned BVHNode<BoundingVolumeClass>::getPotentialContactsWith(const BVHNode<BoundingVolumeClass>* other, PotentialContact* contacts, unsigned limit) const {
  if (!overlaps(other) || limit == 0)
    return 0;

  if (isLeaf() && other->isLeaf()) {
    contacts->body[0] = body;
    contacts->body[1] = other->body;
    return 1;
  }

  if (other->isLeaf() || (!isLeaf() && volume->getSize() >= other->volume->getSize())) {
    unsigned count = children[0]->getPotentialContactsWith(other, contacts, limit);

    if (limit > count)
      return count + children[1]->getPotentialContactsWith(other, contacts + count, limit - count);
    else
      return count;
  } else {
    unsigned count = getPotentialContactsWith(other->children[0], contacts, limit);

    if (limit > count)
      return count + getPotentialContactsWith(other->children[1], contacts + count, limit - count);
    else
      return count;
  }
}

#endif  // COLLIDE_COARSE_H
