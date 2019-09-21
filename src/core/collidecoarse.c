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
