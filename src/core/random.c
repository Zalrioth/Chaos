#include "core/random.h"

void random_seed(struct Random* random, unsigned int seed) {
  if (seed == 0)
    seed = (unsigned)clock();

  for (unsigned i = 0; i < 17; i++) {
    seed = seed * 2891336453 + 1;
    random->buffer[i] = seed;
  }

  random->p1 = 0;
  random->p2 = 10;
}

unsigned int random_rotl(unsigned int n, unsigned int r) {
  return (n << r) | (n >> (32 - r));
}

unsigned int random_rotr(unsigned int n, unsigned int r) {
  return (n >> r) | (n << (32 - r));
}

unsigned int random_bits(struct Random* random) {
  unsigned result;

  result = random->buffer[random->p1] = random_rotl(random->buffer[random->p2], 13) + random_rotl(random->buffer[random->p1], 9);

  if (--random->p1 < 0)
    random->p1 = 16;
  if (--random->p2 < 0)
    random->p2 = 16;

  return result;
}

#ifdef SINGLE_PRECISION
real random_random_real(struct Random* random) {
  unsigned bits = random_bits(random);

  union {
    real value;
    unsigned int word;
  } convert;

  convert.word = (bits >> 9) | 0x3f800000;

  return convert.value - 1.0f;
}
#else
real random_random_real(struct Random* random) {
  unsigned bits = random_bits(random);

  union {
    real value;
    unsigned words[2];
  } convert;

  convert.words[0] = bits << 20;
  convert.words[1] = (bits >> 12) | 0x3FF00000;

  return convert.value - 1.0;
}
#endif

real random_real_min_max(struct Random* random, real min, real max) {
  return random_random_real(random) * (max - min) + min;
}

real random_real_scale(struct Random* random, real scale) {
  return random_random_real(random) * scale;
}

unsigned int random_int(struct Random* random, unsigned int max) {
  return random_bits(random) % max;
}

real random_binomial_scale(struct Random* random, real scale) {
  return (random_random_real(random) - random_random_real(random)) * scale;
}

real* random_quaternion(struct Random* random) {
  quaternion q = {random_random_real(random), random_random_real(random), random_random_real(random), random_random_real(random)};
  quaternion_copy(q, quaternion_normalise(q));
  return (quaternion){q[0], q[1], q[2], q[3]};
}

real* random_vector_scale(struct Random* random, real scale) {
  return (vec3){random_binomial_scale(random, scale), random_binomial_scale(random, scale), random_binomial_scale(random, scale)};
}

real* random_vector_xz(struct Random* random, real scale) {
  return (vec3){random_binomial_scale(random, scale), 0, random_binomial_scale(random, scale)};
}

real* random_vector_scale_xyz(struct Random* random, real* scale) {
  return (vec3){random_binomial_scale(random, scale[0]), random_binomial_scale(random, scale[1]), random_binomial_scale(random, scale[2])};
}

real* random_vector_min_max(struct Random* random, real* min, real* max) {
  return (vec3){random_real_min_max(random, min[0], max[0]), random_real_min_max(random, min[1], max[1]), random_real_min_max(random, min[2], max[2])};
}
