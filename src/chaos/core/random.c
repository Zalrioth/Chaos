#include "chaos/core/random.h"

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
float random_random_float(struct Random* random) {
  unsigned bits = random_bits(random);

  union {
    float value;
    unsigned int word;
  } convert;

  convert.word = (bits >> 9) | 0x3f800000;

  return convert.value - 1.0f;
}
#else
float random_random_float(struct Random* random) {
  unsigned bits = random_bits(random);

  union {
    float value;
    unsigned words[2];
  } convert;

  convert.words[0] = bits << 20;
  convert.words[1] = (bits >> 12) | 0x3FF00000;

  return convert.value - 1.0;
}
#endif

float random_float_min_max(struct Random* random, float min, float max) {
  return random_random_float(random) * (max - min) + min;
}

float random_float_scale(struct Random* random, float scale) {
  return random_random_float(random) * scale;
}

unsigned int random_int(struct Random* random, unsigned int max) {
  return random_bits(random) % max;
}

float random_binomial_scale(struct Random* random, float scale) {
  return (random_random_float(random) - random_random_float(random)) * scale;
}

quat random_quaternion(struct Random* random) {
  quat q = (quat){.data[0] = random_random_float(random), .data[1] = random_random_float(random), .data[2] = random_random_float(random), .data[3] = random_random_float(random)};
  q = quaternion_normalise(q);
  return (quat){.data[0] = q.data[0], .data[1] = q.data[1], .data[2] = q.data[2], .data[3] = q.data[3]};
}

vec3 random_vector_scale(struct Random* random, float scale) {
  return (vec3){.data[0] = random_binomial_scale(random, scale), .data[1] = random_binomial_scale(random, scale), .data[2] = random_binomial_scale(random, scale)};
}

vec3 random_vector_xz(struct Random* random, float scale) {
  return (vec3){.data[0] = random_binomial_scale(random, scale), .data[1] = 0, .data[2] = random_binomial_scale(random, scale)};
}

vec3 random_vector_scale_xyz(struct Random* random, float* scale) {
  return (vec3){.data[0] = random_binomial_scale(random, scale[0]), .data[1] = random_binomial_scale(random, scale[1]), .data[2] = random_binomial_scale(random, scale[2])};
}

vec3 random_vector_min_max(struct Random* random, float* min, float* max) {
  return (vec3){.data[0] = random_float_min_max(random, min[0], max[0]), .data[1] = random_float_min_max(random, min[1], max[1]), .data[2] = random_float_min_max(random, min[2], max[2])};
}
