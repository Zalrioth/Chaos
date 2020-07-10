#pragma once
#ifndef RANDOM_H
#define RANDOM_H

#include <time.h>
#include <ubermath/ubermath.h>

struct Random {
  int p1, p2;
  unsigned buffer[17];
};

void random_seed(struct Random* random, unsigned int seed);
unsigned int random_rotl(unsigned int n, unsigned int r);
unsigned int random_rotr(unsigned int n, unsigned int r);
unsigned int random_bits(struct Random* random);
float random_random_float(struct Random* random);
float random_float_min_max(struct Random* random, float min, float max);
float random_float_scale(struct Random* random, float scale);
unsigned int random_int(struct Random* random, unsigned int max);
float random_binomial_scale(struct Random* random, float scale);
quat random_quaternion(struct Random* random);
vec3 random_vector_scale(struct Random* random, float scale);
vec3 random_vector_xz(struct Random* random, float scale);
vec3 random_vector_scale_xyz(struct Random* random, float* scale);
vec3 random_vector_min_max(struct Random* random, float* min, float* max);

#endif  // RANDOM_H
