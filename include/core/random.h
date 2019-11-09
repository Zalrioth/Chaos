#pragma once
#ifndef RANDOM_H
#define RANDOM_H

#include <time.h>
#include "core/core.h"

struct Random {
  int p1, p2;
  unsigned buffer[17];
};

unsigned int random_int(struct Random* random, unsigned int max);
real random_binomial_scale(struct Random* random, real scale);
real* random_quaternion(struct Random* random);
real* random_vector_scale(struct Random* random, real scale);
real* random_vector_xz(struct Random* random, real scale);
real* random_vector_scale_xyz(struct Random* random, real* scale);
real* random_vector_min_max(struct Random* random, real* min, real* max);
void random_seed(struct Random* random, unsigned int seed);
unsigned int random_rotl(unsigned int n, unsigned int r);
unsigned int random_rotr(unsigned int n, unsigned int r);
unsigned int random_bits(struct Random* random);
real random_random_real(struct Random* random);
real random_real_min_max(struct Random* random, real min, real max);
real random_real_scale(struct Random* random, real scale);

#endif  // RANDOM_H
