#pragma once
#ifndef RANDOM_H_
#define RANDOM_H_

#include <cstdlib>
#include <ctime>
#include "core/core.hpp"

namespace chaos {
class Random {
 public:
  unsigned rotl(unsigned n, unsigned r);
  unsigned rotr(unsigned n, unsigned r);
  Random();
  Random(unsigned seed);
  void seed(unsigned seed);
  unsigned randomBits();
  float randomReal();
  float randomReal(real scale);
  float randomReal(real min, real max);
  unsigned randomInt(unsigned max);
  float randomBinomial(real scale);
  Vector3 randomVector(real scale);
  Vector3 randomVector(const Vector3& scale);
  Vector3 randomVector(const Vector3& min, const Vector3& max);
  Vector3 randomXZVector(real scale);
  Quaternion randomQuaternion();

 private:
  int p1, p2;
  unsigned buffer[17];
};
}  // namespace chaos

#endif  // RANDOM_H_