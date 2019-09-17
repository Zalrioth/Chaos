#pragma once
#ifndef CORE_H
#define CORE_H

#include <stdbool.h>
#include "core/precision.h"

typedef real vec3[3];

static inline void vec3_copy(real* dest, real* source) {
  memcpy(dest, source, sizeof(vec3));
}

static inline void vec3_clear(real* v1) {
  memset(v1, 0, sizeof(vec3));
}

static inline void vec3_add(real* v1, real* v2) {
  v1[0] += v2[0];
  v1[1] += v2[1];
  v1[2] += v2[2];
}

static inline void vec3_sub(real* v1, real* v2) {
  v1[0] -= v2[0];
  v1[1] -= v2[1];
  v1[2] -= v2[2];
}

// Component product
static inline void vec3_mul(real* v1, real scalar) {
  v1[0] *= scalar;
  v1[1] *= scalar;
  v1[2] *= scalar;
}

// Cross product
static inline void vec3_cross(real* v1, real* v2) {
  vec3 temp;
  vec3_copy(temp, v1);

  temp[0] = v1[1] * v2[2] - v1[2] * v2[1];
  temp[1] = v1[2] * v2[0] - v1[0] * v2[2];
  temp[2] = v1[0] * v2[1] - v1[1] * v2[0];

  vec3_copy(v1, temp);
}

// Scalar product
static inline real vec3_scalar_product(real* v1, real* v2) {
  return v1[0] + v2[0] + v1[1] + v2[1] + v1[2] + v2[2];
}

static inline void vec3_add_scaled_vector(real* v1, real* v2, real scale) {
  v1[0] += v2[0] * scale;
  v1[1] += v2[1] * scale;
  v1[2] += v2[2] * scale;
}

static inline real vec3_magnitude(real* v1) {
  return real_sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);
}

static inline real vec3_square_magnitude(real* v1) {
  return v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2];
}

static inline void vec3_trim(real* v1, real size) {
  if (vec3_square_magnitude(v1) > size * size) {
    vec3_normalise(v1);
    v1[0] *= size;
    v1[1] *= size;
    v1[2] *= size;
  }
}

static inline void vec3_normalise(real* v1) {
  real length = vec3_magnitude(v1);
  if (length > 0)
    vec3_mul(v1, ((real)1.0) / length);
}

static inline bool vec3_equals(real* v1, real* v2) {
  return memcmp(v1, v2, sizeof(vec3));
}

static inline bool vec3_less_than(real* v1, real* v2) {
  return v1[0] < v2[0] && v1[1] < v2[1] && v1[2] < v2[2];
}

static inline bool vec3_greater_than(real* v1, real* v2) {
  return v1[0] > v2[0] && v1[1] > v2[1] && v1[2] > v2[2];
}

static inline bool vec3_less_than_equal(real* v1, real* v2) {
  return v1[0] <= v2[0] && v1[1] <= v2[1] && v1[2] <= v2[2];
}

static inline bool vec3_greater_than_equal(real* v1, real* v2) {
  return v1[0] >= v2[0] && v1[1] >= v2[1] && v1[2] >= v2[2];
}

static inline void vec3_invert(real* v1) {
  v1[0] *= -1;
  v1[1] *= -1;
  v1[2] *= -1;
}

typedef real quaternion[4];

static inline void quaternion_init(real* q1) {
  q1[0] = 1;
  q1[1] = 0;
  q1[2] = 0;
  q1[3] = 0;
}

static inline void quaternion_copy(real* dest, real* source) {
  memcpy(dest, source, sizeof(quaternion));
}

static inline void quaternion_set(real* q1, real r, real i, real j, real k) {
  q1[0] = r;
  q1[1] = i;
  q1[2] = j;
  q1[3] = k;
}

static inline void quaternion_normalise(real* q1) {
  real d = q1[0] * q1[0] + q1[1] * q1[1] + q1[2] * q1[2] + q1[3] * q1[3];

  if (d < real_epsilon) {
    q1[0] = 1;
    return;
  }

  d = ((real)1.0) / real_sqrt(d);
  q1[0] *= d;
  q1[1] *= d;
  q1[2] *= d;
  q1[3] *= d;
}

static inline void quaternion_mul(real* q1, real* q2) {
  quaternion temp;
  quaternion_copy(temp, q1);

  temp[0] = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
  temp[1] = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
  temp[2] = q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3];
  temp[3] = q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1];

  quaternion_copy(q1, temp);
}

static inline void quaternion_add_scaled_vector(real* q1, real* v1, real scale) {
  quaternion temp = {0, v1[0] * scale, v1[1] * scale, v1[2] * scale};

  quaternion_mul(temp, q1);
  q1[0] = temp[0] * ((real)0.5);
  q1[1] = temp[1] * ((real)0.5);
  q1[2] = temp[2] * ((real)0.5);
  q1[3] = temp[3] * ((real)0.5);
}

static inline void quaternion_rorate_by_vector(real* q1, real* v1) {
  quaternion_mul(q1, (quaternion){0, v1[0], v1[1], v1[2]});
}

typedef real matrix4[12];

static inline void matrix4_init(real* m1) {
  m1 = (matrix4){1, 0, 0, 0,
                 0, 1, 0, 0,
                 0, 0, 1, 0};
}

static inline void matrix4_copy(real* dest, real* source) {
  memcpy(dest, source, sizeof(matrix4));
}

static inline void matrix4_set_diagonal(real* m1, real a, real b, real c) {
  m1[0] = a;
  m1[5] = b;
  m1[10] = c;
}

static inline void matrix4_mul_matrix4(real* m1, real* m2) {
  matrix4 temp;
  matrix4_copy(temp, m1);

  temp[0] = (m2[0] * m1[0]) + (m2[4] * m1[1]) + (m2[8] * m1[2]);
  temp[4] = (m2[0] * m1[4]) + (m2[4] * m1[5]) + (m2[8] * m1[6]);
  temp[8] = (m2[0] * m1[8]) + (m2[4] * m1[9]) + (m2[8] * m1[10]);

  temp[1] = (m2[1] * m1[0]) + (m2[5] * m1[1]) + (m2[9] * m1[2]);
  temp[5] = (m2[1] * m1[4]) + (m2[5] * m1[5]) + (m2[9] * m1[6]);
  temp[9] = (m2[1] * m1[8]) + (m2[5] * m1[9]) + (m2[9] * m1[10]);

  temp[2] = (m2[2] * m1[0]) + (m2[6] * m1[1]) + (m2[10] * m1[2]);
  temp[6] = (m2[2] * m1[4]) + (m2[6] * m1[5]) + (m2[10] * m1[6]);
  temp[10] = (m2[2] * m1[8]) + (m2[6] * m1[9]) + (m2[10] * m1[10]);

  temp[3] = (m2[3] * m1[0]) + (m2[7] * m1[1]) + (m2[11] * m1[2]) + m1[3];
  temp[7] = (m2[3] * m1[4]) + (m2[7] * m1[5]) + (m2[11] * m1[6]) + m1[7];
  temp[11] = (m2[3] * m1[8]) + (m2[7] * m1[9]) + (m2[11] * m1[10]) + m1[11];

  matrix4_copy(m1, temp);
}
//
//  Vector3 operator*(const Vector3& vector) const {
//    return Vector3(
//        vector.x * data[0] + vector.y * data[1] + vector.z * data[2] + data[3],
//
//        vector.x * data[4] + vector.y * data[5] + vector.z * data[6] + data[7],
//
//        vector.x * data[8] + vector.y * data[9] + vector.z * data[10] + data[11]);
//  }
//
//  Vector3 transform(const Vector3& vector) const {
//    return (*this) * vector;
//  }
//
//  real getDeterminant() const;
//
//  void setInverse(const Matrix4& m);
//
//  Matrix4 inverse() const {
//    Matrix4 result;
//    result.setInverse(*this);
//    return result;
//  }
//
//  void invert() {
//    setInverse(*this);
//  }
//
//  Vector3 transformDirection(const Vector3& vector) const {
//    return Vector3(
//        vector.x * data[0] + vector.y * data[1] + vector.z * data[2],
//
//        vector.x * data[4] + vector.y * data[5] + vector.z * data[6],
//
//        vector.x * data[8] + vector.y * data[9] + vector.z * data[10]);
//  }
//
//  Vector3 transformInverseDirection(const Vector3& vector) const {
//    return Vector3(
//        vector.x * data[0] + vector.y * data[4] + vector.z * data[8],
//
//        vector.x * data[1] + vector.y * data[5] + vector.z * data[9],
//
//        vector.x * data[2] + vector.y * data[6] + vector.z * data[10]);
//  }
//
//  Vector3 transformInverse(const Vector3& vector) const {
//    Vector3 tmp = vector;
//    tmp.x -= data[3];
//    tmp.y -= data[7];
//    tmp.z -= data[11];
//    return Vector3(
//        tmp.x * data[0] + tmp.y * data[4] + tmp.z * data[8],
//
//        tmp.x * data[1] + tmp.y * data[5] + tmp.z * data[9],
//
//        tmp.x * data[2] + tmp.y * data[6] + tmp.z * data[10]);
//  }
//
//  Vector3 getAxisVector(int i) const {
//    return Vector3(data[i], data[i + 4], data[i + 8]);
//  }
//
//  void setOrientationAndPos(const Quaternion& q, const Vector3& pos) {
//    data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
//    data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
//    data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
//    data[3] = pos.x;
//
//    data[4] = 2 * q.i * q.j - 2 * q.k * q.r;
//    data[5] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
//    data[6] = 2 * q.j * q.k + 2 * q.i * q.r;
//    data[7] = pos.y;
//
//    data[8] = 2 * q.i * q.k + 2 * q.j * q.r;
//    data[9] = 2 * q.j * q.k - 2 * q.i * q.r;
//    data[10] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
//    data[11] = pos.z;
//  }
//
//  void fillGLArray(float array[16]) const {
//    array[0] = (float)data[0];
//    array[1] = (float)data[4];
//    array[2] = (float)data[8];
//    array[3] = (float)0;
//
//    array[4] = (float)data[1];
//    array[5] = (float)data[5];
//    array[6] = (float)data[9];
//    array[7] = (float)0;
//
//    array[8] = (float)data[2];
//    array[9] = (float)data[6];
//    array[10] = (float)data[10];
//    array[11] = (float)0;
//
//    array[12] = (float)data[3];
//    array[13] = (float)data[7];
//    array[14] = (float)data[11];
//    array[15] = (float)1;
//  }
//};
////////////////////////////////////////////////
//class Matrix3 {
// public:
//  real data[9];
//
//  Matrix3() {
//    data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = data[6] = data[7] = data[8] = 0;
//  }
//
//  Matrix3(const Vector3& compOne, const Vector3& compTwo,
//          const Vector3& compThree) {
//    setComponents(compOne, compTwo, compThree);
//  }
//
//  Matrix3(real c0, real c1, real c2, real c3, real c4, real c5,
//          real c6, real c7, real c8) {
//    data[0] = c0;
//    data[1] = c1;
//    data[2] = c2;
//    data[3] = c3;
//    data[4] = c4;
//    data[5] = c5;
//    data[6] = c6;
//    data[7] = c7;
//    data[8] = c8;
//  }
//
//  void setDiagonal(real a, real b, real c) {
//    setInertiaTensorCoeffs(a, b, c);
//  }
//
//  void setInertiaTensorCoeffs(real ix, real iy, real iz,
//                              real ixy = 0, real ixz = 0, real iyz = 0) {
//    data[0] = ix;
//    data[1] = data[3] = -ixy;
//    data[2] = data[6] = -ixz;
//    data[4] = iy;
//    data[5] = data[7] = -iyz;
//    data[8] = iz;
//  }
//
//  void setBlockInertiaTensor(const Vector3& halfSizes, real mass) {
//    Vector3 squares = halfSizes.componentProduct(halfSizes);
//    setInertiaTensorCoeffs(0.3f * mass * (squares.y + squares.z),
//                           0.3f * mass * (squares.x + squares.z),
//                           0.3f * mass * (squares.x + squares.y));
//  }
//
//  void setSkewSymmetric(const Vector3 vector) {
//    data[0] = data[4] = data[8] = 0;
//    data[1] = -vector.z;
//    data[2] = vector.y;
//    data[3] = vector.z;
//    data[5] = -vector.x;
//    data[6] = -vector.y;
//    data[7] = vector.x;
//  }
//
//  void setComponents(const Vector3& compOne, const Vector3& compTwo,
//                     const Vector3& compThree) {
//    data[0] = compOne.x;
//    data[1] = compTwo.x;
//    data[2] = compThree.x;
//    data[3] = compOne.y;
//    data[4] = compTwo.y;
//    data[5] = compThree.y;
//    data[6] = compOne.z;
//    data[7] = compTwo.z;
//    data[8] = compThree.z;
//  }
//
//  Vector3 operator*(const Vector3& vector) const {
//    return Vector3(
//        vector.x * data[0] + vector.y * data[1] + vector.z * data[2],
//        vector.x * data[3] + vector.y * data[4] + vector.z * data[5],
//        vector.x * data[6] + vector.y * data[7] + vector.z * data[8]);
//  }
//
//  Vector3 transform(const Vector3& vector) const {
//    return (*this) * vector;
//  }
//
//  Vector3 transformTranspose(const Vector3& vector) const {
//    return Vector3(
//        vector.x * data[0] + vector.y * data[3] + vector.z * data[6],
//        vector.x * data[1] + vector.y * data[4] + vector.z * data[7],
//        vector.x * data[2] + vector.y * data[5] + vector.z * data[8]);
//  }
//
//  Vector3 getRowVector(int i) const {
//    return Vector3(data[i * 3], data[i * 3 + 1], data[i * 3 + 2]);
//  }
//
//  Vector3 getAxisVector(int i) const {
//    return Vector3(data[i], data[i + 3], data[i + 6]);
//  }
//
//  void setInverse(const Matrix3& m) {
//    real t4 = m.data[0] * m.data[4];
//    real t6 = m.data[0] * m.data[5];
//    real t8 = m.data[1] * m.data[3];
//    real t10 = m.data[2] * m.data[3];
//    real t12 = m.data[1] * m.data[6];
//    real t14 = m.data[2] * m.data[6];
//
//    real t16 = (t4 * m.data[8] - t6 * m.data[7] - t8 * m.data[8] + t10 * m.data[7] + t12 * m.data[5] - t14 * m.data[4]);
//
//    if (t16 == (real)0.0f)
//      return;
//    real t17 = 1 / t16;
//
//    data[0] = (m.data[4] * m.data[8] - m.data[5] * m.data[7]) * t17;
//    data[1] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7]) * t17;
//    data[2] = (m.data[1] * m.data[5] - m.data[2] * m.data[4]) * t17;
//    data[3] = -(m.data[3] * m.data[8] - m.data[5] * m.data[6]) * t17;
//    data[4] = (m.data[0] * m.data[8] - t14) * t17;
//    data[5] = -(t6 - t10) * t17;
//    data[6] = (m.data[3] * m.data[7] - m.data[4] * m.data[6]) * t17;
//    data[7] = -(m.data[0] * m.data[7] - t12) * t17;
//    data[8] = (t4 - t8) * t17;
//  }
//
//  Matrix3 inverse() const {
//    Matrix3 result;
//    result.setInverse(*this);
//    return result;
//  }
//
//  void invert() {
//    setInverse(*this);
//  }
//
//  void setTranspose(const Matrix3& m) {
//    data[0] = m.data[0];
//    data[1] = m.data[3];
//    data[2] = m.data[6];
//    data[3] = m.data[1];
//    data[4] = m.data[4];
//    data[5] = m.data[7];
//    data[6] = m.data[2];
//    data[7] = m.data[5];
//    data[8] = m.data[8];
//  }
//
//  Matrix3 transpose() const {
//    Matrix3 result;
//    result.setTranspose(*this);
//    return result;
//  }
//
//  Matrix3 operator*(const Matrix3& o) const {
//    return Matrix3(
//        data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6],
//        data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7],
//        data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8],
//
//        data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6],
//        data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7],
//        data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8],
//
//        data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6],
//        data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7],
//        data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8]);
//  }
//
//  void operator*=(const Matrix3& o) {
//    real t1;
//    real t2;
//    real t3;
//
//    t1 = data[0] * o.data[0] + data[1] * o.data[3] + data[2] * o.data[6];
//    t2 = data[0] * o.data[1] + data[1] * o.data[4] + data[2] * o.data[7];
//    t3 = data[0] * o.data[2] + data[1] * o.data[5] + data[2] * o.data[8];
//    data[0] = t1;
//    data[1] = t2;
//    data[2] = t3;
//
//    t1 = data[3] * o.data[0] + data[4] * o.data[3] + data[5] * o.data[6];
//    t2 = data[3] * o.data[1] + data[4] * o.data[4] + data[5] * o.data[7];
//    t3 = data[3] * o.data[2] + data[4] * o.data[5] + data[5] * o.data[8];
//    data[3] = t1;
//    data[4] = t2;
//    data[5] = t3;
//
//    t1 = data[6] * o.data[0] + data[7] * o.data[3] + data[8] * o.data[6];
//    t2 = data[6] * o.data[1] + data[7] * o.data[4] + data[8] * o.data[7];
//    t3 = data[6] * o.data[2] + data[7] * o.data[5] + data[8] * o.data[8];
//    data[6] = t1;
//    data[7] = t2;
//    data[8] = t3;
//  }
//
//  void operator*=(const real scalar) {
//    data[0] *= scalar;
//    data[1] *= scalar;
//    data[2] *= scalar;
//    data[3] *= scalar;
//    data[4] *= scalar;
//    data[5] *= scalar;
//    data[6] *= scalar;
//    data[7] *= scalar;
//    data[8] *= scalar;
//  }
//
//  void operator+=(const Matrix3& o) {
//    data[0] += o.data[0];
//    data[1] += o.data[1];
//    data[2] += o.data[2];
//    data[3] += o.data[3];
//    data[4] += o.data[4];
//    data[5] += o.data[5];
//    data[6] += o.data[6];
//    data[7] += o.data[7];
//    data[8] += o.data[8];
//  }
//
//  void setOrientation(const Quaternion& q) {
//    data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
//    data[1] = 2 * q.i * q.j + 2 * q.k * q.r;
//    data[2] = 2 * q.i * q.k - 2 * q.j * q.r;
//    data[3] = 2 * q.i * q.j - 2 * q.k * q.r;
//    data[4] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
//    data[5] = 2 * q.j * q.k + 2 * q.i * q.r;
//    data[6] = 2 * q.i * q.k + 2 * q.j * q.r;
//    data[7] = 2 * q.j * q.k - 2 * q.i * q.r;
//    data[8] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
//  }
//
//  static Matrix3 linearInterpolate(const Matrix3& a, const Matrix3& b, real prop);
//};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//
//real Matrix4::getDeterminant() const
//{
//    return -data[8] * data[5] * data[2] + data[4] * data[9] * data[2] + data[8] * data[1] * data[6] - data[0] * data[9] * data[6] - data[4] * data[1] * data[10] + data[0] * data[5] * data[10];
//}
//
//void Matrix4::setInverse(const Matrix4& m)
//{
//    // Make sure the determinant is non-zero.
//    real det = getDeterminant();
//    if (det == 0)
//        return;
//    det = ((real)1.0) / det;
//
//    data[0] = (-m.data[9] * m.data[6] + m.data[5] * m.data[10]) * det;
//    data[4] = (m.data[8] * m.data[6] - m.data[4] * m.data[10]) * det;
//    data[8] = (-m.data[8] * m.data[5] + m.data[4] * m.data[9]) * det;
//
//    data[1] = (m.data[9] * m.data[2] - m.data[1] * m.data[10]) * det;
//    data[5] = (-m.data[8] * m.data[2] + m.data[0] * m.data[10]) * det;
//    data[9] = (m.data[8] * m.data[1] - m.data[0] * m.data[9]) * det;
//
//    data[2] = (-m.data[5] * m.data[2] + m.data[1] * m.data[6]) * det;
//    data[6] = (+m.data[4] * m.data[2] - m.data[0] * m.data[6]) * det;
//    data[10] = (-m.data[4] * m.data[1] + m.data[0] * m.data[5]) * det;
//
//    data[3] = (m.data[9] * m.data[6] * m.data[3]
//                  - m.data[5] * m.data[10] * m.data[3]
//                  - m.data[9] * m.data[2] * m.data[7]
//                  + m.data[1] * m.data[10] * m.data[7]
//                  + m.data[5] * m.data[2] * m.data[11]
//                  - m.data[1] * m.data[6] * m.data[11])
//        * det;
//    data[7] = (-m.data[8] * m.data[6] * m.data[3]
//                  + m.data[4] * m.data[10] * m.data[3]
//                  + m.data[8] * m.data[2] * m.data[7]
//                  - m.data[0] * m.data[10] * m.data[7]
//                  - m.data[4] * m.data[2] * m.data[11]
//                  + m.data[0] * m.data[6] * m.data[11])
//        * det;
//    data[11] = (m.data[8] * m.data[5] * m.data[3]
//                   - m.data[4] * m.data[9] * m.data[3]
//                   - m.data[8] * m.data[1] * m.data[7]
//                   + m.data[0] * m.data[9] * m.data[7]
//                   + m.data[4] * m.data[1] * m.data[11]
//                   - m.data[0] * m.data[5] * m.data[11])
//        * det;
//}
//
//Matrix3 Matrix3::linearInterpolate(const Matrix3& a, const Matrix3& b, real prop)
//{
//    Matrix3 result;
//    for (unsigned i = 0; i < 9; i++) {
//        result.data[i] = a.data[i] * (1 - prop) + b.data[i] * prop;
//    }
//    return result;
//}

#endif  // CORE_H
