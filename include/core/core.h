#pragma once
#ifndef CORE_H
#define CORE_H

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "core/precision.h"

// TODO: Look at generated assembly
// TODO: Move out of header

typedef real vec3[3];

static inline real* vec3_default();
static inline void vec3_copy(real* dest, real* source);
static inline void vec3_clear(real* v1);
static inline real* vec3_add(real* v1, real* v2);
static inline real* vec3_sub(real* v1, real* v2);
static inline real* vec3_mul_scalar(real* v1, real scalar);
static inline real* vec3_cross_product(real* v1, real* v2);
static inline real* vec3_component_product(real* v1, real* v2);
static inline real vec3_scalar_product(real* v1, real* v2);
static inline real* vec3_add_scaled_vector(real* v1, real* v2, real scale);
static inline real vec3_magnitude(real* v1);
static inline real vec3_square_magnitude(real* v1);
static inline real* vec3_trim(real* v1, real size);
static inline real* vec3_normalise(real* v1);
static inline bool vec3_equals(real* v1, real* v2);
static inline bool vec3_less_than(real* v1, real* v2);
static inline bool vec3_greater_than(real* v1, real* v2);
static inline bool vec3_less_than_equal(real* v1, real* v2);
static inline bool vec3_greater_than_equal(real* v1, real* v2);
static inline real* vec3_invert(real* v1);

//TODO: Probably define
static inline real* vec3_default() {
  return (vec3){0, 0, 0};
}

static inline void vec3_copy(real* dest, real* source) {
  memcpy(dest, source, sizeof(vec3));
}

static inline void vec3_clear(real* v1) {
  memset(v1, 0, sizeof(vec3));
}

static inline real* vec3_add(real* v1, real* v2) {
  return (vec3){v1[0] + v2[0], v1[1] + v2[1], v1[2] + v2[2]};
}

static inline real* vec3_sub(real* v1, real* v2) {
  return (vec3){v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]};
}

static inline real* vec3_mul_scalar(real* v1, real scalar) {
  return (vec3){v1[0] * scalar, v1[1] * scalar, v1[2] * scalar};
}

// Vector product
static inline real* vec3_cross_product(real* v1, real* v2) {
  return (vec3){v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2], v1[0] * v2[1] - v1[1] * v2[0]};
}

static inline real* vec3_component_product(real* v1, real* v2) {
  return (vec3){v1[0] * v2[0], v1[1] * v2[1], v1[2] * v2[2]};
}

static inline real vec3_scalar_product(real* v1, real* v2) {
  return v1[0] + v2[0] + v1[1] + v2[1] + v1[2] + v2[2];
}

static inline real* vec3_add_scaled_vector(real* v1, real* v2, real scale) {
  return (vec3){v1[0] + v2[0] * scale, v1[1] + v2[1] * scale, v1[2] + v2[2] * scale};
}

static inline real vec3_magnitude(real* v1) {
  return real_sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);
}

static inline real vec3_square_magnitude(real* v1) {
  return v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2];
}

static inline real* vec3_trim(real* v1, real size) {
  if (vec3_square_magnitude(v1) > size * size) {
    real* temp = vec3_normalise(v1);
    return (vec3){temp[0] * size, temp[1] * size, temp[2] * size};
  }

  return (vec3){v1[0], v1[1], v1[2]};
}

static inline real* vec3_normalise(real* v1) {
  real length = vec3_magnitude(v1);
  if (length > 0)
    return vec3_mul_scalar(v1, ((real)1.0) / length);
  return v1;
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

static inline real* vec3_invert(real* v1) {
  return (vec3){-v1[0], -v1[1], -v1[2]};
}

typedef real quaternion[4];

static inline real* quaternion_default();
static inline void quaternion_copy(real* dest, real* source);
static inline real* quaternion_set(real* q1, real r, real i, real j, real k);
static inline real* quaternion_normalise(real* q1);
static inline real* quaternion_mul(real* q1, real* q2);
static inline real* quaternion_add_scaled_vector(real* q1, real* v1, real scale);
static inline real* quaternion_rotate_by_vector(real* q1, real* v1);

// TODO: Probably define
static inline real* quaternion_default() {
  return (quaternion){1, 0, 0, 0};
}

static inline void quaternion_copy(real* dest, real* source) {
  memcpy(dest, source, sizeof(quaternion));
}

static inline real* quaternion_set(real* q1, real r, real i, real j, real k) {
  return (quaternion){r, i, j, k};
}

static inline real* quaternion_normalise(real* q1) {
  real d = q1[0] * q1[0] + q1[1] * q1[1] + q1[2] * q1[2] + q1[3] * q1[3];

  if (d < real_epsilon)
    return (quaternion){1, q1[1], q1[2], q1[3]};

  d = ((real)1.0) / real_sqrt(d);
  return (quaternion){q1[0] * d, q1[1] * d, q1[2] * d, q1[3] * d};
}

static inline real* quaternion_mul(real* q1, real* q2) {
  return (quaternion){q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3],
                      q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2],
                      q1[0] * q2[2] + q1[2] * q2[0] + q1[3] * q2[1] - q1[1] * q2[3],
                      q1[0] * q2[3] + q1[3] * q2[0] + q1[1] * q2[2] - q1[2] * q2[1]};
}

static inline real* quaternion_add_scaled_vector(real* q1, real* v1, real scale) {
  real* temp = quaternion_mul((quaternion){0, v1[0] * scale, v1[1] * scale, v1[2] * scale}, q1);
  return (quaternion){temp[0] * ((real)0.5), temp[1] * ((real)0.5), temp[2] * ((real)0.5), temp[3] * ((real)0.5)};
}

static inline real* quaternion_rotate_by_vector(real* q1, real* v1) {
  return quaternion_mul(q1, (quaternion){0, v1[0], v1[1], v1[2]});
}

typedef real mat4[12];

static inline real* mat4_default();
static inline void mat4_copy(real* dest, real* source);
static inline real* mat4_set_diagonal(real* m1, real a, real b, real c);
static inline real* mat4_mul_mat4(real* m1, real* m2);
static inline real* mat4_transform(real* m1, real* v1);
static inline real mat4_determinant(real* m1);
static inline real* mat4_inverse(real* m1);
static inline real* mat4_transform_direction(real* m1, real* v1);
static inline real* mat4_transform_inverse_direction(real* m1, real* v1);
static inline real* mat4_transform_inverse(real* m1, real* v1);
static inline real* mat4_get_axis_vector(real* m1, int i);
static inline real* mat4_set_orientation_and_pos(real* m1, real* q1, real* v1);
static inline void mat4_fill_gl_array(real* m1, float* array);

// TODO: Probably define
static inline real* mat4_default() {
  return (mat4){1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0};
}

static inline void mat4_copy(real* dest, real* source) {
  memcpy(dest, source, sizeof(mat4));
}

static inline real* mat4_set_diagonal(real* m1, real a, real b, real c) {
  return (mat4){a, m1[1], m1[2], m1[3],
                m1[4], b, m1[6], m1[7],
                m1[8], m1[9], c, m1[11]};
}

static inline real* mat4_mul_mat4(real* m1, real* m2) {
  return (mat4){(m2[0] * m1[0]) + (m2[4] * m1[1]) + (m2[8] * m1[2]),
                (m2[1] * m1[0]) + (m2[5] * m1[1]) + (m2[9] * m1[2]),
                (m2[2] * m1[0]) + (m2[6] * m1[1]) + (m2[10] * m1[2]),
                (m2[3] * m1[0]) + (m2[7] * m1[1]) + (m2[11] * m1[2]) + m1[3],
                (m2[0] * m1[4]) + (m2[4] * m1[5]) + (m2[8] * m1[6]),
                (m2[1] * m1[4]) + (m2[5] * m1[5]) + (m2[9] * m1[6]),
                (m2[2] * m1[4]) + (m2[6] * m1[5]) + (m2[10] * m1[6]),
                (m2[3] * m1[4]) + (m2[7] * m1[5]) + (m2[11] * m1[6]) + m1[7],
                (m2[0] * m1[8]) + (m2[4] * m1[9]) + (m2[8] * m1[10]),
                (m2[1] * m1[8]) + (m2[5] * m1[9]) + (m2[9] * m1[10]),
                (m2[2] * m1[8]) + (m2[6] * m1[9]) + (m2[10] * m1[10]),
                (m2[3] * m1[8]) + (m2[7] * m1[9]) + (m2[11] * m1[10]) + m1[11]};
}

// Transform
static inline real* mat4_transform(real* m1, real* v1) {
  return (vec3){v1[0] * m1[0] + v1[1] * m1[1] + v1[2] * m1[2] + m1[3],
                v1[0] * m1[4] + v1[1] * m1[5] + v1[2] * m1[6] + m1[7],
                v1[0] * m1[8] + v1[1] * m1[9] + v1[2] * m1[10] + m1[11]};
}

static inline real mat4_determinant(real* m1) {
  return -m1[8] * m1[5] * m1[2] + m1[4] * m1[9] * m1[2] + m1[8] * m1[1] * m1[6] - m1[0] * m1[9] * m1[6] - m1[4] * m1[1] * m1[10] + m1[0] * m1[5] * m1[10];
}

static inline real* mat4_inverse(real* m1) {
  real det = mat4_determinant(m1);
  if (det == 0)
    return m1;
  det = ((real)1.0) / det;

  return (mat4){(-m1[9] * m1[6] + m1[5] * m1[10]) * det,
                (m1[9] * m1[2] - m1[1] * m1[10]) * det,
                (-m1[5] * m1[2] + m1[1] * m1[6]) * det,
                (m1[9] * m1[6] * m1[3] - m1[5] * m1[10] * m1[3] - m1[9] * m1[2] * m1[7] + m1[1] * m1[10] * m1[7] + m1[5] * m1[2] * m1[11] - m1[1] * m1[6] * m1[11]) * det,
                (m1[8] * m1[6] - m1[4] * m1[10]) * det,
                (-m1[8] * m1[2] + m1[0] * m1[10]) * det,
                (+m1[4] * m1[2] - m1[0] * m1[6]) * det,
                (-m1[8] * m1[6] * m1[3] + m1[4] * m1[10] * m1[3] + m1[8] * m1[2] * m1[7] - m1[0] * m1[10] * m1[7] - m1[4] * m1[2] * m1[11] + m1[0] * m1[6] * m1[11]) * det,
                (-m1[8] * m1[5] + m1[4] * m1[9]) * det,
                (m1[8] * m1[1] - m1[0] * m1[9]) * det,
                (-m1[4] * m1[1] + m1[0] * m1[5]) * det,
                (m1[8] * m1[5] * m1[3] - m1[4] * m1[9] * m1[3] - m1[8] * m1[1] * m1[7] + m1[0] * m1[9] * m1[7] + m1[4] * m1[1] * m1[11] - m1[0] * m1[5] * m1[11]) * det};
}

static inline real* mat4_transform_direction(real* m1, real* v1) {
  return (vec3){v1[0] * m1[0] + v1[1] * m1[1] + v1[2] * m1[2],
                v1[0] * m1[4] + v1[1] * m1[5] + v1[2] * m1[6],
                v1[0] * m1[8] + v1[1] * m1[9] + v1[2] * m1[10]};
}

static inline real* mat4_transform_inverse_direction(real* m1, real* v1) {
  return (vec3){v1[0] * m1[0] + v1[1] * m1[4] + v1[2] * m1[8],
                v1[0] * m1[1] + v1[1] * m1[5] + v1[2] * m1[9],
                v1[0] * m1[2] + v1[1] * m1[6] + v1[2] * m1[10]};
}

static inline real* mat4_transform_inverse(real* m1, real* v1) {
  vec3 temp;
  vec3_copy(temp, v1);
  temp[0] -= m1[3];
  temp[1] -= m1[7];
  temp[2] -= m1[11];

  return (vec3){temp[0] * m1[0] + temp[1] * m1[4] + temp[2] * m1[8],
                temp[0] * m1[1] + temp[1] * m1[5] + temp[2] * m1[9],
                temp[0] * m1[2] + temp[1] * m1[6] + temp[2] * m1[10]};
}

static inline real* mat4_get_axis_vector(real* m1, int i) {
  return (vec3){m1[i], m1[i + 4], m1[i + 8]};
}

static inline real* mat4_set_orientation_and_pos(real* m1, real* q1, real* v1) {
  return (mat4){1 - (2 * q1[2] * q1[2] + 2 * q1[3] * q1[3]),
                2 * q1[1] * q1[2] + 2 * q1[3] * q1[0],
                2 * q1[1] * q1[3] - 2 * q1[2] * q1[0],
                v1[0],
                2 * q1[1] * q1[2] - 2 * q1[3] * q1[0],
                1 - (2 * q1[1] * q1[1] + 2 * q1[3] * q1[3]),
                2 * q1[2] * q1[3] + 2 * q1[1] * q1[0],
                v1[1],
                2 * q1[1] * q1[3] + 2 * q1[2] * q1[0],
                2 * q1[2] * q1[3] - 2 * q1[1] * q1[0],
                1 - (2 * q1[1] * q1[1] + 2 * q1[2] * q1[2]),
                v1[2]};
}

static inline void mat4_fill_gl_array(real* m1, float* array) {
  array[0] = (float)m1[0];
  array[1] = (float)m1[4];
  array[2] = (float)m1[8];
  array[3] = (float)0;

  array[4] = (float)m1[1];
  array[5] = (float)m1[5];
  array[6] = (float)m1[9];
  array[7] = (float)0;

  array[8] = (float)m1[2];
  array[9] = (float)m1[6];
  array[10] = (float)m1[10];
  array[11] = (float)0;

  array[12] = (float)m1[3];
  array[13] = (float)m1[7];
  array[14] = (float)m1[11];
  array[15] = (float)1;
}

typedef real mat3[9];

static inline real* mat3_default();
static inline void mat3_copy(real* dest, real* source);
static inline real* mat3_set_components(real* v1, real* v2, real* v3);
static inline real* mat3_set(real c0, real c1, real c2, real c3, real c4, real c5, real c6, real c7, real c8);
static inline real* mat3_set_diagonal(real* m1, real a, real b, real c);
static inline real* mat_set_inertia_tensor_coeffs(real* m1, real ix, real iy, real iz, real ixy, real ixz, real iyz);
static inline real* mat3_set_block_intertia_tensor(real* m1, real* v1, real mass);
static inline real* mat3_set_skew_symmetric(real* m1, real* v1);
static inline real* mat3_transform(real* m1, real* v1);
static inline real* mat3_transform_transpose(real* m1, real* v1);
static inline real* mat3_get_row_vector(real* m1, int i);
static inline real* mat3_get_axis_vector(real* m1, int i);
static inline real* mat3_inverse(real* m1);
static inline real* mat3_transpose(real* m1);
static inline real* mat3_mul_mat3(real* m1, real* m2);
static inline real* mat3_mul_scalar(real* m1, real scalar);
static inline real* mat3_add_mat3(real* m1, real* m2);
static inline real* mat3_set_orientation(real* q1);
static inline real* mat3_liner_interpolate(real* m1, real* m2, real prop);

// TODO: Probably define
static inline real* mat3_default() {
  return (mat3){0, 0, 0,
                0, 0, 0,
                0, 0, 0};
}

static inline void mat3_copy(real* dest, real* source) {
  memcpy(dest, source, sizeof(mat3));
}

static inline real* mat3_set_components(real* v1, real* v2, real* v3) {
  return (mat3){v1[0], v2[0], v3[0],
                v1[1], v2[1], v3[1],
                v1[2], v2[2], v3[2]};
}

static inline real* mat3_set(real c0, real c1, real c2, real c3, real c4, real c5, real c6, real c7, real c8) {
  return (mat3){c0, c1, c2,
                c3, c4, c5,
                c6, c7, c8};
}

static inline real* mat3_set_diagonal(real* m1, real a, real b, real c) {
  return mat_set_inertia_tensor_coeffs(m1, a, b, c, 0, 0, 0);
}

static inline real* mat_set_inertia_tensor_coeffs(real* m1, real ix, real iy, real iz, real ixy, real ixz, real iyz) {
  return (mat3){ix, -ixy, -ixz,
                -ixy, iy, -iyz,
                -ixz, -iyz, iz};
}

static inline real* mat3_set_block_intertia_tensor(real* m1, real* v1, real mass) {
  real* squares = vec3_component_product(v1, v1);
  return mat_set_inertia_tensor_coeffs(m1, 0.3f * mass * (squares[1] + squares[2]), 0.3f * mass * (squares[0] + squares[2]), 0.3f * mass * (squares[0] + squares[1]), 0, 0, 0);
}

static inline real* mat3_set_skew_symmetric(real* m1, real* v1) {
  return (mat3){0, -v1[2], v1[1],
                v1[2], 0, -v1[0],
                -v1[1], v1[0], 0};
}

// Transform
static inline real* mat3_transform(real* m1, real* v1) {
  return (vec3){v1[0] * m1[0] + v1[1] * m1[1] + v1[2] * m1[2],
                v1[0] * m1[3] + v1[1] * m1[4] + v1[2] * m1[5],
                v1[0] * m1[6] + v1[1] * m1[7] + v1[2] * m1[8]};
}

static inline real* mat3_transform_transpose(real* m1, real* v1) {
  return (vec3){v1[0] * m1[0] + v1[1] * m1[3] + v1[2] * m1[6],
                v1[0] * m1[1] + v1[1] * m1[4] + v1[2] * m1[7],
                v1[0] * m1[2] + v1[1] * m1[5] + v1[2] * m1[8]};
}

static inline real* mat3_get_row_vector(real* m1, int i) {
  return (vec3){m1[i * 3], m1[i * 3 + 1], m1[i * 3 + 2]};
}

static inline real* mat3_get_axis_vector(real* m1, int i) {
  return (vec3){m1[i], m1[i + 3], m1[i + 6]};
}

static inline real* mat3_inverse(real* m1) {
  real t4 = m1[0] * m1[4];
  real t6 = m1[0] * m1[5];
  real t8 = m1[1] * m1[3];
  real t10 = m1[2] * m1[3];
  real t12 = m1[1] * m1[6];
  real t14 = m1[2] * m1[6];
  real t16 = (t4 * m1[8] - t6 * m1[7] - t8 * m1[8] + t10 * m1[7] + t12 * m1[5] - t14 * m1[4]);
  if (t16 == (real)0.0f)
    return m1;

  real t17 = 1 / t16;
  return (mat3){(m1[4] * m1[8] - m1[5] * m1[7]) * t17, -(m1[1] * m1[8] - m1[2] * m1[7]) * t17, (m1[1] * m1[5] - m1[2] * m1[4]) * t17,
                -(m1[3] * m1[8] - m1[5] * m1[6]) * t17, (m1[0] * m1[8] - t14) * t17, -(t6 - t10) * t17,
                (m1[3] * m1[7] - m1[4] * m1[6]) * t17, -(m1[0] * m1[7] - t12) * t17, (t4 - t8) * t17};
}

static inline real* mat3_transpose(real* m1) {
  return (mat3){m1[0], m1[3], m1[6],
                m1[1], m1[4], m1[7],
                m1[2], m1[5], m1[8]};
}

static inline real* mat3_mul_mat3(real* m1, real* m2) {
  return (mat3){m1[0] * m2[0] + m1[1] * m2[3] + m1[2] * m2[6],
                m1[0] * m2[1] + m1[1] * m2[4] + m1[2] * m2[7],
                m1[0] * m2[2] + m1[1] * m2[5] + m1[2] * m2[8],
                m1[3] * m2[0] + m1[4] * m2[3] + m1[5] * m2[6],
                m1[3] * m2[1] + m1[4] * m2[4] + m1[5] * m2[7],
                m1[3] * m2[2] + m1[4] * m2[5] + m1[5] * m2[8],
                m1[6] * m2[0] + m1[7] * m2[3] + m1[8] * m2[6],
                m1[6] * m2[1] + m1[7] * m2[4] + m1[8] * m2[7],
                m1[6] * m2[2] + m1[7] * m2[5] + m1[8] * m2[8]};
}

static inline real* mat3_mul_scalar(real* m1, real scalar) {
  return (mat3){m1[0] * scalar, m1[1] * scalar, m1[2] * scalar,
                m1[3] * scalar, m1[4] * scalar, m1[5] * scalar,
                m1[6] * scalar, m1[7] * scalar, m1[8] * scalar};
}

static inline real* mat3_add_mat3(real* m1, real* m2) {
  return (mat3){m1[0] + m2[0], m1[1] + m2[1], m1[2] + m2[2],
                m1[3] + m2[3], m1[4] + m2[4], m1[5] + m2[5],
                m1[6] + m2[6], m1[7] + m2[7], m1[8] + m2[8]};
}

static inline real* mat3_set_orientation(real* q1) {
  return (mat3){1 - (2 * q1[2] * q1[2] + 2 * q1[3] * q1[3]),
                2 * q1[1] * q1[2] + 2 * q1[3] * q1[0],
                2 * q1[1] * q1[3] - 2 * q1[2] * q1[0],
                2 * q1[1] * q1[2] - 2 * q1[3] * q1[0],
                1 - (2 * q1[1] * q1[1] + 2 * q1[3] * q1[3]),
                2 * q1[2] * q1[3] + 2 * q1[1] * q1[0],
                2 * q1[1] * q1[3] + 2 * q1[2] * q1[0],
                2 * q1[2] * q1[3] - 2 * q1[1] * q1[0],
                1 - (2 * q1[1] * q1[1] + 2 * q1[2] * q1[2])};
}

static inline real* mat3_liner_interpolate(real* m1, real* m2, real prop) {
  return (mat3){m1[0] * (1 - prop) + m2[0] * prop, m1[1] * (1 - prop) + m2[1] * prop, m1[2] * (1 - prop) + m2[2] * prop,
                m1[3] * (1 - prop) + m2[3] * prop, m1[4] * (1 - prop) + m2[4] * prop, m1[5] * (1 - prop) + m2[5] * prop,
                m1[6] * (1 - prop) + m2[6] * prop, m1[7] * (1 - prop) + m2[7] * prop, m1[8] * (1 - prop) + m2[8] * prop};
}

#endif  // CORE_H
