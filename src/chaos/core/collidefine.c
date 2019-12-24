#include "chaos/core/collidefine.h"

real transform_to_axis(struct CollisionBox* box, real* axis) {
  return box->half_size[0] * real_abs(vec3_magnitude(vec3_component_product(axis, collision_primitive_get_axis(&box->collision_primitive, 0)))) + box->half_size[1] * real_abs(vec3_magnitude(vec3_component_product(axis, collision_primitive_get_axis(&box->collision_primitive, 1)))) + box->half_size[2] * real_abs(vec3_magnitude(vec3_component_product(axis, collision_primitive_get_axis(&box->collision_primitive, 2))));
}

bool overlap_on_axis(struct CollisionBox* one, struct CollisionBox* two, real* axis, real* to_centre) {
  real one_project = transform_to_axis(one, axis);
  real two_project = transform_to_axis(two, axis);
  real distance = real_abs(vec3_magnitude(vec3_component_product(to_centre, axis)));

  return (distance < one_project + two_project);
}

void collision_primitive_calculate_internals(struct CollisionPrimitive* collision_primitive) {
  mat4_copy(collision_primitive->transform, mat4_mul_mat4(rigid_body_get_transform(collision_primitive->body), collision_primitive->offset));
}

real* collision_primitive_get_axis(struct CollisionPrimitive* collision_primitive, unsigned int index) {
  return mat4_get_axis_vector(collision_primitive->transform, index);
}
real* collision_primitive_get_transform(struct CollisionPrimitive* collision_primitive) {
  return collision_primitive->transform;
}

bool intersection_test_box_and_half_space(struct CollisionBox* box, struct CollisionPlane* plane);

bool intersection_test_sphere_and_half_space(struct CollisionSphere* sphere, struct CollisionPlane* plane) {
  real ball_distance = vec3_magnitude(vec3_component_product(plane->direction, collision_primitive_get_axis(&sphere->collision_primitive, 3))) - sphere->radius;

  return ball_distance <= plane->offset;
}

bool intersection_test_sphere_and_sphere(struct CollisionSphere* one, struct CollisionSphere* two) {
  return vec3_square_magnitude(vec3_sub(collision_primitive_get_axis(&one->collision_primitive, 3), collision_primitive_get_axis(&two->collision_primitive, 3))) < (one->radius + two->radius) * (one->radius + two->radius);
}

/*#define TEST_OVERLAP(axis) overlap_on_axis(one, two, (axis), to_centre)

 bool intersection_test_box_and_box(struct CollisionBox* one, struct CollisionBox* two) {
  Vector3 to_centre = two.getAxis(3) - one.getAxis(3);
  //collision_primitive_get_axis(box, 0)

  //vec3 to_centre =
  real* to_centre = vec3_sub(collision_primitive_get_axis(two, 3), collision_primitive_get_axis(one, 3));

  return (
      TEST_OVERLAP(one.getAxis(0)) && TEST_OVERLAP(one.getAxis(1)) && TEST_OVERLAP(one.getAxis(2)) && TEST_OVERLAP(two.getAxis(0)) && TEST_OVERLAP(two.getAxis(1)) && TEST_OVERLAP(two.getAxis(2)) && TEST_OVERLAP(one.getAxis(0) % two.getAxis(0)) && TEST_OVERLAP(one.getAxis(0) % two.getAxis(1)) && TEST_OVERLAP(one.getAxis(0) % two.getAxis(2)) && TEST_OVERLAP(one.getAxis(1) % two.getAxis(0)) && TEST_OVERLAP(one.getAxis(1) % two.getAxis(1)) && TEST_OVERLAP(one.getAxis(1) % two.getAxis(2)) && TEST_OVERLAP(one.getAxis(2) % two.getAxis(0)) && TEST_OVERLAP(one.getAxis(2) % two.getAxis(1)) && TEST_OVERLAP(one.getAxis(2) % two.getAxis(2)));
}
#undef TEST_OVERLAP*/

bool intersection_test_box_and_half_space(struct CollisionBox* box, struct CollisionPlane* plane) {
  real projected_radius = transform_to_axis(box, plane->direction);
  real box_distance = vec3_magnitude(vec3_component_product(plane->direction, collision_primitive_get_axis(&box->collision_primitive, 3))) - projected_radius;

  return box_distance <= plane->offset;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

bool collision_data_has_more_contacts(struct CollisionData* collision_data) {
  return collision_data->contacts_left > 0;
}

void collision_data_reset(struct CollisionData* collision_data, unsigned int max_contacts) {
  collision_data->contacts_left = max_contacts;
  collision_data->contact_count = 0;
  collision_data->contacts = collision_data->contact_array;
}

void collision_data_add_contacts(struct CollisionData* collision_data, unsigned int count) {
  collision_data->contacts_left -= count;
  collision_data->contact_count += count;
  collision_data->contacts += count;
}

unsigned int collision_detector_sphere_and_true_plane(struct CollisionSphere* sphere, struct CollisionPlane* plane, struct CollisionData* data) {
  if (data->contacts_left <= 0)
    return 0;

  vec3 position;
  vec3_copy(position, collision_primitive_get_axis(&sphere->collision_primitive, 3));

  real centre_distance = vec3_magnitude(vec3_component_product(plane->direction, position)) - plane->offset;
  if (centre_distance * centre_distance > sphere->radius * sphere->radius)
    return 0;

  vec3 normal;
  vec3_copy(normal, plane->direction);

  real penetration = -centre_distance;
  if (centre_distance < 0) {
    vec3_copy(normal, vec3_mul_scalar(normal, -1));
    penetration = -penetration;
  }
  penetration += sphere->radius;

  struct Contact* contact = data->contacts;
  vec3_copy(contact->contact_normal, normal);
  contact->penetration = penetration;
  vec3_copy(contact->contact_point, vec3_sub(position, vec3_mul_scalar(plane->direction, centre_distance)));
  contact_set_body_data(contact, sphere->collision_primitive.body, NULL, data->friction, data->restitution);

  collision_data_add_contacts(data, 1);

  return 1;
}

unsigned int collision_detector_sphere_and_half_space(struct CollisionSphere* sphere, struct CollisionPlane* plane, struct CollisionData* data) {
  if (data->contacts_left <= 0)
    return 0;

  vec3 position;
  vec3_copy(position, collision_primitive_get_axis(&sphere->collision_primitive, 3));

  real ball_distance = vec3_magnitude(vec3_component_product(plane->direction, position)) - sphere->radius - plane->offset;
  if (ball_distance >= 0)
    return 0;

  struct Contact* contact = data->contacts;
  vec3_copy(contact->contact_normal, plane->direction);
  contact->penetration = -ball_distance;
  vec3_copy(contact->contact_point, vec3_sub(position, vec3_mul_scalar(plane->direction, ball_distance + sphere->radius)));
  contact_set_body_data(contact, sphere->collision_primitive.body, NULL, data->friction, data->restitution);

  collision_data_add_contacts(data, 1);

  return 1;
}

unsigned int collision_detector_sphere_and_sphere(struct CollisionSphere* one, struct CollisionSphere* two, struct CollisionData* data) {
  if (data->contacts_left <= 0)
    return 0;

  vec3 position_one;
  vec3_copy(position_one, collision_primitive_get_axis(&one->collision_primitive, 3));

  vec3 position_two;
  vec3_copy(position_two, collision_primitive_get_axis(&two->collision_primitive, 3));

  vec3 midline;
  vec3_copy(midline, vec3_sub(position_one, position_two));

  real size = vec3_magnitude(midline);
  if (size <= 0.0f || size >= one->radius + two->radius)
    return 0;

  vec3 normal;
  vec3_copy(normal, vec3_mul_scalar(midline, ((real)1.0) / size));

  struct Contact* contact = data->contacts;
  vec3_copy(contact->contact_normal, normal);
  vec3_copy(contact->contact_point, vec3_add(position_one, vec3_mul_scalar(midline, (real)0.5)));
  contact->penetration = (one->radius + two->radius - size);
  contact_set_body_data(contact, one->collision_primitive.body, two->collision_primitive.body, data->friction, data->restitution);

  collision_data_add_contacts(data, 1);

  return 1;
}

real penetration_on_axis(struct CollisionBox* one, struct CollisionBox* two, real* axis, real* to_centre) {
  real one_project = transform_to_axis(one, axis);
  real two_project = transform_to_axis(two, axis);
  real distance = real_abs(vec3_magnitude(vec3_component_product(to_centre, axis)));

  return one_project + two_project - distance;
}

bool try_axis(struct CollisionBox* one, struct CollisionBox* two, real* axis, real* to_centre, unsigned int index, real* smallest_penetration, unsigned int* smallest_case) {
  if (vec3_square_magnitude(axis) < 0.0001)
    return true;

  vec3_copy(axis, vec3_normalise(axis));

  real penetration = penetration_on_axis(one, two, axis, to_centre);

  if (penetration < 0)
    return false;
  if (penetration < *smallest_penetration) {
    *smallest_penetration = penetration;
    *smallest_case = index;
  }
  return true;
}

void fill_point_face_box_box(struct CollisionBox* one, struct CollisionBox* two, real* to_centre, struct CollisionData* data, unsigned int best, real pen) {
  struct Contact* contact = data->contacts;

  vec3 normal;
  vec3_copy(normal, collision_primitive_get_axis(&one->collision_primitive, best));

  if (vec3_magnitude(vec3_component_product(collision_primitive_get_axis(&one->collision_primitive, best), to_centre)) > 0)
    vec3_copy(normal, vec3_mul_scalar(normal, -1.0f));

  vec3 vertex;
  vec3_copy(vertex, two->half_size);

  if (vec3_magnitude(vec3_component_product(collision_primitive_get_axis(&two->collision_primitive, 0), normal)) < 0)
    vertex[0] = -vertex[0];
  if (vec3_magnitude(vec3_component_product(collision_primitive_get_axis(&two->collision_primitive, 1), normal)) < 0)
    vertex[1] = -vertex[1];
  if (vec3_magnitude(vec3_component_product(collision_primitive_get_axis(&two->collision_primitive, 2), normal)) < 0)
    vertex[2] = -vertex[2];

  vec3_copy(contact->contact_normal, normal);
  contact->penetration = pen;
  vec3_copy(contact->contact_point, vec3_component_product(collision_primitive_get_transform(&two->collision_primitive), vertex));
  contact_set_body_data(contact, one->collision_primitive.body, two->collision_primitive.body, data->friction, data->restitution);
}

real* contact_point(real* p_one, real* d_one, real one_size, real* p_two, real* d_two, real two_size, bool use_one) {
  vec3 to_st, c_one, c_two;
  real dp_sta_one, dp_sta_two, dp_one_two, sm_one, sm_two;
  real denom, mua, mub;

  sm_one = vec3_square_magnitude(d_one);
  sm_two = vec3_square_magnitude(d_two);
  dp_one_two = vec3_magnitude(vec3_component_product(d_two, d_one));

  vec3_copy(to_st, vec3_sub(p_one, p_two));
  dp_sta_one = vec3_magnitude(vec3_component_product(d_one, to_st));
  dp_sta_two = vec3_magnitude(vec3_component_product(d_two, to_st));

  denom = sm_one * sm_two - dp_one_two * dp_one_two;

  if (real_abs(denom) < 0.0001f)
    return use_one ? p_one : p_two;

  mua = (dp_one_two * dp_sta_one - sm_two * dp_sta_one) / denom;
  mub = (sm_one * dp_sta_two - dp_one_two * dp_sta_one) / denom;

  if (mua > one_size || mua < -one_size || mub > two_size || mub < -two_size) {
    return use_one ? p_one : p_two;
  } else {
    vec3_copy(c_one, vec3_add(p_one, vec3_mul_scalar(d_one, mua)));
    vec3_copy(c_two, vec3_add(p_two, vec3_mul_scalar(d_two, mub)));

    return vec3_add(vec3_mul_scalar(c_one, 0.5), vec3_mul_scalar(c_two, 0.5));
  }
}

/*// This preprocessor definition is only used as a convenience in the boxAndBox contact generation method.
#define CHECK_OVERLAP(axis, index)                              \
  if (!tryAxis(one, two, (axis), toCentre, (index), pen, best)) \
    return 0;

unsigned CollisionDetector::boxAndBox(const CollisionBox& one, const CollisionBox& two, CollisionData* data) {
  Vector3 toCentre = two.getAxis(3) - one.getAxis(3);

  real pen = REAL_MAX;
  unsigned best = 0xffffff;

  CHECK_OVERLAP(one.getAxis(0), 0);
  CHECK_OVERLAP(one.getAxis(1), 1);
  CHECK_OVERLAP(one.getAxis(2), 2);

  CHECK_OVERLAP(two.getAxis(0), 3);
  CHECK_OVERLAP(two.getAxis(1), 4);
  CHECK_OVERLAP(two.getAxis(2), 5);

  unsigned bestSingleAxis = best;

  CHECK_OVERLAP(one.getAxis(0) % two.getAxis(0), 6);
  CHECK_OVERLAP(one.getAxis(0) % two.getAxis(1), 7);
  CHECK_OVERLAP(one.getAxis(0) % two.getAxis(2), 8);
  CHECK_OVERLAP(one.getAxis(1) % two.getAxis(0), 9);
  CHECK_OVERLAP(one.getAxis(1) % two.getAxis(1), 10);
  CHECK_OVERLAP(one.getAxis(1) % two.getAxis(2), 11);
  CHECK_OVERLAP(one.getAxis(2) % two.getAxis(0), 12);
  CHECK_OVERLAP(one.getAxis(2) % two.getAxis(1), 13);
  CHECK_OVERLAP(one.getAxis(2) % two.getAxis(2), 14);

  assert(best != 0xffffff);

  if (best < 3) {
    fillPointFaceBoxBox(one, two, toCentre, data, best, pen);
    data->addContacts(1);

    return 1;
  } else if (best < 6) {
    fillPointFaceBoxBox(two, one, toCentre * -1.0f, data, best - 3, pen);
    data->addContacts(1);

    return 1;
  } else {
    best -= 6;
    unsigned oneAxisIndex = best / 3;
    unsigned twoAxisIndex = best % 3;
    Vector3 oneAxis = one.getAxis(oneAxisIndex);
    Vector3 twoAxis = two.getAxis(twoAxisIndex);
    Vector3 axis = oneAxis % twoAxis;
    axis.normalise();

    if (axis * toCentre > 0)
      axis = axis * -1.0f;

    Vector3 ptOnOneEdge = one.halfSize;
    Vector3 ptOnTwoEdge = two.halfSize;
    for (unsigned i = 0; i < 3; i++) {
      if (i == oneAxisIndex)
        ptOnOneEdge[i] = 0;
      else if (one.getAxis(i) * axis > 0)
        ptOnOneEdge[i] = -ptOnOneEdge[i];

      if (i == twoAxisIndex)
        ptOnTwoEdge[i] = 0;
      else if (two.getAxis(i) * axis < 0)
        ptOnTwoEdge[i] = -ptOnTwoEdge[i];
    }

    ptOnOneEdge = one.transform * ptOnOneEdge;
    ptOnTwoEdge = two.transform * ptOnTwoEdge;

    Vector3 vertex = contactPoint(ptOnOneEdge, oneAxis, one.halfSize[oneAxisIndex], ptOnTwoEdge, twoAxis, two.halfSize[twoAxisIndex], bestSingleAxis > 2);
    Contact* contact = data->contacts;

    contact->penetration = pen;
    contact->contactNormal = axis;
    contact->contactPoint = vertex;
    contact->setBodyData(one.body, two.body, data->friction, data->restitution);
    data->addContacts(1);
    return 1;
  }
  return 0;
}
#undef CHECK_OVERLAP*/

unsigned int collision_detector_box_and_point(struct CollisionBox* box, real* point, struct CollisionData* data) {
  vec3 rel_pt;
  vec3_copy(rel_pt, mat4_transform_inverse(box->collision_primitive.transform, point));

  vec3 normal;
  real min_depth = box->half_size[0] - real_abs(rel_pt[0]);

  if (min_depth < 0)
    return 0;

  vec3_copy(normal, vec3_mul_scalar(collision_primitive_get_axis(&box->collision_primitive, 0), rel_pt[0] < 0 ? -1 : 1));

  real depth = box->half_size[1] - real_abs(rel_pt[1]);
  if (depth < 0)
    return 0;
  else if (depth < min_depth) {
    min_depth = depth;
    vec3_copy(normal, vec3_mul_scalar(collision_primitive_get_axis(&box->collision_primitive, 1), rel_pt[1] < 0 ? -1 : 1));
  }

  depth = box->half_size[2] - real_abs(rel_pt[2]);
  if (depth < 0)
    return 0;
  else if (depth < min_depth) {
    min_depth = depth;
    vec3_copy(normal, vec3_mul_scalar(collision_primitive_get_axis(&box->collision_primitive, 2), rel_pt[2] < 0 ? -1 : 1));
  }

  struct Contact* contact = data->contacts;
  vec3_copy(contact->contact_normal, normal);
  vec3_copy(contact->contact_point, point);
  contact->penetration = min_depth;

  contact_set_body_data(contact, box->collision_primitive.body, NULL, data->friction, data->restitution);

  collision_data_add_contacts(data, 1);

  return 1;
}

unsigned int collision_detector_box_and_sphere(struct CollisionBox* box, struct CollisionSphere* sphere, struct CollisionData* data) {
  vec3 centre;
  vec3_copy(centre, collision_primitive_get_axis(&sphere->collision_primitive, 3));

  vec3 rel_centre;
  vec3_copy(rel_centre, mat4_transform_inverse(box->collision_primitive.transform, centre));

  if (real_abs(rel_centre[0]) - sphere->radius > box->half_size[0] || real_abs(rel_centre[1]) - sphere->radius > box->half_size[1] || real_abs(rel_centre[2]) - sphere->radius > box->half_size[2])
    return 0;

  vec3 closest_pt = {0.0, 0.0, 0.0};
  real dist;

  dist = rel_centre[0];
  if (dist > box->half_size[0])
    dist = box->half_size[0];
  if (dist < -box->half_size[0])
    dist = -box->half_size[0];
  closest_pt[0] = dist;

  dist = rel_centre[1];
  if (dist > box->half_size[1])
    dist = box->half_size[1];
  if (dist < -box->half_size[1])
    dist = -box->half_size[1];
  closest_pt[1] = dist;

  dist = rel_centre[2];
  if (dist > box->half_size[2])
    dist = box->half_size[2];
  if (dist < -box->half_size[2])
    dist = -box->half_size[2];
  closest_pt[2] = dist;

  dist = vec3_square_magnitude(vec3_sub(closest_pt, rel_centre));
  if (dist > sphere->radius * sphere->radius)
    return 0;

  vec3 closest_pt_world;
  vec3_copy(closest_pt_world, mat4_transform(box->collision_primitive.transform, closest_pt));

  struct Contact* contact = data->contacts;
  vec3_copy(contact->contact_normal, vec3_sub(closest_pt_world, centre));
  vec3_copy(contact->contact_normal, vec3_normalise(contact->contact_normal));
  vec3_copy(contact->contact_point, closest_pt_world);
  contact->penetration = sphere->radius - real_sqrt(dist);
  contact_set_body_data(contact, box->collision_primitive.body, sphere->collision_primitive.body, data->friction, data->restitution);

  collision_data_add_contacts(data, 1);

  return 1;
}

unsigned int collision_detector_box_and_half_space(struct CollisionBox* box, struct CollisionPlane* plane, struct CollisionData* data) {
  if (data->contacts_left <= 0)
    return 0;

  if (!intersection_test_box_and_half_space(box, plane))
    return 0;

  static real mults[8][3] = {{1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {-1, -1, 1}, {1, 1, -1}, {-1, 1, -1}, {1, -1, -1}, {-1, -1, -1}};

  struct Contact* contact = data->contacts;
  unsigned int contacts_used = 0;
  for (unsigned int i = 0; i < 8; i++) {
    vec3 vertex_pos = {mults[i][0], mults[i][1], mults[i][2]};
    vec3_copy(vertex_pos, vec3_component_product(vertex_pos, box->half_size));
    vec3_copy(vertex_pos, mat4_transform(box->collision_primitive.transform, vertex_pos));

    real vertex_distance = vec3_magnitude(vec3_component_product(vertex_pos, plane->direction));

    if (vertex_distance <= plane->offset) {
      vec3_copy(contact->contact_point, plane->direction);
      vec3_copy(contact->contact_point, vec3_mul_scalar(contact->contact_point, vertex_distance - plane->offset));
      vec3_copy(contact->contact_point, vec3_add(contact->contact_point, vertex_pos));
      vec3_copy(contact->contact_normal, plane->direction);
      contact->penetration = plane->offset - vertex_distance;

      contact_set_body_data(contact, box->collision_primitive.body, NULL, data->friction, data->restitution);

      contact++;
      contacts_used++;
      if (contacts_used == (unsigned int)data->contacts_left)
        return contacts_used;
    }
  }

  collision_data_add_contacts(data, contacts_used);

  return contacts_used;
}