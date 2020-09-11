#include "chaos/core/collidefine.h"

float transform_to_axis(struct CollisionBox* box, vec3 axis) {
  return box->half_size.data[0] * fabsf(vec3_magnitude(vec3_component_product(axis, collision_primitive_get_axis(&box->collision_primitive, 0)))) + box->half_size.data[1] * fabsf(vec3_magnitude(vec3_component_product(axis, collision_primitive_get_axis(&box->collision_primitive, 1)))) + box->half_size.data[2] * fabsf(vec3_magnitude(vec3_component_product(axis, collision_primitive_get_axis(&box->collision_primitive, 2))));
}

bool overlap_on_axis(struct CollisionBox* one, struct CollisionBox* two, vec3 axis, vec3 to_centre) {
  float one_project = transform_to_axis(one, axis);
  float two_project = transform_to_axis(two, axis);
  float distance = fabsf(vec3_magnitude(vec3_component_product(to_centre, axis)));

  return (distance < one_project + two_project);
}

void collision_primitive_calculate_internals(struct CollisionPrimitive* collision_primitive) {
  collision_primitive->transform = mat4_mul(collision_primitive->body->transform_matrix, collision_primitive->offset);
}

vec3 collision_primitive_get_axis(struct CollisionPrimitive* collision_primitive, unsigned int index) {
  return mat4_get_axis_vector(collision_primitive->transform, index);
}

bool intersection_test_sphere_and_half_space(struct CollisionSphere* sphere, struct CollisionPlane* plane) {
  float ball_distance = vec3_magnitude(vec3_component_product(plane->direction, collision_primitive_get_axis(&sphere->collision_primitive, 3))) - sphere->radius;

  return ball_distance <= plane->offset;
}

bool intersection_test_sphere_and_sphere(struct CollisionSphere* one, struct CollisionSphere* two) {
  return vec3_square_magnitude(vec3_sub(collision_primitive_get_axis(&one->collision_primitive, 3), collision_primitive_get_axis(&two->collision_primitive, 3))) < (one->radius + two->radius) * (one->radius + two->radius);
}

bool intersection_test_box_and_half_space(struct CollisionBox* box, struct CollisionPlane* plane) {
  float projected_radius = transform_to_axis(box, plane->direction);
  float box_distance = vec3_magnitude(vec3_component_product(plane->direction, collision_primitive_get_axis(&box->collision_primitive, 3))) - projected_radius;

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

  vec3 position = collision_primitive_get_axis(&sphere->collision_primitive, 3);

  float centre_distance = vec3_magnitude(vec3_component_product(plane->direction, position)) - plane->offset;
  if (centre_distance * centre_distance > sphere->radius * sphere->radius)
    return 0;

  vec3 normal = plane->direction;

  float penetration = -centre_distance;
  if (centre_distance < 0) {
    normal = vec3_scale(normal, -1);
    penetration = -penetration;
  }
  penetration += sphere->radius;

  struct Contact* contact = data->contacts;
  contact->contact_normal = normal;
  contact->penetration = penetration;
  contact->contact_point = vec3_sub(position, vec3_scale(plane->direction, centre_distance));
  contact_set_body_data(contact, sphere->collision_primitive.body, NULL, data->friction, data->restitution);

  collision_data_add_contacts(data, 1);

  return 1;
}

unsigned int collision_detector_sphere_and_half_space(struct CollisionSphere* sphere, struct CollisionPlane* plane, struct CollisionData* data) {
  if (data->contacts_left <= 0)
    return 0;

  vec3 position = collision_primitive_get_axis(&sphere->collision_primitive, 3);

  float ball_distance = vec3_magnitude(vec3_component_product(plane->direction, position)) - sphere->radius - plane->offset;
  if (ball_distance >= 0)
    return 0;

  struct Contact* contact = data->contacts;
  contact->contact_normal = plane->direction;
  contact->penetration = -ball_distance;
  contact->contact_point = vec3_sub(position, vec3_scale(plane->direction, ball_distance + sphere->radius));
  contact_set_body_data(contact, sphere->collision_primitive.body, NULL, data->friction, data->restitution);

  collision_data_add_contacts(data, 1);

  return 1;
}

unsigned int collision_detector_sphere_and_sphere(struct CollisionSphere* one, struct CollisionSphere* two, struct CollisionData* data) {
  if (data->contacts_left <= 0)
    return 0;

  vec3 position_one = collision_primitive_get_axis(&one->collision_primitive, 3);
  vec3 position_two = collision_primitive_get_axis(&two->collision_primitive, 3);
  vec3 midline = vec3_sub(position_one, position_two);

  float size = vec3_magnitude(midline);
  if (size <= 0.0f || size >= one->radius + two->radius)
    return 0;

  vec3 normal = vec3_scale(midline, 1.0f / size);

  struct Contact* contact = data->contacts;
  contact->contact_normal = normal;
  contact->contact_point = vec3_add(position_one, vec3_scale(midline, 0.5f));
  contact->penetration = (one->radius + two->radius - size);
  contact_set_body_data(contact, one->collision_primitive.body, two->collision_primitive.body, data->friction, data->restitution);

  collision_data_add_contacts(data, 1);

  return 1;
}

float penetration_on_axis(struct CollisionBox* one, struct CollisionBox* two, vec3 axis, vec3 to_centre) {
  float one_project = transform_to_axis(one, axis);
  float two_project = transform_to_axis(two, axis);
  float distance = fabsf(vec3_magnitude(vec3_component_product(to_centre, axis)));

  return one_project + two_project - distance;
}

bool try_axis(struct CollisionBox* one, struct CollisionBox* two, vec3 axis, vec3 to_centre, unsigned int index, float* smallest_penetration, unsigned int* smallest_case) {
  if (vec3_square_magnitude(axis) < 0.0001)
    return true;

  axis = vec3_normalise(axis);

  float penetration = penetration_on_axis(one, two, axis, to_centre);

  if (penetration < 0)
    return false;
  if (penetration < *smallest_penetration) {
    *smallest_penetration = penetration;
    *smallest_case = index;
  }
  return true;
}

void fill_point_face_box_box(struct CollisionBox* one, struct CollisionBox* two, vec3 to_centre, struct CollisionData* data, unsigned int best, float pen) {
  struct Contact* contact = data->contacts;

  vec3 normal = collision_primitive_get_axis(&one->collision_primitive, best);

  if (vec3_magnitude(vec3_component_product(collision_primitive_get_axis(&one->collision_primitive, best), to_centre)) > 0)
    normal = vec3_scale(normal, -1.0f);

  vec3 vertex = two->half_size;

  if (vec3_magnitude(vec3_component_product(collision_primitive_get_axis(&two->collision_primitive, 0), normal)) < 0)
    vertex.data[0] = -vertex.data[0];
  if (vec3_magnitude(vec3_component_product(collision_primitive_get_axis(&two->collision_primitive, 1), normal)) < 0)
    vertex.data[1] = -vertex.data[1];
  if (vec3_magnitude(vec3_component_product(collision_primitive_get_axis(&two->collision_primitive, 2), normal)) < 0)
    vertex.data[2] = -vertex.data[2];

  contact->contact_normal = normal;
  contact->penetration = pen;
  contact->contact_point = mat4_transform(two->collision_primitive.transform, vertex);
  contact_set_body_data(contact, one->collision_primitive.body, two->collision_primitive.body, data->friction, data->restitution);
}

vec3 contact_point(vec3 p_one, vec3 d_one, float one_size, vec3 p_two, vec3 d_two, float two_size, bool use_one) {
  vec3 to_st, c_one, c_two;
  float dp_sta_one, dp_sta_two, dp_one_two, sm_one, sm_two;
  float denom, mua, mub;

  sm_one = vec3_square_magnitude(d_one);
  sm_two = vec3_square_magnitude(d_two);
  dp_one_two = vec3_magnitude(vec3_component_product(d_two, d_one));

  to_st = vec3_sub(p_one, p_two);
  dp_sta_one = vec3_magnitude(vec3_component_product(d_one, to_st));
  dp_sta_two = vec3_magnitude(vec3_component_product(d_two, to_st));

  denom = sm_one * sm_two - dp_one_two * dp_one_two;

  if (fabsf(denom) < 0.0001f)
    return use_one ? p_one : p_two;

  mua = (dp_one_two * dp_sta_one - sm_two * dp_sta_one) / denom;
  mub = (sm_one * dp_sta_two - dp_one_two * dp_sta_one) / denom;

  if (mua > one_size || mua < -one_size || mub > two_size || mub < -two_size) {
    return use_one ? p_one : p_two;
  } else {
    c_one = vec3_add(p_one, vec3_scale(d_one, mua));
    c_two = vec3_add(p_two, vec3_scale(d_two, mub));

    return vec3_add(vec3_scale(c_one, 0.5), vec3_scale(c_two, 0.5));
  }
}

unsigned int collision_detector_box_and_point(struct CollisionBox* box, vec3 point, struct CollisionData* data) {
  vec3 rel_pt = mat4_transform_inverse(box->collision_primitive.transform, point);

  vec3 normal;
  float min_depth = box->half_size.data[0] - fabsf(rel_pt.data[0]);

  if (min_depth < 0)
    return 0;

  normal = vec3_scale(collision_primitive_get_axis(&box->collision_primitive, 0), rel_pt.data[0] < 0 ? -1 : 1);

  float depth = box->half_size.data[1] - fabsf(rel_pt.data[1]);
  if (depth < 0)
    return 0;
  else if (depth < min_depth) {
    min_depth = depth;
    normal = vec3_scale(collision_primitive_get_axis(&box->collision_primitive, 1), rel_pt.data[1] < 0 ? -1 : 1);
  }

  depth = box->half_size.data[2] - fabsf(rel_pt.data[2]);
  if (depth < 0)
    return 0;
  else if (depth < min_depth) {
    min_depth = depth;
    normal = vec3_scale(collision_primitive_get_axis(&box->collision_primitive, 2), rel_pt.data[2] < 0 ? -1 : 1);
  }

  struct Contact* contact = data->contacts;
  contact->contact_normal = normal;
  contact->contact_point = point;
  contact->penetration = min_depth;

  contact_set_body_data(contact, box->collision_primitive.body, NULL, data->friction, data->restitution);

  collision_data_add_contacts(data, 1);

  return 1;
}

unsigned int collision_detector_box_and_sphere(struct CollisionBox* box, struct CollisionSphere* sphere, struct CollisionData* data) {
  vec3 centre = collision_primitive_get_axis(&sphere->collision_primitive, 3);

  vec3 rel_centre = mat4_transform_inverse(box->collision_primitive.transform, centre);

  if (fabsf(rel_centre.data[0]) - sphere->radius > box->half_size.data[0] || fabsf(rel_centre.data[1]) - sphere->radius > box->half_size.data[1] || fabsf(rel_centre.data[2]) - sphere->radius > box->half_size.data[2])
    return 0;

  vec3 closest_pt = (vec3){.data[0] = 0.0, .data[1] = 0.0, .data[2] = 0.0};
  float dist;

  dist = rel_centre.data[0];
  if (dist > box->half_size.data[0])
    dist = box->half_size.data[0];
  if (dist < -box->half_size.data[0])
    dist = -box->half_size.data[0];
  closest_pt.data[0] = dist;

  dist = rel_centre.data[1];
  if (dist > box->half_size.data[1])
    dist = box->half_size.data[1];
  if (dist < -box->half_size.data[1])
    dist = -box->half_size.data[1];
  closest_pt.data[1] = dist;

  dist = rel_centre.data[2];
  if (dist > box->half_size.data[2])
    dist = box->half_size.data[2];
  if (dist < -box->half_size.data[2])
    dist = -box->half_size.data[2];
  closest_pt.data[2] = dist;

  dist = vec3_square_magnitude(vec3_sub(closest_pt, rel_centre));
  if (dist > sphere->radius * sphere->radius)
    return 0;

  vec3 closest_pt_world = mat4_transform(box->collision_primitive.transform, closest_pt);

  struct Contact* contact = data->contacts;
  contact->contact_normal = vec3_sub(closest_pt_world, centre);
  contact->contact_normal = vec3_normalise(contact->contact_normal);
  contact->contact_point = closest_pt_world;
  contact->penetration = sphere->radius - sqrtf(dist);
  contact_set_body_data(contact, box->collision_primitive.body, sphere->collision_primitive.body, data->friction, data->restitution);

  collision_data_add_contacts(data, 1);

  return 1;
}

unsigned int collision_detector_box_and_half_space(struct CollisionBox* box, struct CollisionPlane* plane, struct CollisionData* data) {
  if (data->contacts_left <= 0)
    return 0;

  if (!intersection_test_box_and_half_space(box, plane))
    return 0;

  static float mults[8][3] = {{1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {-1, -1, 1}, {1, 1, -1}, {-1, 1, -1}, {1, -1, -1}, {-1, -1, -1}};

  struct Contact* contact = data->contacts;
  unsigned int contacts_used = 0;
  for (unsigned int i = 0; i < 8; i++) {
    vec3 vertex_pos = (vec3){.data[0] = mults[i][0], .data[1] = mults[i][1], .data[2] = mults[i][2]};
    vertex_pos = vec3_component_product(vertex_pos, box->half_size);
    vertex_pos = mat4_transform(box->collision_primitive.transform, vertex_pos);

    float vertex_distance = vec3_dot(vertex_pos, plane->direction);

    if (vertex_distance <= plane->offset) {
      contact->contact_point = plane->direction;
      contact->contact_point = vec3_scale(contact->contact_point, vertex_distance - plane->offset);
      contact->contact_point = vec3_add(contact->contact_point, vertex_pos);
      contact->contact_normal = plane->direction;
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