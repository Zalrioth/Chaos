#include "core/collidefine.h"

using namespace chaos;

void CollisionPrimitive::calculateInternals() {
  transform = body->getTransform() * offset;
}

bool IntersectionTests::sphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane) {
  real ballDistance = plane.direction * sphere.getAxis(3) - sphere.radius;

  return ballDistance <= plane.offset;
}

bool IntersectionTests::sphereAndSphere(const CollisionSphere& one, const CollisionSphere& two) {
  Vector3 midline = one.getAxis(3) - two.getAxis(3);

  return midline.squareMagnitude() < (one.radius + two.radius) * (one.radius + two.radius);
}

static inline real transformToAxis(const CollisionBox& box, const Vector3& axis) {
  return box.halfSize.x * real_abs(axis * box.getAxis(0)) + box.halfSize.y * real_abs(axis * box.getAxis(1)) + box.halfSize.z * real_abs(axis * box.getAxis(2));
}

static inline bool overlapOnAxis(const CollisionBox& one, const CollisionBox& two, const Vector3& axis, const Vector3& toCentre) {
  real oneProject = transformToAxis(one, axis);
  real twoProject = transformToAxis(two, axis);
  real distance = real_abs(toCentre * axis);

  return (distance < oneProject + twoProject);
}

// This preprocessor definition is only used as a convenience
// in the boxAndBox intersection  method.
#define TEST_OVERLAP(axis) overlapOnAxis(one, two, (axis), toCentre)

bool IntersectionTests::boxAndBox(const CollisionBox& one, const CollisionBox& two) {
  Vector3 toCentre = two.getAxis(3) - one.getAxis(3);

  return (
      TEST_OVERLAP(one.getAxis(0)) && TEST_OVERLAP(one.getAxis(1)) && TEST_OVERLAP(one.getAxis(2)) && TEST_OVERLAP(two.getAxis(0)) && TEST_OVERLAP(two.getAxis(1)) && TEST_OVERLAP(two.getAxis(2)) && TEST_OVERLAP(one.getAxis(0) % two.getAxis(0)) && TEST_OVERLAP(one.getAxis(0) % two.getAxis(1)) && TEST_OVERLAP(one.getAxis(0) % two.getAxis(2)) && TEST_OVERLAP(one.getAxis(1) % two.getAxis(0)) && TEST_OVERLAP(one.getAxis(1) % two.getAxis(1)) && TEST_OVERLAP(one.getAxis(1) % two.getAxis(2)) && TEST_OVERLAP(one.getAxis(2) % two.getAxis(0)) && TEST_OVERLAP(one.getAxis(2) % two.getAxis(1)) && TEST_OVERLAP(one.getAxis(2) % two.getAxis(2)));
}
#undef TEST_OVERLAP

bool IntersectionTests::boxAndHalfSpace(const CollisionBox& box, const CollisionPlane& plane) {
  real projectedRadius = transformToAxis(box, plane.direction);
  real boxDistance = plane.direction * box.getAxis(3) - projectedRadius;

  return boxDistance <= plane.offset;
}

unsigned CollisionDetector::sphereAndTruePlane(const CollisionSphere& sphere, const CollisionPlane& plane, CollisionData* data) {
  if (data->contactsLeft <= 0)
    return 0;

  Vector3 position = sphere.getAxis(3);
  real centreDistance = plane.direction * position - plane.offset;

  if (centreDistance * centreDistance > sphere.radius * sphere.radius)
    return 0;

  Vector3 normal = plane.direction;
  real penetration = -centreDistance;
  if (centreDistance < 0) {
    normal *= -1;
    penetration = -penetration;
  }
  penetration += sphere.radius;

  Contact* contact = data->contacts;
  contact->contactNormal = normal;
  contact->penetration = penetration;
  contact->contactPoint = position - plane.direction * centreDistance;
  contact->setBodyData(sphere.body, NULL, data->friction, data->restitution);

  data->addContacts(1);

  return 1;
}

unsigned CollisionDetector::sphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane, CollisionData* data) {
  if (data->contactsLeft <= 0)
    return 0;

  Vector3 position = sphere.getAxis(3);
  real ballDistance = plane.direction * position - sphere.radius - plane.offset;

  if (ballDistance >= 0)
    return 0;

  Contact* contact = data->contacts;
  contact->contactNormal = plane.direction;
  contact->penetration = -ballDistance;
  contact->contactPoint = position - plane.direction * (ballDistance + sphere.radius);
  contact->setBodyData(sphere.body, NULL, data->friction, data->restitution);

  data->addContacts(1);

  return 1;
}

unsigned CollisionDetector::sphereAndSphere(const CollisionSphere& one, const CollisionSphere& two, CollisionData* data) {
  if (data->contactsLeft <= 0)
    return 0;

  Vector3 positionOne = one.getAxis(3);
  Vector3 positionTwo = two.getAxis(3);

  Vector3 midline = positionOne - positionTwo;
  real size = midline.magnitude();

  if (size <= 0.0f || size >= one.radius + two.radius)
    return 0;

  Vector3 normal = midline * (((real)1.0) / size);

  Contact* contact = data->contacts;
  contact->contactNormal = normal;
  contact->contactPoint = positionOne + midline * (real)0.5;
  contact->penetration = (one.radius + two.radius - size);
  contact->setBodyData(one.body, two.body, data->friction, data->restitution);

  data->addContacts(1);

  return 1;
}

static inline real penetrationOnAxis(const CollisionBox& one, const CollisionBox& two, const Vector3& axis, const Vector3& toCentre) {
  real oneProject = transformToAxis(one, axis);
  real twoProject = transformToAxis(two, axis);
  real distance = real_abs(toCentre * axis);

  return oneProject + twoProject - distance;
}

static inline bool tryAxis(const CollisionBox& one, const CollisionBox& two, Vector3 axis, const Vector3& toCentre, unsigned index, real& smallestPenetration, unsigned& smallestCase) {
  if (axis.squareMagnitude() < 0.0001)
    return true;
  axis.normalise();

  real penetration = penetrationOnAxis(one, two, axis, toCentre);

  if (penetration < 0)
    return false;
  if (penetration < smallestPenetration) {
    smallestPenetration = penetration;
    smallestCase = index;
  }
  return true;
}

void fillPointFaceBoxBox(const CollisionBox& one, const CollisionBox& two, const Vector3& toCentre, CollisionData* data, unsigned best, real pen) {
  Contact* contact = data->contacts;

  Vector3 normal = one.getAxis(best);
  if (one.getAxis(best) * toCentre > 0)
    normal = normal * -1.0f;

  Vector3 vertex = two.halfSize;
  if (two.getAxis(0) * normal < 0)
    vertex.x = -vertex.x;
  if (two.getAxis(1) * normal < 0)
    vertex.y = -vertex.y;
  if (two.getAxis(2) * normal < 0)
    vertex.z = -vertex.z;

  contact->contactNormal = normal;
  contact->penetration = pen;
  contact->contactPoint = two.getTransform() * vertex;
  contact->setBodyData(one.body, two.body, data->friction, data->restitution);
}

static inline Vector3 contactPoint(const Vector3& pOne, const Vector3& dOne, real oneSize, const Vector3& pTwo, const Vector3& dTwo, real twoSize, bool useOne) {
  Vector3 toSt, cOne, cTwo;
  real dpStaOne, dpStaTwo, dpOneTwo, smOne, smTwo;
  real denom, mua, mub;

  smOne = dOne.squareMagnitude();
  smTwo = dTwo.squareMagnitude();
  dpOneTwo = dTwo * dOne;

  toSt = pOne - pTwo;
  dpStaOne = dOne * toSt;
  dpStaTwo = dTwo * toSt;

  denom = smOne * smTwo - dpOneTwo * dpOneTwo;

  if (real_abs(denom) < 0.0001f)
    return useOne ? pOne : pTwo;

  mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
  mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

  if (mua > oneSize || mua < -oneSize || mub > twoSize || mub < -twoSize) {
    return useOne ? pOne : pTwo;
  } else {
    cOne = pOne + dOne * mua;
    cTwo = pTwo + dTwo * mub;

    return cOne * 0.5 + cTwo * 0.5;
  }
}

// This preprocessor definition is only used as a convenience
// in the boxAndBox contact generation method.
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
#undef CHECK_OVERLAP

unsigned CollisionDetector::boxAndPoint(const CollisionBox& box, const Vector3& point, CollisionData* data) {
  Vector3 relPt = box.transform.transformInverse(point);
  Vector3 normal;
  real min_depth = box.halfSize.x - real_abs(relPt.x);

  if (min_depth < 0)
    return 0;

  normal = box.getAxis(0) * ((relPt.x < 0) ? -1 : 1);

  real depth = box.halfSize.y - real_abs(relPt.y);
  if (depth < 0)
    return 0;
  else if (depth < min_depth) {
    min_depth = depth;
    normal = box.getAxis(1) * ((relPt.y < 0) ? -1 : 1);
  }

  depth = box.halfSize.z - real_abs(relPt.z);
  if (depth < 0)
    return 0;
  else if (depth < min_depth) {
    min_depth = depth;
    normal = box.getAxis(2) * ((relPt.z < 0) ? -1 : 1);
  }

  Contact* contact = data->contacts;
  contact->contactNormal = normal;
  contact->contactPoint = point;
  contact->penetration = min_depth;

  contact->setBodyData(box.body, NULL, data->friction, data->restitution);

  data->addContacts(1);

  return 1;
}

unsigned CollisionDetector::boxAndSphere(const CollisionBox& box, const CollisionSphere& sphere, CollisionData* data) {
  Vector3 centre = sphere.getAxis(3);
  Vector3 relCentre = box.transform.transformInverse(centre);

  if (real_abs(relCentre.x) - sphere.radius > box.halfSize.x || real_abs(relCentre.y) - sphere.radius > box.halfSize.y || real_abs(relCentre.z) - sphere.radius > box.halfSize.z)
    return 0;

  Vector3 closestPt(0, 0, 0);
  real dist;

  dist = relCentre.x;
  if (dist > box.halfSize.x)
    dist = box.halfSize.x;
  if (dist < -box.halfSize.x)
    dist = -box.halfSize.x;
  closestPt.x = dist;

  dist = relCentre.y;
  if (dist > box.halfSize.y)
    dist = box.halfSize.y;
  if (dist < -box.halfSize.y)
    dist = -box.halfSize.y;
  closestPt.y = dist;

  dist = relCentre.z;
  if (dist > box.halfSize.z)
    dist = box.halfSize.z;
  if (dist < -box.halfSize.z)
    dist = -box.halfSize.z;
  closestPt.z = dist;

  dist = (closestPt - relCentre).squareMagnitude();
  if (dist > sphere.radius * sphere.radius)
    return 0;

  Vector3 closestPtWorld = box.transform.transform(closestPt);

  Contact* contact = data->contacts;
  contact->contactNormal = (closestPtWorld - centre);
  contact->contactNormal.normalise();
  contact->contactPoint = closestPtWorld;
  contact->penetration = sphere.radius - real_sqrt(dist);
  contact->setBodyData(box.body, sphere.body, data->friction, data->restitution);

  data->addContacts(1);

  return 1;
}

unsigned CollisionDetector::boxAndHalfSpace(const CollisionBox& box, const CollisionPlane& plane, CollisionData* data) {
  if (data->contactsLeft <= 0)
    return 0;

  if (!IntersectionTests::boxAndHalfSpace(box, plane))
    return 0;

  static real mults[8][3] = {{1, 1, 1}, {-1, 1, 1}, {1, -1, 1}, {-1, -1, 1}, {1, 1, -1}, {-1, 1, -1}, {1, -1, -1}, {-1, -1, -1}};

  Contact* contact = data->contacts;
  unsigned contactsUsed = 0;
  for (unsigned i = 0; i < 8; i++) {
    Vector3 vertexPos(mults[i][0], mults[i][1], mults[i][2]);
    vertexPos.componentProductUpdate(box.halfSize);
    vertexPos = box.transform.transform(vertexPos);

    real vertexDistance = vertexPos * plane.direction;

    if (vertexDistance <= plane.offset) {
      contact->contactPoint = plane.direction;
      contact->contactPoint *= (vertexDistance - plane.offset);
      contact->contactPoint += vertexPos;
      contact->contactNormal = plane.direction;
      contact->penetration = plane.offset - vertexDistance;

      contact->setBodyData(box.body, NULL,
                           data->friction, data->restitution);

      contact++;
      contactsUsed++;
      if (contactsUsed == (unsigned)data->contactsLeft)
        return contactsUsed;
    }
  }

  data->addContacts(contactsUsed);
  return contactsUsed;
}