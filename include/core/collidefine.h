#ifndef COLLIDE_FINE_H_
#define COLLIDE_FINE_H_

#include "core/contacts.hpp"
#include <assert.h>
#include <cstdio>
#include <cstdlib>
#include <memory.h>

namespace chaos {
class IntersectionTests;
class CollisionDetector;

class CollisionPrimitive {
public:
    friend class IntersectionTests;
    friend class CollisionDetector;
    RigidBody* body;
    Matrix4 offset;
    void calculateInternals();
    Vector3 getAxis(unsigned index) const
    {
        return transform.getAxisVector(index);
    }
    const Matrix4& getTransform() const
    {
        return transform;
    }

protected:
    Matrix4 transform;
};

class CollisionSphere : public CollisionPrimitive {
public:
    real radius;
};

class CollisionPlane {
public:
    Vector3 direction;
    real offset;
};

class CollisionBox : public CollisionPrimitive {
public:
    Vector3 halfSize;
};

class IntersectionTests {
public:
    static bool sphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane);
    static bool sphereAndSphere(const CollisionSphere& one, const CollisionSphere& two);
    static bool boxAndBox(const CollisionBox& one, const CollisionBox& two);
    static bool boxAndHalfSpace(const CollisionBox& box, const CollisionPlane& plane);
};

struct CollisionData {
    Contact* contactArray;
    Contact* contacts;
    int contactsLeft;
    unsigned contactCount;
    real friction;
    real restitution;
    real tolerance;
    bool hasMoreContacts()
    {
        return contactsLeft > 0;
    }
    void reset(unsigned maxContacts)
    {
        contactsLeft = maxContacts;
        contactCount = 0;
        contacts = contactArray;
    }
    void addContacts(unsigned count)
    {
        contactsLeft -= count;
        contactCount += count;

        contacts += count;
    }
};

class CollisionDetector {
public:
    static unsigned sphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane, CollisionData* data);
    static unsigned sphereAndTruePlane(const CollisionSphere& sphere, const CollisionPlane& plane, CollisionData* data);
    static unsigned sphereAndSphere(const CollisionSphere& one, const CollisionSphere& two, CollisionData* data);
    static unsigned boxAndHalfSpace(const CollisionBox& box, const CollisionPlane& plane, CollisionData* data);
    static unsigned boxAndBox(const CollisionBox& one, const CollisionBox& two, CollisionData* data);
    static unsigned boxAndPoint(const CollisionBox& box, const Vector3& point, CollisionData* data);
    static unsigned boxAndSphere(const CollisionBox& box, const CollisionSphere& sphere, CollisionData* data);
};
}

#endif // COLLIDE_FINE_H_