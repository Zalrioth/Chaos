#include <core/collidecoarse.hpp>

using namespace chaos;

BoundingSphere::BoundingSphere(const Vector3& centre, real radius)
{
    BoundingSphere::centre = centre;
    BoundingSphere::radius = radius;
}

BoundingSphere::BoundingSphere(const BoundingSphere& one, const BoundingSphere& two)
{
    Vector3 centreOffset = two.centre - one.centre;
    real distance = centreOffset.squareMagnitude();
    real radiusDiff = two.radius - one.radius;

    if (radiusDiff * radiusDiff >= distance) {
        if (one.radius > two.radius) {
            centre = one.centre;
            radius = one.radius;
        } else {
            centre = two.centre;
            radius = two.radius;
        }
    }

    else {
        distance = real_sqrt(distance);
        radius = (distance + one.radius + two.radius) * ((real)0.5);

        centre = one.centre;
        if (distance > 0) {
            centre += centreOffset * ((radius - one.radius) / distance);
        }
    }
}

bool BoundingSphere::overlaps(const BoundingSphere* other) const
{
    real distanceSquared = (centre - other->centre).squareMagnitude();
    return distanceSquared < (radius + other->radius) * (radius + other->radius);
}

real BoundingSphere::getGrowth(const BoundingSphere& other) const
{
    BoundingSphere newSphere(*this, other);

    return newSphere.radius * newSphere.radius - radius * radius;
}
