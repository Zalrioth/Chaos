#pragma once
#ifndef JOINTS_H_
#define JOINTS_H_

#include "core/contacts.hpp"

namespace chaos {
class Joint : public ContactGenerator {
public:
    RigidBody* body[2];
    Vector3 position[2];
    real error;
    void set(RigidBody* a, const Vector3& a_pos, RigidBody* b, const Vector3& b_pos, real error);
    unsigned addContact(Contact* contact, unsigned limit) const;
};
}

#endif // JOINTS_H_
