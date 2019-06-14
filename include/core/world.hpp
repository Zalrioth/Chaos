#pragma once
#ifndef WORLD_H_
#define WORLD_H_

#include "core/body.hpp"
#include "core/contacts.hpp"
#include <cstdlib>

namespace chaos {
class World {
    bool calculateIterations;
    struct BodyRegistration {
        RigidBody* body;
        BodyRegistration* next;
    };
    BodyRegistration* firstBody;
    ContactResolver resolver;
    struct ContactGenRegistration {
        ContactGenerator* gen;
        ContactGenRegistration* next;
    };
    ContactGenRegistration* firstContactGen;
    Contact* contacts;
    unsigned maxContacts;

public:
    World(unsigned maxContacts, unsigned iterations = 0);
    ~World();
    unsigned generateContacts();
    void runPhysics(real duration);
    void startFrame();
};
}

#endif // WORLD_H_
