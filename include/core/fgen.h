#pragma once
#ifndef FGEN_H
#define FGEN_H

#include "core/body.h"

// Swith this to a DOB approach
// TODO: Redo this don't like how this works
enum ForceType { GRAVITY,
                 GAME_STATE };

union Force {
  struct Gravity gravity;
};

struct ForceGenerator {
  enum ForceType force_type;
  void (*update_force)(struct RigidBody*, real);
};

struct Gravity {
  vec3 gravity;
};

gravity_init(struct Gravity* gravity);
void gravity_update_force(strict RigidBody* body, real duration);

class Spring : public ForceGenerator {
  Vector3 connectionPoint;
  Vector3 otherConnectionPoint;
  RigidBody* other;
  real springConstant;
  real restLength;

 public:
  Spring(const Vector3& localConnectionPt, RigidBody* other, const Vector3& otherConnectionPt, real springConstant, real restLength);

  virtual void updateForce(RigidBody* body, real duration);
};

class Explosion : public ForceGenerator, public ParticleForceGenerator {
  real timePassed;

 public:
  Vector3 detonation;
  real implosionMaxRadius;
  real implosionMinRadius;
  real implosionDuration;
  real implosionForce;
  real shockwaveSpeed;
  real shockwaveThickness;
  real peakConcussionForce;
  real concussionDuration;
  real peakConvectionForce;
  real chimneyRadius;
  real chimneyHeight;
  real convectionDuration;

 public:
  Explosion();
  virtual void updateForce(RigidBody* body, real duration);
  virtual void updateForce(Particle* particle, real duration) = 0;
};

class Aero : public ForceGenerator {
 protected:
  Matrix3 tensor;
  Vector3 position;
  const Vector3* windspeed;

 public:
  Aero(const Matrix3& tensor, const Vector3& position, const Vector3* windspeed);
  virtual void updateForce(RigidBody* body, real duration);

 protected:
  void updateForceFromTensor(RigidBody* body, real duration,
                             const Matrix3& tensor);
};

class AeroControl : public Aero {
 protected:
  Matrix3 maxTensor;
  Matrix3 minTensor;
  real controlSetting;

 private:
  Matrix3 getTensor();

 public:
  AeroControl(const Matrix3& base, const Matrix3& min, const Matrix3& max, const Vector3& position, const Vector3* windspeed);
  void setControl(real value);
  virtual void updateForce(RigidBody* body, real duration);
};

class AngledAero : public Aero {
  Quaternion orientation;

 public:
  AngledAero(const Matrix3& tensor, const Vector3& position, const Vector3* windspeed);
  void setOrientation(const Quaternion& quat);
  virtual void updateForce(RigidBody* body, real duration);
};

class Buoyancy : public ForceGenerator {
  real maxDepth;
  real volume;
  real waterHeight;
  real liquidDensity;
  Vector3 centreOfBuoyancy;

 public:
  Buoyancy(const Vector3& cOfB,
           real maxDepth, real volume, real waterHeight,
           real liquidDensity = 1000.0f);

  virtual void updateForce(RigidBody* body, real duration);
};

class ForceRegistry {
 protected:
  struct ForceRegistration {
    RigidBody* body;
    ForceGenerator* fg;
  };

  // NOTE: Use custom data structure for this
  typedef std::vector<ForceRegistration> Registry;
  Registry registrations;

 public:
  void add(RigidBody* body, ForceGenerator* fg);
  void remove(RigidBody* body, ForceGenerator* fg);
  void clear();
  void updateForces(real duration);
};

#endif  // FGEN_H
