#ifndef POINTMASS_H
#define POINTMASS_H

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"
#include "CGL/vector4D.h"

using namespace CGL;

// Forward declarations
class Halfedge;

struct PointMass {
  PointMass(Vector3D position, bool pinned)
      : pinned(pinned), start_position(position), position(position),
        last_position(position) {}

  Vector3D normal();
  // Vector3D velocity(double delta_t) {
  //   return (position - last_position) / delta_t;
  // }

  // static values
  bool pinned;
  bool smoke_source;
  Vector3D start_position;

  // dynamic values
  Vector3D velocity;
  Vector3D position;
  Vector3D last_position;
  Vector3D forces;
  Vector4D prev_color;
  Vector4D color;

  // mesh reference
  Halfedge *halfedge;
};

#endif /* POINTMASS_H */
