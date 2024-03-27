#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
  if ((pm.position - this->origin).norm() <= this->radius) {
    // Mass is inside of the sphere
    Vector3D correction = ((pm.position - this->origin).unit() * this->radius + this->origin) - pm.last_position;
    pm.position = correction * (1.0 - this->friction) + pm.last_position;
  } 
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
