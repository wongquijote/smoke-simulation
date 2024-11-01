#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Plane::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with planes.
  double before = dot((pm.last_position - this->point).unit(), this->normal.unit());
  double after = dot((pm.position - this->point).unit(), this->normal.unit());
  if ((before < 0.0 && after >= 0.0) || (before >= 0.0 && after < 0.0)) {
    // Mass crosses plane
    double t = dot((this->point - pm.position), this->normal.unit());
    if (after < 0.0) {
        t = t / this->normal.norm2();
    }
    else {
        t = t / ((-1.0) * this->normal.norm2());
    }
    Vector3D tangent_pt = pm.position + t * this->normal.unit();
    if (before < 0.0) {
      tangent_pt += (-1.0) * SURFACE_OFFSET * this->normal.unit();
    } else {
      tangent_pt += SURFACE_OFFSET * this->normal.unit();
    }
    Vector3D correction = tangent_pt - pm.last_position;
    pm.position = correction * (1.0 - this->friction) + pm.last_position;
  }

}

void Plane::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

  Vector3f sPoint(point.x, point.y, point.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

  positions.col(0) << sPoint + 2 * (sCross + sParallel);
  positions.col(1) << sPoint + 2 * (sCross - sParallel);
  positions.col(2) << sPoint + 2 * (-sCross + sParallel);
  positions.col(3) << sPoint + 2 * (-sCross - sParallel);

  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;

  if (shader.uniform("u_color", false) != -1) {
    shader.setUniform("u_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }

  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}
