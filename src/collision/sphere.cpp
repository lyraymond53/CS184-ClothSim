#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
  Vector3D pointMass_position = pm.position;
  float pointRadius = (pm.position - this->origin).norm();
  if (pointRadius <= this->radius) {
    Vector3D dir = pm.position - this->origin;
    dir.normalize();

    Vector3D tangentPoint = origin + (dir * this->radius);

    Vector3D correctionDir = tangentPoint - pm.last_position;
    correctionDir.normalize();

    Vector3D corrected = pm.last_position + (1-friction) * (correctionDir * (tangentPoint - pm.last_position).norm());

    pm.position = corrected;
  }

}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
