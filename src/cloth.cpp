#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.

  // Populate point masses
  double width_dist = width / num_width_points, height_dist = height / num_height_points;
  for (int j = 0; j < num_height_points; j++) {
      for (int i = 0; i < num_width_points; i++) {
          if (this->orientation == HORIZONTAL) {
              float xpos = i * width_dist;
              float zpos = j * height_dist;
              std::vector<int> v = {i, j};
              this->point_masses.emplace_back(PointMass(Vector3D(xpos, 1, zpos), (find(pinned.begin(), pinned.end(), v) != pinned.end())));
          }
          else {
              float xpos = i * width_dist;
              float ypos = j  * height_dist;
              std::vector<int> v = {i, j};
              double rand_z =  -1.0 + (2.0/RAND_MAX)*(rand() + 1) / 1000.0;
              this->point_masses.emplace_back(PointMass(Vector3D(xpos, ypos, rand_z), (find(pinned.begin(), pinned.end(), v) != pinned.end())));

          }
      }
  }

    for (int j = 0; j < num_height_points; j++) {
        for (int i = 0; i < num_width_points; i++) {
            int index = i + j * num_width_points;
           // Check left
           if (i - 1 >= 0) {
               int indexA = (i - 1) + num_width_points * j;
               springs.emplace_back(Spring(&point_masses[index], &point_masses[indexA], STRUCTURAL));
           }
           // Check up
           if (j - 1 >= 0) {
               int indexA = i + num_width_points * (j - 1);
               springs.emplace_back(Spring(&point_masses[index], &point_masses[indexA], STRUCTURAL));
           }
           // Check upleft
            if (i - 1 >= 0 && j - 1 >= 0) {
                int indexA = (i - 1) + num_width_points * (j - 1);
                springs.emplace_back(Spring(&point_masses[index], &point_masses[indexA], SHEARING));
            }
            // Check upright
            if (i + 1 < num_width_points && j - 1 >= 0) {
                int indexA = (i + 1) + num_width_points * (j - 1);
                springs.emplace_back(Spring(&point_masses[index], &point_masses[indexA], SHEARING));
            }
           // Check 2left
            if (i - 2 >= 0) {
                int indexA = (i - 2) + num_width_points * j;
                springs.emplace_back(Spring(&point_masses[index], &point_masses[indexA], BENDING));
            }
            // Check 2up
            if (j - 2 >= 0) {
                int indexA = i + num_width_points * (j - 2);
                springs.emplace_back(Spring(&point_masses[index], &point_masses[indexA], BENDING));
            }
      }
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
    double mass = width * height * cp->density / num_width_points / num_height_points;
    double delta_t = 1.0f / frames_per_sec / simulation_steps;

    // TODO (Part 2): Compute total force acting on each point mass.

    // Clear forces
    for (int j = 0; j < num_height_points; j++) {
        for (int i = 0; i < num_width_points; i++) {
            int index = i + j * num_width_points;
            point_masses[index].forces = 0;
        }
    }

    // Calculate external forces
    for (int j = 0; j < num_height_points; j++) {
        for (int i = 0; i < num_width_points; i++) {
            int index = i + j * num_width_points;
            Vector3D extForce = external_accelerations[0] * mass;
            point_masses[index].forces += extForce;
        }
    }

    for (Spring sp : springs) {
        Vector3D springForce;
        Vector3D posA = sp.pm_a->position;
        Vector3D posB = sp.pm_b->position;

        Vector3D spDirection = posB - posA;
        spDirection.normalize();

        if (cp->enable_bending_constraints && sp.spring_type == BENDING) {
            springForce = spDirection * (cp->ks * .2 * ((posA - posB).norm() - sp.rest_length));
            sp.pm_a->forces += springForce;
            sp.pm_b->forces += -springForce;
        }
        if ((cp->enable_bending_constraints && sp.spring_type == SHEARING) ||
                (cp->enable_bending_constraints && sp.spring_type == STRUCTURAL)) {
            springForce = spDirection * (cp->ks * ((posA - posB).norm() - sp.rest_length));
            sp.pm_a->forces += springForce;
            sp.pm_b->forces += -springForce;
        }
    }

////   TODO (Part 2): Use Verlet integration to compute new point mass positions
    for (int j = 0; j < num_height_points; j++) {
        for (int i = 0; i < num_width_points; i++) {
            int index = i + j * num_width_points;

            if (point_masses[index].pinned) {
                continue;
            }

            Vector3D prevPos = point_masses[index].last_position;
            Vector3D curPos = point_masses[index].position;
            Vector3D newPos = curPos + (1.0 - cp->damping / 100.0) * (curPos - prevPos) + (point_masses[index].forces / mass * delta_t * delta_t);

            point_masses[index].last_position = curPos;
            point_masses[index].position = newPos;
        }
    }

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
    for (int j = 0; j < num_height_points; j++) {
        for (int i = 0; i < num_width_points; i++) {
            self_collide(point_masses[i + j * num_width_points], simulation_steps);
        }
    }

  // TODO (Part 3): Handle collisions with other primitives.
    for (int j = 0; j < num_height_points; j++) {
        for (int i = 0; i < num_width_points; i++) {
            for (int k = 0; k < collision_objects->size(); k++) {
                collision_objects->at(k)->collide(point_masses[j * num_width_points + i]);
            }
        }
    }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].

    for (int j = 0; j < num_height_points; j++) {
        for (int i = 0; i < num_height_points; i++) {
            int index = i + j * num_width_points;
            Vector3D posA = springs[index].pm_a->position;
            Vector3D posB = springs[index].pm_b->position;
            float maxSpring = springs[index].rest_length * 1.1;

            Vector3D dist = posB - posA;
            if (dist.norm() > maxSpring) {
                float diff = dist.norm() - maxSpring;
                dist.normalize();

                if (!springs[index].pm_a->pinned && !springs[index].pm_b->pinned) {
                    springs[index].pm_a->position += dist * (diff/2.0);
                    springs[index].pm_b->position -= dist * (diff/2.0);
                }

                else if (springs[index].pm_a->pinned) {
                    springs[index].pm_b->position -= dist * diff;
                }

                else if (springs[index].pm_b->pinned) {
                    springs[index].pm_a->position += dist * diff;
                }
            }
        }
    }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  vector<PointMass *> *table;
  for (int i = 0; i < point_masses.size(); i++) {
      double key = hash_position(point_masses[i].position);
      if (map.count(key) > 0) {
          table = map[key];
          table->emplace_back(&point_masses[i]);
      } else {
          table = new vector<PointMass *>();
          table->emplace_back(&point_masses[i]);
      }
      map[key] = table;
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  double minDist = 2.0 * thickness;
  int numCorrected = 0;

  Vector3D corrected;
  float index = hash_position(pm.position);

  if (map.count(index) > 0) {
      vector<PointMass *> *v = map[index];
      for (int i = 0; i < v->size(); i++) {
          Vector3D vecPos = v->at(i)->position;
          Vector3D dirVec = vecPos - pm.position;

          double actualDist = dirVec.norm();

          if (minDist > actualDist) {
              if (&pm != v->at(i)) {
                  Vector3D correction = dirVec;
                  correction.normalize();
                  correction = correction * (actualDist - minDist);
                  corrected += correction;

                  numCorrected += 1;
              }
          }
      }
  }
  if (numCorrected > 0) {
      corrected = (corrected / (double) numCorrected) * (simulation_steps / 100);
      pm.position += corrected;
  }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.

  double w = 3.0 * width / num_width_points;
  double h = 3.0 * height / num_height_points;
  double t = max(w, t);

  double truncX = w * floor(pos.x / w);
  double truncY = h * floor(pos.y / h);
  double truncZ = t * floor(pos.z / t);

  return truncX * 17 + truncY * 29 + truncZ * 41;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
