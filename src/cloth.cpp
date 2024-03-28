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

// Adds a spring btw point mass at (x0, y0) and mass1 if the x0 and y0 coordinates are in bounds
void add_spring(int x0, int y0, PointMass* mass1, Cloth* mesh, e_spring_type spring_type) {
    if ((x0 >= 0) && (x0 < mesh->num_width_points) && (y0 >= 0) && (y0 < mesh->num_height_points)) {
        PointMass* mass0 = &(mesh->point_masses[x0 + y0 * mesh->num_width_points]);
        Spring* spring = new Spring(mass0, mass1, spring_type);
        mesh->springs.push_back(*spring);
    }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
    for (int j = 0; j < num_height_points; j++) {
        for (int i = 0; i < num_width_points; i++) {
            double x = width * ((double)i / (double)num_width_points);
            double y = height * ((double)j / (double)num_height_points);
            double z = 1;
            if (this->orientation == HORIZONTAL) {
                z = y;
                y = 1;
            } else {
                z = ((double)rand() / (double)RAND_MAX) * (1.0/1000.0 - (-1.0/1000.0)) - (1.0/1000.0);
            }
            Vector3D pos = Vector3D(x, y, z);
            PointMass* mass = new PointMass(pos, false);
            this->point_masses.push_back(*mass);
        }
    }

    // Check if pinned
    for (int i = 0; i < pinned.size(); i++) {
        int index = pinned[i][0] + num_width_points * pinned[i][1];
        this->point_masses[index].pinned = true;
    }

    // Add springs
    for (int i = 0; i < this->point_masses.size(); i++) {
        PointMass* curr = &(this->point_masses[i]);
        int x = i % this->num_width_points;
        int y = i / this->num_width_points;

        // Structural
        int left_x = x - 1;
        int left_y = y;
        int top_x = x;
        int top_y = y + 1;

        // Shearing
        int upper_left_x = x - 1;
        int upper_left_y = y + 1;
        int upper_right_x = x + 1;
        int upper_right_y = y + 1;

        // Bending
        int two_left_x = x - 2;
        int two_left_y = y;
        int two_up_x = x;
        int two_up_y = y + 2;

        // Add constraints
        add_spring(left_x, left_y, curr, this, STRUCTURAL);
        add_spring(top_x, top_y, curr, this, STRUCTURAL);
        add_spring(upper_left_x, upper_left_y, curr, this, SHEARING);
        add_spring(upper_right_x, upper_right_y, curr, this, SHEARING);
        add_spring(two_left_x, two_left_y, curr, this, BENDING);
        add_spring(two_up_x, two_up_y, curr, this, BENDING);
    }

}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters* cp,
    vector<Vector3D> external_accelerations,
    vector<CollisionObject*>* collision_objects) {
    double mass = width * height * cp->density / num_width_points / num_height_points;
    double delta_t = 1.0f / frames_per_sec / simulation_steps;

    // TODO (Part 2): Compute total force acting on each point mass.
    Vector3D a_ext = 0.0;
    for (int i = 0; i < external_accelerations.size(); i++) {
        a_ext += external_accelerations[i];
    }
    
    Vector3D ext_force = mass * a_ext;
    for (int i = 0; i < this->point_masses.size(); i++) {
        PointMass* curr = &(this->point_masses[i]);
        curr->forces = ext_force;
    }

    for (int i = 0; i < this->springs.size(); i++) {
        Spring* curr = &(this->springs[i]);
        PointMass* mass_a = curr->pm_a;
        PointMass* mass_b = curr->pm_b;
        Vector3D spring_force = cp->ks * ((mass_a->position - mass_b->position).norm() - curr->rest_length) *
            (mass_a->position - mass_b->position).unit();
        if ((cp->enable_structural_constraints && (curr->spring_type == STRUCTURAL)) ||
            (cp->enable_shearing_constraints && (curr->spring_type == SHEARING)) ||
            (cp->enable_bending_constraints && (curr->spring_type == BENDING))) {
            if (curr->spring_type == BENDING) {
                spring_force *= 0.2;
            }
            mass_a->forces -= spring_force;
            mass_b->forces += spring_force;
        }
        
    }
    // TODO (Part 2): Use Verlet integration to compute new point mass positions
    for (int i = 0; i < this->point_masses.size(); i++) {
        PointMass* curr = &(this->point_masses[i]);
        if (curr->pinned) {
            continue;
        }
        
        // Calculate new position
        Vector3D v_dt = curr->position - curr->last_position;
        Vector3D a_t = curr->forces / mass;
        Vector3D new_pos = curr->position + (1.0 - (cp->damping / 100.0)) * v_dt + a_t * (delta_t * delta_t);

        // Update last position
        curr->last_position = curr->position;
        curr->position = new_pos;
    }

    // TODO (Part 4): Handle self-collisions.
    build_spatial_map();
    for (int i = 0; i < this->point_masses.size(); i++) {
      PointMass* curr = &(this->point_masses[i]);
      self_collide(*curr, simulation_steps);
    }

    // TODO (Part 3): Handle collisions with other primitives.
    for (int i = 0; i < this->point_masses.size(); i++) {
      PointMass* curr = &(this->point_masses[i]);
      for (int j = 0; j < collision_objects->size(); j++) {
        CollisionObject* collision = (*collision_objects)[j];
        collision->collide(*curr);
      }
    }


    // TODO (Part 2): Constrain the changes to be such that the spring does not change
    // in length more than 10% per timestep [Provot 1995].
    for (int i = 0; i < this->springs.size(); i++) {
        Spring* curr = &(this->springs[i]);
        PointMass* mass_a = curr->pm_a;
        PointMass* mass_b = curr->pm_b;
        if (mass_a->pinned && mass_b->pinned) {
            continue;
        }

        double dist = (mass_a->position - mass_b->position).norm();
        if (dist > 1.1 * curr->rest_length) {
            Vector3D direction = (mass_a->position - mass_b->position).unit();
            if (mass_a->pinned) {
                mass_b->position = (-1.0) * direction * (1.1 * curr->rest_length) + mass_a->position;
            }
            else if (mass_b->pinned) {
                mass_a->position = direction * (1.1 * curr->rest_length) + mass_b->position;
            }
            else {
                double diff = dist - (1.1 * curr->rest_length);
                mass_a->position += (-1.0) * direction * (diff / 2.0);
                mass_b->position += direction * (diff / 2.0);
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
  for (int i = 0; i < this->point_masses.size(); i++) {
    PointMass* curr = &(this->point_masses[i]);
    float box = hash_position(curr->position);
    vector<PointMass*>* masses;
    if (this->map.count(box) == 0) {
      masses = new vector<PointMass*>;
      masses->push_back(curr);
      this->map[box] = masses;
    } else {
      masses = this->map[box];
      masses->push_back(curr);
    }
    this->map[box] = masses;
  }

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  float box = hash_position(pm.position);
  vector<PointMass*>* masses = this->map[box];
  Vector3D correction = Vector3D();
  int total = 0;
  for (int i = 0; i < masses->size(); i++) {
    PointMass* curr = (*masses)[i];
    if (&(*curr) == &pm) {
      // Same point mass
      continue;
    }
    double dist = (pm.position - curr->position).norm();
    if (dist < 2.0 * this->thickness) {
      correction += (pm.position - curr->position).unit() * (2.0 * this->thickness - dist);
      total += 1;
    }
  }

  if (total != 0) {
      correction = (correction / ((double)total)) / simulation_steps;
  }
  pm.position = pm.position + correction;
  
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  double box_width = 3.0 * width / (double)num_width_points;
  double box_height = 3.0 * height / (double)num_height_points;
  double box_t = max(box_width, box_height);
  int x_hash = floor(pos.x / box_width);
  int y_hash = floor(pos.y / box_height);
  int z_hash = floor(pos.z / box_t);

  float res = x_hash + y_hash + z_hash;
  return res; 
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
